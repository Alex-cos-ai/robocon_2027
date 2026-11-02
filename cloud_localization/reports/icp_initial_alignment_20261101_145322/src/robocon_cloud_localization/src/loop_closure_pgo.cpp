#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using namespace std::chrono_literals;

// Pose of the virtual lidar frame in Waffle base_footprint. Its +Z axis points
// down; the sensor is mounted 0.10 m rearward and 0.46 m above the footprint.
static gtsam::Pose3 lidarInBase() {
  return gtsam::Pose3(gtsam::Rot3::Rx(M_PI), gtsam::Point3(-.10, 0, .46));
}

struct Keyframe {
  gtsam::Pose3 wheel_pose;
  gtsam::Pose3 lio_pose;
  Cloud::Ptr cloud{new Cloud};
  std::vector<float> descriptor;
};

class LoopClosurePGO final : public rclcpp::Node {
 public:
  LoopClosurePGO() : Node("scan_context_gicp_isam2") {
    declare_parameter("mode", "map");
    declare_parameter("map_directory", "maps/fastlio_mid360_downward");
    declare_parameter("map_file", "");
    declare_parameter("localization_map_voxel", 0.10);
    declare_parameter("keyframe_distance", 0.55);
    declare_parameter("keyframe_yaw", 0.28);
    declare_parameter("keyframe_min_interval", 0.50);
    declare_parameter("keyframe_voxel", 0.16);
    declare_parameter("scan_context_rings", 20);
    declare_parameter("scan_context_sectors", 60);
    declare_parameter("scan_context_radius", 18.0);
    declare_parameter("loop_separation", 12);
    declare_parameter("loop_similarity", 0.42);
    declare_parameter("gicp_max_correspondence", 0.8);
    declare_parameter("loop_fitness_max", 0.10);
    declare_parameter("loop_inlier_ratio_min", 0.35);
    declare_parameter("localization_fitness_max", 0.35);
    declare_parameter("localization_icp_weight", 0.10);
    declare_parameter("localization_max_correction", 0.35);
    declare_parameter("localization_max_rotation_correction", 0.25);
    declare_parameter("map_publish_rate", 1.0);
    declare_parameter("initial_pose_x", 0.0);
    declare_parameter("initial_pose_y", 0.0);
    declare_parameter("initial_pose_yaw", 0.0);
    mode_ = get_parameter("mode").as_string();
    map_directory_ = get_parameter("map_directory").as_string();
    map_file_ = get_parameter("map_file").as_string();
    rings_ = get_parameter("scan_context_rings").as_int();
    sectors_ = get_parameter("scan_context_sectors").as_int();
    radius_ = get_parameter("scan_context_radius").as_double();
    if (rings_ < 4 || sectors_ < 12 || radius_ <= 1.0 || (mode_ != "map" && mode_ != "localize"))
      throw std::runtime_error("Invalid Scan Context dimensions/radius or mode");
    const auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.5),
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .04,.04,.04,.025,.025,.035).finished()));
    loop_noise_ = robust;
    wheel_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .03,.03,.05,.08,.08,.25).finished());
    lio_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.5),
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .30,.30,.60,1.0,1.0,1.0).finished()));
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .02,.02,.03,.02,.02,.025).finished());
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.05;
    params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
    localization_offset_ = gtsam::Pose3(gtsam::Rot3::Rz(get_parameter("initial_pose_yaw").as_double()),
      gtsam::Point3(get_parameter("initial_pose_x").as_double(),get_parameter("initial_pose_y").as_double(),0));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS(),
      std::bind(&LoopClosurePGO::cloudCallback, this, std::placeholders::_1));
    lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fast_lio/position", 50,
      std::bind(&LoopClosurePGO::lioOdomCallback, this, std::placeholders::_1));
    wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50, std::bind(&LoopClosurePGO::wheelOdomCallback, this, std::placeholders::_1));
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/localization/optimized_map", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/localization/optimized_path", 10);
    pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/pose", 10);
    save_srv_ = create_service<std_srvs::srv::Trigger>("/save_map",
      std::bind(&LoopClosurePGO::saveMap, this, std::placeholders::_1, std::placeholders::_2));
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto period = std::chrono::duration<double>(1.0 / get_parameter("map_publish_rate").as_double());
    map_timer_ = create_wall_timer(period, std::bind(&LoopClosurePGO::publishMap, this));
    if (mode_ == "localize") loadMap();
    const auto map_source = map_file_.empty() ? map_directory_ : map_file_;
    RCLCPP_INFO(get_logger(), "FAST-LIO2 pose graph ready: Scan Context + GICP loop checks + GTSAM/iSAM2; mode=%s, map=%s",
      mode_.c_str(), map_source.c_str());
  }

 private:
  static gtsam::Pose3 fromMsg(const geometry_msgs::msg::Pose &p) {
    const auto &q=p.orientation;
    return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w,q.x,q.y,q.z),gtsam::Point3(p.position.x,p.position.y,p.position.z));
  }
  static geometry_msgs::msg::Pose toMsg(const gtsam::Pose3 &p) {
    geometry_msgs::msg::Pose out;
    out.position.x=p.x(); out.position.y=p.y(); out.position.z=p.z();
    const auto q=p.rotation().toQuaternion();
    out.orientation.x=q.x(); out.orientation.y=q.y(); out.orientation.z=q.z(); out.orientation.w=q.w();
    return out;
  }
  std::vector<float> scanContext(const Cloud &cloud) const {
    std::vector<float> d(rings_*sectors_, -std::numeric_limits<float>::infinity());
    for (const auto &p: cloud.points) {
      // Undo the downward-facing mount before building Scan Context in the
      // level base_footprint frame.
      const double x=p.x-.10;
      const double y=-p.y;
      const double z=.46-p.z;
      const float r=std::hypot(x,y);
      if (!std::isfinite(r)||!std::isfinite(z)||r<.2||r>=radius_) continue;
      int ring=std::min(rings_-1,int(std::floor(r/radius_*rings_)));
      double a=std::atan2(y,x); if(a<0) a+=2*M_PI;
      int sector=std::min(sectors_-1,int(std::floor(a/(2*M_PI)*sectors_)));
      d[ring*sectors_+sector]=std::max(d[ring*sectors_+sector],static_cast<float>(z));
    }
    for(float &v:d) if(!std::isfinite(v)) v=0.f;
    return d;
  }
  double contextDistance(const std::vector<float> &a,const std::vector<float> &b) const {
    double best=1.0;
    for(int shift=0;shift<sectors_;++shift) {
      double dot=0,aa=0,bb=0;
      for(int r=0;r<rings_;++r) for(int s=0;s<sectors_;++s) {
        const double x=a[r*sectors_+s], y=b[r*sectors_+(s+shift)%sectors_];
        dot+=x*y; aa+=x*x; bb+=y*y;
      }
      const double sim=(aa>1e-9&&bb>1e-9)?dot/std::sqrt(aa*bb):0;
      best=std::min(best,1-sim);
    }
    return best;
  }
  Cloud::Ptr downsample(const Cloud::ConstPtr &cloud) const {
    Cloud::Ptr out(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZI> grid;
    const float leaf=get_parameter("keyframe_voxel").as_double();
    grid.setInputCloud(cloud); grid.setLeafSize(leaf,leaf,leaf); grid.filter(*out);
    return out;
  }
  void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_lio_=fromMsg(msg->pose.pose); latest_stamp_=msg->header.stamp; have_lio_pose_=true;
    publishPose(msg->header.stamp);
  }
  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_wheel_=fromMsg(msg->pose.pose); have_wheel_pose_=true;
  }
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_lio_pose_ || !have_wheel_pose_ || msg->data.empty()) return;
    Cloud::Ptr input(new Cloud);
    // MID360 clouds call the float intensity field `reflectivity`; PCL's
    // PointXYZI converter looks for `intensity`. Keep the binary layout intact
    // and alias the field name before conversion.
    auto pcl_msg = *msg;
    for (auto &field : pcl_msg.fields) {
      if (field.name == "reflectivity") {
        field.name = "intensity";
        break;
      }
    }
    pcl::fromROSMsg(pcl_msg,*input);
    if (input->size()<80) return;
    auto cloud=downsample(input);
    if(cloud->size()<50) return;
    if(mode_=="localize") { localize(cloud,msg->header.stamp); return; }
    if (last_keyframe_) {
      const rclcpp::Time current_stamp(msg->header.stamp);
      const rclcpp::Time previous_stamp(last_keyframe_stamp_);
      if ((current_stamp-previous_stamp).seconds() < get_parameter("keyframe_min_interval").as_double())
        return;
    }
    const auto lidar_in_base=lidarInBase();
    const auto wheel_lidar=latest_wheel_.compose(lidar_in_base);
    if (!have_wheel_anchor_) { wheel_anchor_=wheel_lidar; have_wheel_anchor_=true; }
    if (!have_lio_anchor_) { lio_anchor_=latest_lio_; have_lio_anchor_=true; }
    const auto wheel_pose=wheel_anchor_.between(wheel_lidar);
    const auto lio_pose=lio_anchor_.between(latest_lio_);
    const double moved=last_keyframe_?keyframes_.back().wheel_pose.between(wheel_pose).translation().norm():1e9;
    const Eigen::Vector3d turn_rpy=last_keyframe_?keyframes_.back().wheel_pose.between(wheel_pose).rotation().rpy():Eigen::Vector3d(1e9,0,0);
    const double turned=turn_rpy.norm();
    if(last_keyframe_&&moved<get_parameter("keyframe_distance").as_double()&&turned<get_parameter("keyframe_yaw").as_double())return;
    Keyframe kf; kf.wheel_pose=wheel_pose; kf.lio_pose=lio_pose; kf.cloud=cloud; kf.descriptor=scanContext(*cloud);
    const size_t index=keyframes_.size();
    if(index==0) {
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0,kf.wheel_pose,prior_noise_));
      initials_.insert(0,kf.wheel_pose);
      isam_->update(graph_,initials_); graph_.resize(0); initials_.clear();
    } else {
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(index-1,index,
        keyframes_.back().wheel_pose.between(kf.wheel_pose),wheel_noise_));
      // Keep FAST-LIO2 scan-to-map motion as a robust secondary constraint.
      // Wheel encoder odometry anchors the simulation trajectory where planar
      // field scans make the inertial lidar frontend temporarily degenerate.
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(index-1,index,
        keyframes_.back().lio_pose.between(kf.lio_pose),lio_noise_));
      initials_.insert(index,kf.wheel_pose);
      isam_->update(graph_,initials_); graph_.resize(0); initials_.clear();
      detectLoop(index,kf);
    }
    keyframes_.push_back(std::move(kf)); last_keyframe_=true; last_keyframe_stamp_=msg->header.stamp;
    optimized_=isam_->calculateEstimate();
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"FAST-LIO2 keyframes=%zu loop closures=%zu",keyframes_.size(),loop_count_);
    publishPath();
  }
  void detectLoop(size_t current,const Keyframe &kf) {
    if(current<=static_cast<size_t>(get_parameter("loop_separation").as_int())) return;
    double best=get_parameter("loop_similarity").as_double();
    size_t candidate=current, nearest=0;
    double nearest_distance=std::numeric_limits<double>::infinity();
    for(size_t i=0;i+get_parameter("loop_separation").as_int()<current;++i) {
      const double d=contextDistance(kf.descriptor,keyframes_[i].descriptor);
      if(d<nearest_distance){nearest_distance=d;nearest=i;}
      if(d<best){best=d;candidate=i;}
    }
    if(candidate==current) {
      RCLCPP_INFO(get_logger(),"No Scan Context loop candidate at keyframe %zu: nearest=%zu distance=%.3f threshold=%.3f",
        current,nearest,nearest_distance,get_parameter("loop_similarity").as_double());
      return;
    }
    const auto predicted=keyframes_[candidate].wheel_pose.between(kf.wheel_pose);
    const Eigen::Matrix4f guess=predicted.matrix().cast<float>();
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> gicp;
    gicp.setMaximumIterations(45);
    gicp.setMaxCorrespondenceDistance(get_parameter("gicp_max_correspondence").as_double());
    gicp.setTransformationEpsilon(1e-5); gicp.setEuclideanFitnessEpsilon(1e-5);
    gicp.setInputSource(kf.cloud); gicp.setInputTarget(keyframes_[candidate].cloud);
    Cloud aligned; gicp.align(aligned,guess);
    if(!gicp.hasConverged()) {
      RCLCPP_INFO(get_logger(),"GICP rejected loop candidate %zu->%zu: optimizer did not converge (Scan Context %.3f)",candidate,current,best);
      return;
    }
    const double fitness=gicp.getFitnessScore(get_parameter("gicp_max_correspondence").as_double());
    pcl::search::KdTree<pcl::PointXYZI> tree;
    tree.setInputCloud(keyframes_[candidate].cloud);
    size_t inliers=0;
    for(const auto &p:aligned.points){std::vector<int> ids(1);std::vector<float> ds(1);if(tree.nearestKSearch(p,1,ids,ds)>0&&ds[0]<std::pow(get_parameter("gicp_max_correspondence").as_double(),2))++inliers;}
    const double overlap=aligned.empty()?0.0:double(inliers)/aligned.size();
    if(fitness>get_parameter("loop_fitness_max").as_double()||overlap<get_parameter("loop_inlier_ratio_min").as_double()){
      RCLCPP_INFO(get_logger(),"GICP rejected loop %zu->%zu: Scan Context %.3f, fitness %.3fm (limit %.3fm), overlap %.0f%% (minimum %.0f%%)",
        candidate,current,best,fitness,get_parameter("loop_fitness_max").as_double(),overlap*100,get_parameter("loop_inlier_ratio_min").as_double()*100);return;
    }
    const gtsam::Pose3 relative(Eigen::Matrix4d(gicp.getFinalTransformation().cast<double>()));
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(candidate,current,relative,loop_noise_));
    isam_->update(graph_); graph_.resize(0); ++loop_count_;
    RCLCPP_WARN(get_logger(),"Accepted Scan Context loop %zu->%zu (descriptor=%.3f GICP=%.3fm overlap=%.0f%%); iSAM2 updated",candidate,current,best,fitness,overlap*100);
  }
  Eigen::Matrix4f poseMatrix(size_t i) const { return optimized_.at<gtsam::Pose3>(i).matrix().cast<float>(); }
  void localize(const Cloud::Ptr &cloud,const builtin_interfaces::msg::Time &stamp) {
    if(map_cloud_->empty())return;
    if(!localization_started_){
      localization_wheel_previous_=latest_wheel_;
      optimized_localization_=localization_offset_;
      localization_started_=true;
    }
    // Propagate the last map pose by the latest wheel-odometry increment.
    // This keeps the estimate continuous while the planar field offers weak
    // constraints along the floor.
    const auto lidar_in_base=lidarInBase();
    const auto base_motion=localization_wheel_previous_.between(latest_wheel_);
    localization_wheel_previous_=latest_wheel_;
    const auto lidar_motion=lidar_in_base.inverse().compose(base_motion).compose(lidar_in_base);
    const auto predicted=optimized_localization_.compose(lidar_motion);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> gicp;
    gicp.setMaximumIterations(35); gicp.setMaxCorrespondenceDistance(.65);
    gicp.setInputSource(cloud); gicp.setInputTarget(map_cloud_);
    Eigen::Matrix4f guess=predicted.matrix().cast<float>();
    Cloud aligned; gicp.align(aligned,guess);
    const double fitness=gicp.getFitnessScore(.65);
    const double fitness_limit=get_parameter("localization_fitness_max").as_double();
    if(!gicp.hasConverged()||fitness>fitness_limit) {
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "GICP localization rejected: converged=%s fitness=%.3fm threshold=%.3fm source=%zu map=%zu; using wheel prediction",
        gicp.hasConverged()?"true":"false",fitness,fitness_limit,cloud->size(),map_cloud_->size());
      optimized_localization_=predicted;
      publishLocalizationPose(stamp,1.0);
      return;
    }
    const gtsam::Pose3 icp_pose(Eigen::Matrix4d(gicp.getFinalTransformation().cast<double>()));
    const gtsam::Vector6 correction=predicted.localCoordinates(icp_pose);
    const double rotation_error=correction.head<3>().norm();
    const double translation_error=correction.tail<3>().norm();
    if(translation_error>get_parameter("localization_max_correction").as_double()||
       rotation_error>get_parameter("localization_max_rotation_correction").as_double()) {
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "GICP localization correction rejected: fitness=%.3fm delta=%.3fm rotation=%.3frad; using wheel prediction",
        fitness,translation_error,rotation_error);
      optimized_localization_=predicted;
      publishLocalizationPose(stamp,1.0);
      return;
    }
    const double icp_weight=std::clamp(get_parameter("localization_icp_weight").as_double(),0.0,1.0);
    optimized_localization_=predicted.retract(icp_weight*correction);
    publishLocalizationPose(stamp,fitness);
  }
  void publishLocalizationPose(const builtin_interfaces::msg::Time &stamp,double variance) {
    nav_msgs::msg::Odometry pose; pose.header.stamp=stamp; pose.header.frame_id="map"; pose.child_frame_id="mid360_link";
    pose.pose.pose=toMsg(optimized_localization_);
    const double position_variance=std::max(1e-4,variance);
    pose.pose.covariance[0]=position_variance;pose.pose.covariance[7]=position_variance;
    pose.pose.covariance[35]=std::max(1e-4,position_variance/2);
    pose_pub_->publish(pose);
    publishMapToOdom(stamp, optimized_localization_);
    // FAST-LIO already publishes lidar_init -> lidar_link. Publish only the
    // map-to-initial-frame correction here so composing the TF tree lands on
    // the ICP pose instead of applying FAST-LIO's drift a second time.
    const auto map_to_lidar_init=optimized_localization_.compose(latest_lio_.inverse());
    const Eigen::Matrix4d fused=map_to_lidar_init.matrix();
    geometry_msgs::msg::TransformStamped tf; tf.header=pose.header; tf.child_frame_id="lidar_init";
    tf.transform.translation.x=fused(0,3);tf.transform.translation.y=fused(1,3);tf.transform.translation.z=fused(2,3);
    Eigen::Quaterniond q(fused.block<3,3>(0,0));tf.transform.rotation.x=q.x();tf.transform.rotation.y=q.y();tf.transform.rotation.z=q.z();tf.transform.rotation.w=q.w();tf_pub_->sendTransform(tf);
  }
  void publishMapToOdom(const builtin_interfaces::msg::Time &stamp,
                        const gtsam::Pose3 &map_lidar) {
    if (!have_wheel_pose_) return;
    // latest_wheel_ is the odom-frame base pose. Convert it to the simulated
    // lidar origin, then derive the one map -> odom correction that makes the
    // standard odom -> base_footprint -> mid360 chain agree with the graph.
    const auto odom_lidar = latest_wheel_.compose(lidarInBase());
    const auto map_odom = map_lidar.compose(odom_lidar.inverse());
    const auto t = map_odom.translation();
    const auto q = map_odom.rotation().toQuaternion();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_pub_->sendTransform(tf);
  }
  void publishPose(const builtin_interfaces::msg::Time &stamp) {
    if(mode_!="map"||keyframes_.empty()||optimized_.size()!=keyframes_.size()||
       !have_wheel_pose_||!have_wheel_anchor_)return;

    // FAST-LIO's live lidar_init -> lidar_link transform can drift on this
    // mostly planar, downward-facing simulated scan. Keep the optimized graph
    // pose at the last keyframe, then propagate the live chassis pose from
    // that keyframe using the same wheel odometry used by the graph. Finally,
    // cancel the raw FAST-LIO transform so the composed map -> base TF follows
    // the simulated odometry and stops when the Gazebo robot stops.
    const auto wheel_lidar=latest_wheel_.compose(lidarInBase());
    const auto wheel_pose=wheel_anchor_.between(wheel_lidar);
    const auto wheel_delta=keyframes_.back().wheel_pose.between(wheel_pose);
    const auto target_pose=optimized_.at<gtsam::Pose3>(keyframes_.size()-1).compose(wheel_delta);
    const auto map_to_lidar_init=target_pose.compose(latest_lio_.inverse());
    publishMapToOdom(stamp, target_pose);
    const auto t=map_to_lidar_init.translation();const auto q=map_to_lidar_init.rotation().toQuaternion();
    geometry_msgs::msg::TransformStamped tf; tf.header.stamp=stamp;tf.header.frame_id="map";tf.child_frame_id="lidar_init";
    tf.transform.translation.x=t.x();tf.transform.translation.y=t.y();tf.transform.translation.z=t.z();
    tf.transform.rotation.x=q.x();tf.transform.rotation.y=q.y();tf.transform.rotation.z=q.z();tf.transform.rotation.w=q.w();tf_pub_->sendTransform(tf);
  }
  void publishPath() {
    nav_msgs::msg::Path path; path.header.stamp=now();path.header.frame_id="map";
    for(size_t i=0;i<keyframes_.size();++i){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose=toMsg(optimized_.at<gtsam::Pose3>(i));path.poses.push_back(p);}
    path_pub_->publish(path);
  }
  void publishMap() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(mode_=="localize"&&!map_cloud_->empty()){
      sensor_msgs::msg::PointCloud2 loaded;pcl::toROSMsg(*map_cloud_,loaded);loaded.header.stamp=now();loaded.header.frame_id="map";map_pub_->publish(loaded);return;
    }
    if(mode_!="map"||optimized_.size()!=keyframes_.size()||keyframes_.empty())return;
    Cloud combined;
    for(size_t i=0;i<keyframes_.size();++i){Cloud transformed;pcl::transformPointCloud(*keyframes_[i].cloud,transformed,poseMatrix(i));combined+=transformed;}
    sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(combined,msg);msg.header.stamp=now();msg.header.frame_id="map";map_pub_->publish(msg);
    publishPose(msg.header.stamp);
  }
  void saveMap(const std_srvs::srv::Trigger::Request::SharedPtr,const std_srvs::srv::Trigger::Response::SharedPtr res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(keyframes_.size()<3||optimized_.size()!=keyframes_.size()){res->success=false;res->message="Need at least 3 FAST-LIO2 keyframes before saving.";return;}
    namespace fs=std::filesystem;
    fs::path dir(map_directory_);
    fs::create_directories(dir);
    // A previous save may contain more keyframes than this run. Remove the
    // directory before writing so the map bundle cannot expose stale scans.
    fs::remove_all(dir/"keyframes");
    fs::create_directories(dir/"keyframes");
    Cloud combined; std::ofstream meta(dir/"keyframes.txt");
    meta<<"# index followed by row-major 4x4 optimized lidar pose; each keyframe PCD is stored in local lidar coordinates\n"<<std::setprecision(12);
    for(size_t i=0;i<keyframes_.size();++i){
      Cloud transformed;pcl::transformPointCloud(*keyframes_[i].cloud,transformed,poseMatrix(i));combined+=transformed;
      const auto file=dir/"keyframes"/(std::to_string(i)+".pcd");
      pcl::io::savePCDFileBinary(file.string(),*keyframes_[i].cloud);
      const auto m=optimized_.at<gtsam::Pose3>(i).matrix();meta<<i;for(int r=0;r<4;++r)for(int c=0;c<4;++c)meta<<' '<<m(r,c);meta<<'\n';
    }
    pcl::VoxelGrid<pcl::PointXYZI> voxel;voxel.setInputCloud(combined.makeShared());voxel.setLeafSize(.05,.05,.05);
    Cloud filtered;voxel.filter(filtered);
    const auto pcd=dir/"map.pcd";const int result=pcl::io::savePCDFileBinary(pcd.string(),filtered);
    std::ofstream info(dir/"map_info.txt");info<<"frontend=FAST-LIO2 (IKFoM + ikd-Tree)\nloop_detection=Scan Context\nloop_verification=GICP fitness and overlap\npose_graph=GTSAM iSAM2, wheel-odometry factors plus robust FAST-LIO2 relative factors\nlidar_mount_translation_base_footprint=-0.10,0,0.46\nlidar_mount_rpy_base_footprint=3.1415926536,0,0\nkeyframes="<<keyframes_.size()<<"\nloop_closures="<<loop_count_<<"\npoints="<<filtered.size()<<"\nframe=map, anchored at the initial lidar pose\n";
    res->success=result==0;res->message=std::string(res->success?"Saved ":"Failed to write ")+pcd.string()+" ("+std::to_string(filtered.size())+" points, "+std::to_string(keyframes_.size())+" keyframes, "+std::to_string(loop_count_)+" verified loops)";
  }
  void loadMap() {
    namespace fs=std::filesystem;
    const fs::path file=map_file_.empty()?fs::path(map_directory_)/"map.pcd":fs::path(map_file_);
    if(!fs::is_regular_file(file)) throw std::runtime_error("Map PCD not found: "+file.string());
    Cloud::Ptr source(new Cloud);
    if(pcl::io::loadPCDFile(file.string(),*source)<0) throw std::runtime_error("Unable to load map PCD: "+file.string());
    if(source->empty()) throw std::runtime_error("Map PCD is empty: "+file.string());
    const double leaf=get_parameter("localization_map_voxel").as_double();
    if(leaf<=0.0) throw std::runtime_error("localization_map_voxel must be positive");
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(source);
    voxel.setLeafSize(static_cast<float>(leaf),static_cast<float>(leaf),static_cast<float>(leaf));
    voxel.filter(*map_cloud_);
    if(map_cloud_->empty()) throw std::runtime_error("Map PCD has no finite points after voxel filtering: "+file.string());
    RCLCPP_INFO(get_logger(),"Loaded map %s: %zu raw points, %zu after %.2fm voxel filtering",
      file.c_str(),source->size(),map_cloud_->size(),leaf);
  }
  std::string mode_,map_directory_,map_file_;int rings_=20,sectors_=60;double radius_=18;
  std::mutex mutex_;bool have_lio_pose_=false,have_wheel_pose_=false,have_wheel_anchor_=false,have_lio_anchor_=false,last_keyframe_=false;size_t loop_count_=0;
  gtsam::Pose3 latest_lio_,latest_wheel_,lio_anchor_,wheel_anchor_,localization_wheel_previous_,optimized_localization_,localization_offset_;bool localization_started_=false;builtin_interfaces::msg::Time latest_stamp_;
  std::vector<Keyframe> keyframes_;Cloud::Ptr map_cloud_{new Cloud};
  gtsam::NonlinearFactorGraph graph_;gtsam::Values initials_,optimized_;
  std::unique_ptr<gtsam::ISAM2> isam_;gtsam::SharedNoiseModel loop_noise_,wheel_noise_,lio_noise_,prior_noise_;
  builtin_interfaces::msg::Time last_keyframe_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_,wheel_odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LoopClosurePGO>());rclcpp::shutdown();return 0;}
