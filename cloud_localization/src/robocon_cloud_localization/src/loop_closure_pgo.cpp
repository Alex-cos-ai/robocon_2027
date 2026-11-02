#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "global_relocalization.hpp"
#include "mapping_registration.hpp"
#include "stamped_pose_buffer.hpp"

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using namespace std::chrono_literals;

// Pose of the virtual lidar frame in Waffle base_footprint. Its +Z axis points
// down; the sensor is mounted 0.10 m rearward and 0.46 m above the footprint.
static gtsam::Pose3 lidarInBase() {
  return gtsam::Pose3(gtsam::Rot3::Rx(M_PI), gtsam::Point3(-.10, 0, .46));
}

struct Keyframe {
  int64_t stamp = 0;
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
    declare_parameter("loop_fitness_max", 0.01);
    declare_parameter("loop_inlier_ratio_min", 0.70);
    declare_parameter("mapping_sync_max_gap", 0.20);
    declare_parameter("mapping_local_registration", true);
    declare_parameter("mapping_local_submap_frames", 5);
    declare_parameter("mapping_local_fitness_max", 0.01);
    declare_parameter("mapping_local_inlier_ratio_min", 0.70);
    declare_parameter("mapping_local_max_correction", 0.30);
    declare_parameter("mapping_local_max_rotation", 0.15);
    declare_parameter("loop_search_radius", 2.5);
    declare_parameter("loop_min_time", 15.0);
    declare_parameter("loop_candidates", 3);
    declare_parameter("loop_submap_frames", 2);
    declare_parameter("loop_inlier_distance", 0.20);
    declare_parameter("loop_max_correction", 1.0);
    declare_parameter("loop_max_rotation", 0.35);
    declare_parameter("loop_consistent_frames", 2);
    declare_parameter("loop_consistency_translation", 0.20);
    declare_parameter("loop_consistency_rotation", 0.10);
    declare_parameter("loop_min_interval", 3.0);
    declare_parameter("loop_ambiguity_ratio", 1.25);
    declare_parameter("localization_fitness_max", 0.35);
    declare_parameter("localization_icp_weight", 0.10);
    declare_parameter("localization_max_correction", 0.35);
    declare_parameter("localization_max_rotation_correction", 0.25);
    declare_parameter("localization_initial_max_correction", 1.0);
    declare_parameter("localization_initial_max_rotation_correction", 0.35);
    declare_parameter("localization_initial_fitness_max", 0.01);
    declare_parameter("localization_initial_inlier_distance", 0.20);
    declare_parameter("localization_initial_inlier_ratio_min", 0.60);
    declare_parameter("localization_initial_consistent_frames", 3);
    declare_parameter("localization_initial_consistency_translation", 0.10);
    declare_parameter("localization_initial_consistency_rotation", 0.05);
    declare_parameter("localization_global_enabled", true);
    declare_parameter("localization_global_backend", "bbs_gpu_gicp");
    declare_parameter("localization_global_failure_frames", 10);
    declare_parameter("localization_global_retry_seconds", 5.0);
    declare_parameter("localization_global_voxel", 0.25);
    declare_parameter("localization_global_xy_step", 0.50);
    declare_parameter("localization_global_z_step", 0.50);
    declare_parameter("localization_global_yaw_step", 0.2617993878);
    declare_parameter("localization_global_refine_candidates", 32);
    declare_parameter("localization_global_ambiguity_ratio", 1.25);
    declare_parameter("localization_bbs_resolution", 0.20);
    declare_parameter("localization_bbs_levels", 5);
    declare_parameter("localization_bbs_scan_voxel", 0.25);
    declare_parameter("localization_bbs_roll_pitch_window", 0.035);
    declare_parameter("localization_bbs_score_threshold", 0.60);
    declare_parameter("localization_bbs_timeout_ms", 3000);
    declare_parameter("localization_bbs_cpu_threads", 2);
    declare_parameter("localization_bbs_gpu_batch_size", 10000);
    declare_parameter("map_publish_rate", 1.0);
    declare_parameter("initial_pose_x", 0.0);
    declare_parameter("initial_pose_y", 0.0);
    declare_parameter("initial_pose_z", 0.0);
    declare_parameter("initial_pose_roll", 0.0);
    declare_parameter("initial_pose_pitch", 0.0);
    declare_parameter("initial_pose_yaw", 0.0);
    mode_ = get_parameter("mode").as_string();
    map_directory_ = get_parameter("map_directory").as_string();
    map_file_ = get_parameter("map_file").as_string();
    rings_ = get_parameter("scan_context_rings").as_int();
    sectors_ = get_parameter("scan_context_sectors").as_int();
    radius_ = get_parameter("scan_context_radius").as_double();
    if (rings_ < 4 || sectors_ < 12 || radius_ <= 1.0 || (mode_ != "map" && mode_ != "localize"))
      throw std::runtime_error("Invalid Scan Context dimensions/radius or mode");
    for (const auto *name : {"localization_initial_max_correction",
         "localization_initial_max_rotation_correction", "localization_initial_fitness_max",
         "localization_initial_inlier_distance", "localization_initial_consistency_translation",
         "localization_initial_consistency_rotation", "localization_global_retry_seconds",
         "localization_global_voxel", "localization_global_xy_step", "localization_global_z_step",
         "localization_global_yaw_step", "localization_global_ambiguity_ratio"}) {
      const double value = get_parameter(name).as_double();
      if (!std::isfinite(value) || value <= 0.0)
        throw std::runtime_error(std::string(name) + " must be finite and positive");
    }
    const double initial_overlap = get_parameter("localization_initial_inlier_ratio_min").as_double();
    if (!std::isfinite(initial_overlap) || initial_overlap <= 0.0 || initial_overlap > 1.0 ||
        get_parameter("localization_initial_consistent_frames").as_int() < 1)
      throw std::runtime_error("Invalid localization initialization overlap or frame count");
    if (get_parameter("localization_global_failure_frames").as_int()<1 ||
        get_parameter("localization_global_refine_candidates").as_int()<1 ||
        get_parameter("localization_global_yaw_step").as_double()>M_PI ||
        get_parameter("localization_global_ambiguity_ratio").as_double()<1.0)
      throw std::runtime_error("Invalid global relocalization search parameters");
    const auto backend=get_parameter("localization_global_backend").as_string();
    if(backend!="grid_gicp" && backend!="bbs_cpu_gicp" && backend!="bbs_gpu_gicp")
      throw std::runtime_error("localization_global_backend must be grid_gicp, bbs_cpu_gicp or bbs_gpu_gicp");
    for(const auto *key:{"localization_bbs_resolution","localization_bbs_scan_voxel",
                         "localization_bbs_roll_pitch_window"}) {
      const double value=get_parameter(key).as_double();
      if(!std::isfinite(value)||value<=0.0)throw std::runtime_error(std::string(key)+" must be finite and positive");
    }
    const double score=get_parameter("localization_bbs_score_threshold").as_double();
    if(!std::isfinite(score)||score<=0.0||score>=1.0 ||
       get_parameter("localization_bbs_levels").as_int()<1||get_parameter("localization_bbs_levels").as_int()>8||
       get_parameter("localization_bbs_timeout_ms").as_int()<1||
       get_parameter("localization_bbs_cpu_threads").as_int()<1||
       get_parameter("localization_bbs_gpu_batch_size").as_int()<1)
      throw std::runtime_error("Invalid BBS search parameters");
    for(const auto *key:{"mapping_sync_max_gap","mapping_local_fitness_max",
      "mapping_local_max_correction","mapping_local_max_rotation","loop_search_radius",
      "loop_min_time","loop_inlier_distance","loop_max_correction","loop_max_rotation",
      "loop_consistency_translation","loop_consistency_rotation","loop_min_interval","loop_fitness_max"}) {
      const double value=get_parameter(key).as_double();
      if(!std::isfinite(value)||value<=0)throw std::runtime_error(std::string(key)+" must be positive");
    }
    for(const auto *key:{"mapping_local_inlier_ratio_min","loop_inlier_ratio_min"}) {
      const double value=get_parameter(key).as_double();
      if(!std::isfinite(value)||value<=0||value>1)throw std::runtime_error(std::string(key)+" must be in (0,1]");
    }
    if(get_parameter("mapping_local_submap_frames").as_int()<1||get_parameter("loop_candidates").as_int()<1||
      get_parameter("loop_submap_frames").as_int()<0||get_parameter("loop_consistent_frames").as_int()<1||
      get_parameter("loop_separation").as_int()<1)throw std::runtime_error("Invalid mapping frame counts");
    if(!std::isfinite(get_parameter("loop_ambiguity_ratio").as_double())||
       get_parameter("loop_ambiguity_ratio").as_double()<1)throw std::runtime_error("Invalid loop ambiguity ratio");
    const auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.5),
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .04,.04,.04,.025,.025,.035).finished()));
    loop_noise_ = robust;
    mapping_noise_=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5),
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<.02,.02,.02,.03,.03,.03).finished()));
    wheel_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .03,.03,.05,.08,.08,.25).finished());
    lio_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.5),
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .30,.30,.60,1.0,1.0,1.0).finished()));
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << .02,.02,.03,.02,.02,.025).finished());
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.05;
    params.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
    localization_offset_ = gtsam::Pose3(gtsam::Rot3::RzRyRx(
      get_parameter("initial_pose_roll").as_double(), get_parameter("initial_pose_pitch").as_double(),
      get_parameter("initial_pose_yaw").as_double()), gtsam::Point3(
      get_parameter("initial_pose_x").as_double(), get_parameter("initial_pose_y").as_double(),
      get_parameter("initial_pose_z").as_double()));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS(),
      std::bind(&LoopClosurePGO::cloudCallback, this, std::placeholders::_1));
    mapping_cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      "/fast_lio/cloud_registered_body",rclcpp::QoS(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(mode_!="map"||msg->data.empty())return;
        mapping_pending_.push_back(msg);
        if(mapping_pending_.size()>20) {
          mapping_pending_.pop_front();
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Mapping scan queue overflow; discarding oldest scan");
        }
        processMappingScans();
      });
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
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != "localize") return;
        if (msg->header.frame_id != "map") {
          RCLCPP_WARN(get_logger(), "Ignoring /initialpose: pose must be in map frame"); return;
        }
        const auto &p=msg->pose.pose;
        const auto &q=p.orientation;
        const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
            !std::isfinite(p.position.z) || !std::isfinite(norm) || norm<1e-6) return;
        auto normalized=p;
        normalized.orientation.x/=norm; normalized.orientation.y/=norm;
        normalized.orientation.z/=norm; normalized.orientation.w/=norm;
        // RViz's 2D tool supplies chassis XY/yaw, with zero height and level
        // roll/pitch even when this FAST-LIO map's Z axis points downward.
        // Preserve the current height/tilt estimate and apply only XY/yaw,
        // then convert the chassis pose to the lidar registration pose.
        const auto prior=localization_started_ ? optimized_localization_ : localization_offset_;
        const auto base=prior.compose(lidarInBase().inverse());
        const auto tilt=base.rotation().rpy();
        const auto requested=fromMsg(normalized);
        localization_offset_=gtsam::Pose3(
          gtsam::Rot3::RzRyRx(tilt.x(),tilt.y(),requested.rotation().yaw()),
          gtsam::Point3(p.position.x,p.position.y,base.z())).compose(lidarInBase());
        localization_started_=false;localization_aligned_=false;
        localization_initial_matches_=0;localization_failures_=0;localization_unstable_frames_=0;
        global_requested_=false;
        ++localization_generation_;
        RCLCPP_INFO(get_logger(), "Received RViz initial base pose; restarting validated GICP initialization");
      });
    relocalize_srv_ = create_service<std_srvs::srv::Trigger>("/relocalize",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             const std_srvs::srv::Trigger::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        res->success=mode_=="localize" && get_parameter("localization_global_enabled").as_bool();
        if (!res->success) {res->message="Global relocalization requires localize mode and localization_global_enabled";return;}
        localization_aligned_=false;localization_initial_matches_=0;
        localization_failures_=get_parameter("localization_global_failure_frames").as_int();
        ++localization_generation_;
        global_requested_=true;
        last_global_search_=std::chrono::steady_clock::time_point{};
        res->message="Global search requested; waiting for a scan and subsequent validation (this is not a success pose)";
      });
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto period = std::chrono::duration<double>(1.0 / get_parameter("map_publish_rate").as_double());
    map_timer_ = create_wall_timer(period, std::bind(&LoopClosurePGO::publishMap, this));
    if (mode_ == "localize") loadMap();
    const auto map_source = mode_=="map" ? map_directory_ : (map_file_.empty() ? map_directory_ : map_file_);
    RCLCPP_INFO(get_logger(), "FAST-LIO2 pose graph ready: Scan Context + GICP loop checks + GTSAM/iSAM2; mode=%s, map=%s",
      mode_.c_str(), map_source.c_str());
    if(mode_=="map")RCLCPP_INFO(get_logger(),
      "Mapping input=/fast_lio/cloud_registered_body; timestamp-aligned wheel/LIO; local submap GICP=%s; save optimized map with /save_map",
      get_parameter("mapping_local_registration").as_bool()?"enabled":"disabled");
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
    lio_buffer_.add(rclcpp::Time(msg->header.stamp).nanoseconds(),latest_lio_);
    if(mode_=="map")processMappingScans();
    publishPose(msg->header.stamp);
  }
  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_wheel_=fromMsg(msg->pose.pose); have_wheel_pose_=true;
    wheel_buffer_.add(rclcpp::Time(msg->header.stamp).nanoseconds(),latest_wheel_);
    if(mode_=="map"){processMappingScans();publishPose(latest_stamp_);}
  }
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(mode_!="localize")return;
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
    localize(cloud,msg->header.stamp);
  }
  void processMappingScans() {
    while(!mapping_pending_.empty()) {
      const auto msg=mapping_pending_.front();
      const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
      gtsam::Pose3 wheel,lio;
      const double gap=get_parameter("mapping_sync_max_gap").as_double();
      if(!wheel_buffer_.lookup(stamp,gap,wheel)||!lio_buffer_.lookup(stamp,gap,lio)) {
        // Wait for a future bracket, but never wait forever for an old hole.
        if(wheel_buffer_.newest()<=stamp||lio_buffer_.newest()<=stamp)return;
        mapping_pending_.pop_front();
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Mapping scan rejected: no time-aligned wheel/LIO poses");
        continue;
      }
      mapping_pending_.pop_front();
      Cloud::Ptr input(new Cloud);pcl::fromROSMsg(*msg,*input);
      auto cloud=downsample(input);
      if(cloud->size()>=50)addMappingKeyframe(cloud,msg->header.stamp,wheel,lio);
    }
  }
  void addMappingKeyframe(const Cloud::Ptr &cloud,const builtin_interfaces::msg::Time &stamp,
                         const gtsam::Pose3 &wheel,const gtsam::Pose3 &lio) {
    if (last_keyframe_) {
      const rclcpp::Time current_stamp(stamp);
      const rclcpp::Time previous_stamp(last_keyframe_stamp_);
      if ((current_stamp-previous_stamp).seconds() < get_parameter("keyframe_min_interval").as_double())
        return;
    }
    const auto lidar_in_base=lidarInBase();
    const auto wheel_lidar=wheel.compose(lidar_in_base);
    if (!have_wheel_anchor_) { wheel_anchor_=wheel_lidar; have_wheel_anchor_=true; }
    if (!have_lio_anchor_) { lio_anchor_=lio; have_lio_anchor_=true; }
    const auto wheel_pose=wheel_anchor_.between(wheel_lidar);
    const auto lio_pose=lio_anchor_.between(lio);
    const double moved=last_keyframe_?keyframes_.back().wheel_pose.between(wheel_pose).translation().norm():1e9;
    const Eigen::Vector3d turn_rpy=last_keyframe_?keyframes_.back().wheel_pose.between(wheel_pose).rotation().rpy():Eigen::Vector3d(1e9,0,0);
    const double turned=turn_rpy.norm();
    if(last_keyframe_&&moved<get_parameter("keyframe_distance").as_double()&&turned<get_parameter("keyframe_yaw").as_double())return;
    Keyframe kf; kf.stamp=rclcpp::Time(stamp).nanoseconds();
    kf.wheel_pose=wheel_pose; kf.lio_pose=lio_pose; kf.cloud=cloud; kf.descriptor=scanContext(*cloud);
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
      const auto previous=optimized_.at<gtsam::Pose3>(index-1);
      const auto wheel_step=keyframes_.back().wheel_pose.between(kf.wheel_pose);
      auto initial=previous.compose(wheel_step);
      if(get_parameter("mapping_local_registration").as_bool()) {
        const size_t frames=get_parameter("mapping_local_submap_frames").as_int();
        const auto target=mappingSubmap(index-1,index>frames?index-frames:0,index);
        const auto match=robocon::matchMappingCloud(kf.cloud,target,wheel_step.matrix().cast<float>(),
          get_parameter("gicp_max_correspondence").as_double(),
          get_parameter("mapping_local_fitness_max").as_double(),get_parameter("loop_inlier_distance").as_double(),
          get_parameter("mapping_local_inlier_ratio_min").as_double(),
          get_parameter("mapping_local_max_correction").as_double(),get_parameter("mapping_local_max_rotation").as_double());
        if(match.accepted) {
          const gtsam::Pose3 relative(Eigen::Matrix4d(match.pose.cast<double>()));
          graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(index-1,index,relative,mapping_noise_));
          initial=previous.compose(relative);++mapping_match_count_;
        }
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),3000,
          "Mapping local GICP %s: mse=%.5f m^2 overlap=%.1f%% correction=%.3f m/%.3f rad",
          match.accepted?"accepted":"rejected",match.mse,match.overlap*100,match.translation,match.rotation);
      }
      initials_.insert(index,initial);
      isam_->update(graph_,initials_); graph_.resize(0); initials_.clear();
      detectLoop(index,kf);
    }
    keyframes_.push_back(std::move(kf)); last_keyframe_=true; last_keyframe_stamp_=stamp;
    optimized_=isam_->calculateEstimate();
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"FAST-LIO2 keyframes=%zu loop closures=%zu",keyframes_.size(),loop_count_);
    publishPath();
  }
  Cloud::Ptr mappingSubmap(size_t origin,size_t begin,size_t end) const {
    Cloud::Ptr combined(new Cloud);
    const auto inverse=optimized_.at<gtsam::Pose3>(origin).inverse();
    for(size_t i=begin;i<end;++i) {
      Cloud transformed;
      pcl::transformPointCloud(*keyframes_[i].cloud,transformed,
        inverse.compose(optimized_.at<gtsam::Pose3>(i)).matrix().cast<float>());
      *combined+=transformed;
    }
    return downsample(combined);
  }
  void detectLoop(size_t current,const Keyframe &kf) {
    const size_t separation=get_parameter("loop_separation").as_int();
    if(current<=separation)return;
    if(last_loop_stamp_ && (kf.stamp-last_loop_stamp_)*1e-9<get_parameter("loop_min_interval").as_double())return;
    const auto predicted=isam_->calculateEstimate<gtsam::Pose3>(current);
    struct Candidate {size_t index;double descriptor;};
    std::vector<Candidate> candidates;
    size_t nearby=0;
    for(size_t i=0;i+separation<current;++i) {
      if((kf.stamp-keyframes_[i].stamp)*1e-9<get_parameter("loop_min_time").as_double())continue;
      const auto pose=optimized_.at<gtsam::Pose3>(i);
      if((pose.translation()-predicted.translation()).norm()>get_parameter("loop_search_radius").as_double())continue;
      ++nearby;
      const double distance=contextDistance(kf.descriptor,keyframes_[i].descriptor);
      if(distance<=get_parameter("loop_similarity").as_double())candidates.push_back({i,distance});
    }
    std::sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.descriptor<b.descriptor;});
    if(candidates.size()>static_cast<size_t>(get_parameter("loop_candidates").as_int()))
      candidates.resize(get_parameter("loop_candidates").as_int());
    if(candidates.empty()) {
      pending_loop_count_=0;
      RCLCPP_INFO(get_logger(),"No verified loop proposal at keyframe %zu: nearby historical frames=%zu",current,nearby);
      return;
    }
    robocon::MappingMatch best;best.mse=std::numeric_limits<double>::infinity();
    std::vector<std::pair<double,gtsam::Pose3>> alternatives;
    size_t candidate=current;double descriptor=0;
    for(const auto &proposal:candidates) {
      const size_t i=proposal.index,neighbors=get_parameter("loop_submap_frames").as_int();
      const auto target=mappingSubmap(i,i>neighbors?i-neighbors:0,std::min(i+neighbors+1,current-separation));
      const auto guess=optimized_.at<gtsam::Pose3>(i).between(predicted).matrix().cast<float>();
      const auto match=robocon::matchMappingCloud(kf.cloud,target,guess,
        get_parameter("gicp_max_correspondence").as_double(),get_parameter("loop_fitness_max").as_double(),
        get_parameter("loop_inlier_distance").as_double(),get_parameter("loop_inlier_ratio_min").as_double(),
        get_parameter("loop_max_correction").as_double(),get_parameter("loop_max_rotation").as_double());
      if(match.accepted&&match.mse<best.mse){best=match;candidate=i;descriptor=proposal.descriptor;}
      if(match.accepted)alternatives.emplace_back(match.mse,optimized_.at<gtsam::Pose3>(i).compose(
        gtsam::Pose3(Eigen::Matrix4d(match.pose.cast<double>()))));
      if(!match.accepted)RCLCPP_INFO(get_logger(),
        "Rejected loop %zu->%zu: mse=%.5f m^2 overlap=%.1f%% correction=%.3f m/%.3f rad",
        i,current,match.mse,match.overlap*100,match.translation,match.rotation);
    }
    if(candidate==current){pending_loop_count_=0;return;}
    const gtsam::Pose3 relative(Eigen::Matrix4d(best.pose.cast<double>()));
    const auto measured=optimized_.at<gtsam::Pose3>(candidate).compose(relative);
    for(const auto &alternative:alternatives) {
      const auto difference=measured.between(alternative.second);
      if(alternative.first<=best.mse*get_parameter("loop_ambiguity_ratio").as_double()+1e-5&&
         (difference.translation().norm()>.50||gtsam::Rot3::Logmap(difference.rotation()).norm()>.20)) {
        pending_loop_count_=0;
        RCLCPP_WARN(get_logger(),"Rejected ambiguous loop at keyframe %zu: similarly good matches at different poses",current);
        return;
      }
    }
    const auto expected=pending_loop_pose_.compose(pending_loop_wheel_.between(kf.wheel_pose));
    const auto change=expected.between(measured);
    const bool agrees=pending_loop_count_>0 && current==pending_loop_index_+1 &&
      change.translation().norm()<=get_parameter("loop_consistency_translation").as_double()&&
      gtsam::Rot3::Logmap(change.rotation()).norm()<=get_parameter("loop_consistency_rotation").as_double();
    pending_loop_count_=agrees?pending_loop_count_+1:1;
    pending_loop_pose_=measured;pending_loop_wheel_=kf.wheel_pose;pending_loop_index_=current;
    if(pending_loop_count_<get_parameter("loop_consistent_frames").as_int()) {
      RCLCPP_INFO(get_logger(),"Loop %zu->%zu pending: %d/%ld consistent keyframes, mse=%.5f m^2 overlap=%.1f%%",
        candidate,current,pending_loop_count_,get_parameter("loop_consistent_frames").as_int(),best.mse,best.overlap*100);
      return;
    }
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(candidate,current,relative,loop_noise_));
    isam_->update(graph_);graph_.resize(0);
    // Large loop corrections need relinearization of existing factors too.
    for(int i=0;i<3;++i)isam_->update();
    ++loop_count_;last_loop_stamp_=kf.stamp;pending_loop_count_=0;
    const auto corrected=isam_->calculateEstimate<gtsam::Pose3>(current);
    const auto delta=predicted.between(corrected);
    RCLCPP_WARN(get_logger(),
      "Accepted verified loop %zu->%zu: descriptor=%.3f mse=%.5f m^2 overlap=%.1f%%; graph correction=%.3f m/%.3f rad; rebuilding all keyframes",
      candidate,current,descriptor,best.mse,best.overlap*100,delta.translation().norm(),gtsam::Rot3::Logmap(delta.rotation()).norm());
  }
  Eigen::Matrix4f poseMatrix(size_t i) const { return optimized_.at<gtsam::Pose3>(i).matrix().cast<float>(); }
  void pollGlobalSearch() {
    if (!global_search_.valid() || global_search_.wait_for(0s)!=std::future_status::ready) return;
    robocon::GlobalResult result;
    try {result=global_search_.get();}
    catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"Global relocalization failed: %s",e.what());return;
    }
    // Discard results superseded by a manual pose or successful local alignment.
    if (global_generation_!=localization_generation_ || localization_aligned_) return;
    RCLCPP_INFO(get_logger(),
      "Global backend=%s found=%s timeout=%s map_build_ms=%.2f search_ms=%.2f refine_ms=%.2f total_ms=%.2f coarse_score=%.3f",
      result.backend.c_str(),result.found?"true":"false",result.timed_out?"true":"false",
      result.map_build_ms,result.search_ms,result.refine_ms,result.total_ms,result.coarse_score);
    if (!result.found) {
      RCLCPP_WARN(get_logger(),"Global relocalization not accepted: %s",result.detail.c_str());return;
    }
    const auto mount=lidarInBase();
    const auto motion=mount.inverse().compose(global_wheel_start_.between(latest_wheel_)).compose(mount);
    optimized_localization_=gtsam::Pose3(Eigen::Matrix4d(result.pose.cast<double>())).compose(motion);
    localization_wheel_previous_=latest_wheel_;
    localization_started_=true;localization_initial_matches_=0;localization_failures_=0;
    localization_unstable_frames_=0;
    RCLCPP_INFO(get_logger(),
      "Global relocalization candidate: xyz=[%.3f %.3f %.3f] yaw=%.3f rad fitness=%.5f m^2 "
      "overlap=%.1f%%; %s; validating on fresh scans",
      optimized_localization_.x(),optimized_localization_.y(),optimized_localization_.z(),
      optimized_localization_.rotation().yaw(),result.fitness,result.overlap*100.0,result.detail.c_str());
  }
  void startGlobalSearch(const Cloud::Ptr &cloud,const gtsam::Pose3 &predicted) {
    if (global_search_.valid() || !get_parameter("localization_global_enabled").as_bool()) return;
    const auto steady=std::chrono::steady_clock::now();
    if (last_global_search_!=std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(steady-last_global_search_).count()<
          get_parameter("localization_global_retry_seconds").as_double()) return;
    last_global_search_=steady;
    robocon::GlobalOptions options;
    options.backend=get_parameter("localization_global_backend").as_string();
    options.voxel=get_parameter("localization_global_voxel").as_double();
    options.xy_step=get_parameter("localization_global_xy_step").as_double();
    options.z_step=get_parameter("localization_global_z_step").as_double();
    options.yaw_step=get_parameter("localization_global_yaw_step").as_double();
    options.refine_candidates=get_parameter("localization_global_refine_candidates").as_int();
    options.fitness_max=get_parameter("localization_initial_fitness_max").as_double();
    options.inlier_distance=get_parameter("localization_initial_inlier_distance").as_double();
    options.overlap_min=get_parameter("localization_initial_inlier_ratio_min").as_double();
    options.ambiguity_ratio=get_parameter("localization_global_ambiguity_ratio").as_double();
    options.bbs_resolution=get_parameter("localization_bbs_resolution").as_double();
    options.bbs_levels=get_parameter("localization_bbs_levels").as_int();
    options.bbs_scan_voxel=get_parameter("localization_bbs_scan_voxel").as_double();
    options.bbs_roll_pitch_window=get_parameter("localization_bbs_roll_pitch_window").as_double();
    options.bbs_score_threshold=get_parameter("localization_bbs_score_threshold").as_double();
    options.bbs_timeout_ms=get_parameter("localization_bbs_timeout_ms").as_int();
    options.bbs_cpu_threads=get_parameter("localization_bbs_cpu_threads").as_int();
    options.bbs_gpu_batch_size=get_parameter("localization_bbs_gpu_batch_size").as_int();
    try {
      if(!global_backend_ || global_backend_->name()!=options.backend)
        global_backend_=robocon::makeGlobalSearchBackend(map_cloud_,options);
    } catch(const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"Cannot start global backend %s: %s",options.backend.c_str(),e.what());
      return;
    }
    global_generation_=localization_generation_;global_wheel_start_=latest_wheel_;
    global_requested_=false;
    const Cloud::ConstPtr target=map_cloud_,source=cloud;
    const Eigen::Matrix4f guess=predicted.matrix().cast<float>();
    RCLCPP_WARN(get_logger(),"Starting global backend=%s after %d rejected scans; local TF/map publishing continues",
      options.backend.c_str(),localization_failures_);
    const auto prepared=global_backend_;
    global_search_=std::async(std::launch::async,[source,target,guess,options,prepared] {
      return robocon::globalRelocalize(source,target,guess,options,prepared);
    });
  }
  void localizationFailure(const Cloud::Ptr &cloud,const gtsam::Pose3 &predicted,
                           const builtin_interfaces::msg::Time &stamp) {
    optimized_localization_=predicted;localization_initial_matches_=0;++localization_failures_;
    const int failures=get_parameter("localization_global_failure_frames").as_int();
    if (localization_failures_>=failures) {
      if (localization_aligned_) {
        localization_aligned_=false;++localization_generation_;
        RCLCPP_WARN(get_logger(),"GICP tracking lost; entering relocalization");
      }
      startGlobalSearch(cloud,predicted);
    }
    publishLocalizationPose(stamp,1.0);
  }
  void localize(const Cloud::Ptr &cloud,const builtin_interfaces::msg::Time &stamp) {
    if(map_cloud_->empty())return;
    pollGlobalSearch();
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
    if (global_requested_) startGlobalSearch(cloud,predicted);
    const bool initializing = !localization_aligned_;
    const char *stage = initializing ? "initializing" : "tracking";

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> gicp;
    gicp.setMaximumIterations(35); gicp.setMaxCorrespondenceDistance(.65);
    gicp.setInputSource(cloud); gicp.setInputTarget(map_cloud_);
    Eigen::Matrix4f guess=predicted.matrix().cast<float>();
    Cloud aligned; gicp.align(aligned,guess);
    // PCL averages squared nearest-neighbor distances and expects a squared
    // cutoff here, unlike setMaxCorrespondenceDistance (which takes metres).
    const double fitness=gicp.getFitnessScore(.65 * .65);
    const double fitness_limit=get_parameter(initializing ?
      "localization_initial_fitness_max" : "localization_fitness_max").as_double();
    if(!gicp.hasConverged() || !gicp.getFinalTransformation().allFinite() ||
       !std::isfinite(fitness) || fitness>fitness_limit) {
      localization_initial_matches_ = 0;
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "GICP %s rejected: converged=%s fitness=%.5f m^2 limit=%.5f m^2 source=%zu map=%zu; using wheel prediction",
        stage,gicp.hasConverged()?"true":"false",fitness,fitness_limit,cloud->size(),map_cloud_->size());
      localizationFailure(cloud,predicted,stamp);
      return;
    }
    const gtsam::Pose3 icp_pose(Eigen::Matrix4d(gicp.getFinalTransformation().cast<double>()));
    const gtsam::Vector6 correction=predicted.localCoordinates(icp_pose);
    const double rotation_error=correction.head<3>().norm();
    const double translation_error=correction.tail<3>().norm();
    const double translation_limit = get_parameter(initializing ?
      "localization_initial_max_correction" : "localization_max_correction").as_double();
    const double rotation_limit = get_parameter(initializing ?
      "localization_initial_max_rotation_correction" : "localization_max_rotation_correction").as_double();
    // Direct translation differences are in MAP axes; the SE(3) logarithm
    // above is in the predicted sensor frame and is not an XYZ map offset.
    const Eigen::Vector3d map_delta = icp_pose.translation() - predicted.translation();
    const Eigen::Vector3d rotation_delta = predicted.between(icp_pose).rotation().rpy() * (180.0 / M_PI);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
      "GICP %s candidate: fitness=%.5f m^2 map_delta_xyz=[%.3f %.3f %.3f] m "
      "relative_delta_rpy=[%.2f %.2f %.2f] deg; translation=%.3f/%.3f m rotation=%.3f/%.3f rad",
      stage,fitness,map_delta.x(),map_delta.y(),map_delta.z(),
      rotation_delta.x(),rotation_delta.y(),rotation_delta.z(),
      translation_error,translation_limit,rotation_error,rotation_limit);
    if(translation_error>translation_limit || rotation_error>rotation_limit) {
      localization_initial_matches_ = 0;
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "GICP %s correction rejected: exceeds correction limits; using wheel prediction", stage);
      localizationFailure(cloud,predicted,stamp);
      return;
    }
    {
      // Low fitness alone can come from a small patch of floor. Require a
      // substantial fraction of the WHOLE scan to agree with the target.
      pcl::search::KdTree<pcl::PointXYZI> tree;
      tree.setInputCloud(map_cloud_);
      size_t inliers = 0;
      std::vector<int> ids(1);
      std::vector<float> distances(1);
      const double distance = get_parameter("localization_initial_inlier_distance").as_double();
      for (const auto &point : aligned.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
            tree.nearestKSearch(point, 1, ids, distances) > 0 && distances[0] <= distance * distance)
          ++inliers;
      }
      const double overlap = aligned.empty() ? 0.0 : double(inliers) / aligned.size();
      if (overlap < get_parameter("localization_initial_inlier_ratio_min").as_double()) {
        localization_initial_matches_ = 0;
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
          "GICP %s rejected: overlap=%.1f%% below %.1f%% within %.2f m; using wheel prediction",
          stage, overlap * 100.0, get_parameter("localization_initial_inlier_ratio_min").as_double() * 100.0,
          distance);
        localizationFailure(cloud,predicted,stamp);
        return;
      }
      // Check overlap during tracking too: a tiny floor patch can otherwise
      // produce a low fitness while localization has actually been lost.
      localization_failures_=0;
      if (!initializing) {
        const double weight=std::clamp(get_parameter("localization_icp_weight").as_double(),0.0,1.0);
        optimized_localization_=predicted.retract(weight*correction);
        publishLocalizationPose(stamp,fitness);
        return;
      }
      bool consistent = false;
      if (localization_initial_matches_ > 0) {
        // Compare candidates after compensating robot motion between scans.
        const auto previous = localization_initial_candidate_.compose(lidar_motion);
        const auto change = previous.between(icp_pose);
        consistent = change.translation().norm() <=
          get_parameter("localization_initial_consistency_translation").as_double() &&
          gtsam::Rot3::Logmap(change.rotation()).norm() <=
          get_parameter("localization_initial_consistency_rotation").as_double();
        if (!consistent) ++localization_unstable_frames_;
      }
      localization_initial_matches_ = consistent ? localization_initial_matches_ + 1 : 1;
      localization_initial_candidate_ = icp_pose;
      const int required = get_parameter("localization_initial_consistent_frames").as_int();
      if (localization_initial_matches_ < required) {
        if (localization_unstable_frames_>=get_parameter("localization_global_failure_frames").as_int())
          startGlobalSearch(cloud,predicted);
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
          "GICP initialization pending: %d/%d consistent scans, overlap=%.1f%%",
          localization_initial_matches_,required,overlap*100.0);
        optimized_localization_ = predicted;
        publishLocalizationPose(stamp, 1.0);
        return;
      }
      // Apply the full validated initial alignment ONCE. Blending only 10%%
      // here would leave an initial offset above the tracking correction gate.
      optimized_localization_ = icp_pose;
      localization_aligned_ = true;
      localization_unstable_frames_=0;
      ++localization_generation_;
      const auto rpy = icp_pose.rotation().rpy();
      RCLCPP_INFO(get_logger(),
        "GICP initialized: map_pose_xyz=[%.4f %.4f %.4f] m rpy=[%.6f %.6f %.6f] rad "
        "fitness=%.5f m^2 overlap=%.1f%%; tracking limits now %.2f m / %.2f rad",
        icp_pose.x(),icp_pose.y(),icp_pose.z(),rpy.x(),rpy.y(),rpy.z(),fitness,overlap*100.0,
        get_parameter("localization_max_correction").as_double(),
        get_parameter("localization_max_rotation_correction").as_double());
      publishLocalizationPose(stamp, fitness);
      return;
    }
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
    auto wheel=latest_wheel_;
    if(mode_=="map"&&!wheel_buffer_.lookup(rclcpp::Time(stamp).nanoseconds(),
       get_parameter("mapping_sync_max_gap").as_double(),wheel))return;
    const auto odom_lidar = wheel.compose(lidarInBase());
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
    gtsam::Pose3 wheel;
    if(!wheel_buffer_.lookup(rclcpp::Time(stamp).nanoseconds(),
       get_parameter("mapping_sync_max_gap").as_double(),wheel))return;
    const auto wheel_lidar=wheel.compose(lidarInBase());
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
    publishPose(latest_stamp_);
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
    std::ofstream info(dir/"map_info.txt");info<<"frontend=FAST-LIO2 (IKFoM + ikd-Tree)\nmapping_cloud=/fast_lio/cloud_registered_body\nmapping_pose_sync=timestamp interpolation\nloop_detection=Scan Context\nloop_verification=historical submap GICP, all-point MSE, tight overlap, bounded innovation, consecutive keyframes\npose_graph=GTSAM iSAM2, wheel-odometry factors plus robust FAST-LIO2 relative factors\nlidar_mount_translation_base_footprint=-0.10,0,0.46\nlidar_mount_rpy_base_footprint=3.1415926536,0,0\nkeyframes="<<keyframes_.size()<<"\nlocal_registration_factors="<<mapping_match_count_<<"\nloop_closures="<<loop_count_<<"\npoints="<<filtered.size()<<"\nframe=map, anchored at the initial lidar pose\n";
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
  bool localization_aligned_ = false;
  int localization_initial_matches_ = 0;
  gtsam::Pose3 localization_initial_candidate_;
  int localization_failures_=0;
  int localization_unstable_frames_=0;
  bool global_requested_=false;
  size_t localization_generation_=0,global_generation_=0;
  gtsam::Pose3 global_wheel_start_;
  std::chrono::steady_clock::time_point last_global_search_;
  std::future<robocon::GlobalResult> global_search_;
  std::shared_ptr<robocon::GlobalSearchBackend> global_backend_;
  std::vector<Keyframe> keyframes_;Cloud::Ptr map_cloud_{new Cloud};
  StampedPoseBuffer wheel_buffer_,lio_buffer_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> mapping_pending_;
  size_t mapping_match_count_=0,pending_loop_index_=0;
  int pending_loop_count_=0;
  int64_t last_loop_stamp_=0;
  gtsam::Pose3 pending_loop_pose_,pending_loop_wheel_;
  gtsam::NonlinearFactorGraph graph_;gtsam::Values initials_,optimized_;
  std::unique_ptr<gtsam::ISAM2> isam_;gtsam::SharedNoiseModel loop_noise_,wheel_noise_,lio_noise_,prior_noise_,mapping_noise_;
  builtin_interfaces::msg::Time last_keyframe_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapping_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_,wheel_odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LoopClosurePGO>());rclcpp::shutdown();return 0;}
