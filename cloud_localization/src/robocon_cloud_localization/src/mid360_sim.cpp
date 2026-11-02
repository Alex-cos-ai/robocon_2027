// MID360-style ray sensor for Gazebo Classic. It samples the live collision
// geometry over each scan and is deliberately a virtual TF frame, not a body.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace gazebo {
class Mid360Sim final : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    node_ = gazebo_ros::Node::Get();
    auto read = [&](const std::string &key, double fallback) {
      return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;
    };
    const auto parent = sdf->HasElement("parent_link") ? sdf->Get<std::string>("parent_link") : "base_link";
    ground_truth_frame_ = parent;
    link_ = model_->GetLink(parent);
    if (!link_) {
      RCLCPP_FATAL(node_->get_logger(), "MID360 parent link '%s' missing", parent.c_str());
      return;
    }
    ground_truth_only_ = sdf->HasElement("ground_truth_only") && sdf->Get<bool>("ground_truth_only");
    truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/simulation/ground_truth", 10);
    if (ground_truth_only_) {
      rate_ = read("truth_rate", 10.0);
      if (rate_ <= 0.0) {
        RCLCPP_FATAL(node_->get_logger(), "Ground-truth publish rate must be positive");
        return;
      }
      last_ = model_->GetWorld()->SimTime();
      last_truth_ = last_;
      update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&Mid360Sim::Update, this));
      RCLCPP_INFO(node_->get_logger(), "Ground-truth odometry enabled at %.1f Hz", rate_);
      return;
    }
    offset_ = ignition::math::Pose3d(read("x", 0), read("y", 0), read("z", .50),
      read("roll", M_PI), read("pitch", 0), read("yaw", 0));
    rate_ = read("update_rate", 10);
    count_ = static_cast<int>(read("samples", 2000));
    min_range_ = read("min_range", .10);
    max_range_ = read("max_range", 40);
    noise_ = read("noise_stddev", .008);
    field_x_min_ = read("field_x_min", -5.5);
    field_x_max_ = read("field_x_max", 5.5);
    field_y_min_ = read("field_y_min", -5.5);
    field_y_max_ = read("field_y_max", 5.5);
    normal_ = std::normal_distribution<double>(0.0, noise_);
    if (rate_ <= 0 || count_ < 100 || max_range_ <= min_range_) {
      RCLCPP_FATAL(node_->get_logger(), "Invalid MID360 rate, samples, or ranges");
      return;
    }
    cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", rclcpp::SensorDataQoS());
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", rclcpp::SensorDataQoS());
    ray_collision_ = model_->GetWorld()->Physics()->CreateCollision("ray", link_);
    ray_ = boost::dynamic_pointer_cast<physics::RayShape>(model_->GetWorld()->Physics()->CreateShape("ray", ray_collision_));
    if (!ray_) {
      RCLCPP_FATAL(node_->get_logger(), "Gazebo could not create a ray shape");
      return;
    }
    last_ = model_->GetWorld()->SimTime();
    scan_start_ = last_;
    last_imu_ = last_;
    previous_velocity_ = link_->WorldLinearVel();
    scan_data_.reserve(static_cast<size_t>(count_)*24);
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&Mid360Sim::Update, this));
    RCLCPP_INFO(node_->get_logger(),
      "Virtual MID360: base_link offset=(%.2f, %.2f, %.2f)m rpy=(%.1f, %.1f, %.1f)deg; %d rays at %.1fHz",
      offset_.Pos().X(), offset_.Pos().Y(), offset_.Pos().Z(),
      offset_.Rot().Roll()*180/M_PI, offset_.Rot().Pitch()*180/M_PI,
      offset_.Rot().Yaw()*180/M_PI, count_, rate_);
  }

 private:
  static builtin_interfaces::msg::Time stamp(const common::Time &t) {
    builtin_interfaces::msg::Time s; s.sec=t.sec; s.nanosec=t.nsec; return s;
  }
  void publishImu(const common::Time &t, const ignition::math::Pose3d &sensor_pose) {
    const double dt=(t-last_imu_).Double();
    if (dt < .005 || dt <= 0) return;
    const auto velocity=link_->WorldLinearVel();
    // Gravity compensation yields the accelerometer's specific force.
    const auto acceleration=sensor_pose.Rot().RotateVectorReverse(
      (velocity-previous_velocity_)/dt-model_->GetWorld()->Gravity());
    const auto gyro=sensor_pose.Rot().RotateVectorReverse(link_->WorldAngularVel());
    sensor_msgs::msg::Imu imu;
    imu.header.stamp=stamp(t); imu.header.frame_id="mid360_link";
    imu.orientation_covariance[0]=-1;  // Orientation is not supplied as ground truth.
    imu.angular_velocity.x=gyro.X(); imu.angular_velocity.y=gyro.Y(); imu.angular_velocity.z=gyro.Z();
    imu.linear_acceleration.x=acceleration.X(); imu.linear_acceleration.y=acceleration.Y(); imu.linear_acceleration.z=acceleration.Z();
    for(int i : {0,4,8}) { imu.angular_velocity_covariance[i]=1e-6; imu.linear_acceleration_covariance[i]=1e-4; }
    imu_pub_->publish(imu);
    previous_velocity_=velocity; last_imu_=t;
  }
  void addRay(const ignition::math::Pose3d &body_pose, const ignition::math::Pose3d &sensor_pose, int i) {
    // Livox-like 360 degree sweep with a non-repeating elevation pattern.
    const double sweep=static_cast<double>(i)/count_;
    const double azimuth=2*M_PI*sweep+0.0015*std::sin(i*0.6180339887498949);
    const double elevation=(-7+59*std::fmod(i*0.4142135623730951,1.0))*M_PI/180;
    const ignition::math::Vector3d dir(std::cos(elevation)*std::cos(azimuth),
      std::cos(elevation)*std::sin(azimuth),std::sin(elevation));
    const auto world_dir=sensor_pose.Rot().RotateVector(dir);
    const auto start=sensor_pose.Pos()+world_dir*min_range_;
    const auto finish=sensor_pose.Pos()+world_dir*max_range_;
    ray_->SetPoints(body_pose.Rot().RotateVectorReverse(start-body_pose.Pos()),
                    body_pose.Rot().RotateVectorReverse(finish-body_pose.Pos()));
    double distance=0; std::string entity;
    ray_->Update();
    ray_->GetIntersection(distance,entity);
    if(entity.empty()||entity.rfind(model_->GetName()+"::",0)==0) return;
    const double true_range=distance+min_range_;
    const auto hit_world=start+world_dir*distance;
    if(hit_world.X()<field_x_min_||hit_world.X()>field_x_max_||
       hit_world.Y()<field_y_min_||hit_world.Y()>field_y_max_) return;
    const double range=true_range+normal_(rng_);
    if(range<min_range_||range>max_range_) return;
    const float values[4]={static_cast<float>(dir.X()*range),static_cast<float>(dir.Y()*range),
      static_cast<float>(dir.Z()*range),static_cast<float>(100.0/(1+.02*range*range))};
    const size_t n=scan_data_.size(); scan_data_.resize(n+24,0);
    std::memcpy(scan_data_.data()+n,values,16);
    const uint32_t offset=static_cast<uint32_t>((static_cast<uint64_t>(i)*100000000ULL)/count_);
    std::memcpy(scan_data_.data()+n+16,&offset,sizeof(offset));
    scan_data_[n+20]=static_cast<uint8_t>(i%4);
    scan_data_[n+21]=0x10;
  }
  sensor_msgs::msg::PointCloud2 makeCloud(const common::Time &t) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp=stamp(scan_start_); msg.header.frame_id="mid360_link";
    msg.height=1; msg.is_dense=true; msg.is_bigendian=false;
    const char *names[]={"x","y","z","reflectivity","offset_time","line","tag"};
    const uint32_t offsets[]={0,4,8,12,16,20,21};
    for(int k=0;k<7;++k) {
      sensor_msgs::msg::PointField f; f.name=names[k]; f.offset=offsets[k]; f.count=1;
      f.datatype=k<4?sensor_msgs::msg::PointField::FLOAT32:
        (k==4?sensor_msgs::msg::PointField::UINT32:sensor_msgs::msg::PointField::UINT8);
      msg.fields.push_back(f);
    }
    msg.point_step=24; msg.data=std::move(scan_data_);
    msg.width=msg.data.size()/24; msg.row_step=msg.width*24;
    (void)t;
    return msg;
  }
  void publishGroundTruth(const common::Time &t) {
    nav_msgs::msg::Odometry gt; gt.header.stamp=stamp(t); gt.header.frame_id="world"; gt.child_frame_id=ground_truth_frame_;
    const auto p=link_->WorldPose();
    gt.pose.pose.position.x=p.Pos().X(); gt.pose.pose.position.y=p.Pos().Y(); gt.pose.pose.position.z=p.Pos().Z();
    gt.pose.pose.orientation.x=p.Rot().X(); gt.pose.pose.orientation.y=p.Rot().Y();
    gt.pose.pose.orientation.z=p.Rot().Z(); gt.pose.pose.orientation.w=p.Rot().W();
    truth_pub_->publish(gt);
  }
  void Update() {
    const auto t=model_->GetWorld()->SimTime();
    if (ground_truth_only_) {
      if (t < last_truth_) last_truth_ = t;
      if ((t - last_truth_).Double() < 1.0 / rate_) return;
      publishGroundTruth(t);
      last_truth_ = t;
      return;
    }
    if(t<last_){last_=t;scan_start_=t;last_imu_=t;scan_sample_=0;scan_data_.clear();}
    const auto body_pose=link_->WorldPose();
    const auto sensor_pose=body_pose*offset_;
    publishImu(t,sensor_pose);
    const double elapsed=(t-scan_start_).Double();
    const int desired=std::min(count_,static_cast<int>(elapsed*rate_*count_));
    while(scan_sample_<desired) addRay(body_pose,sensor_pose,scan_sample_++);
    if(elapsed<1/rate_) return;
    auto scan=makeCloud(t);
    cloud_pub_->publish(scan);
    publishGroundTruth(t);  // Evaluation only; the estimator never subscribes.
    scan_data_.clear(); scan_sample_=0; scan_start_=t; last_=t;
  }
  physics::ModelPtr model_; physics::LinkPtr link_; physics::CollisionPtr ray_collision_; physics::RayShapePtr ray_;
  gazebo_ros::Node::SharedPtr node_; event::ConnectionPtr update_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  ignition::math::Pose3d offset_; ignition::math::Vector3d previous_velocity_;
  common::Time last_,last_imu_,last_truth_,scan_start_; int scan_sample_=0; std::vector<uint8_t> scan_data_;
  std::mt19937 rng_{360}; std::normal_distribution<double> normal_{0,.008};
  double rate_=10,min_range_=.1,max_range_=40,noise_=.008;
  double field_x_min_=-5.5,field_x_max_=5.5,field_y_min_=-5.5,field_y_max_=5.5;
  int count_=2000;
  bool ground_truth_only_=false;
  std::string ground_truth_frame_="base_link";
};
GZ_REGISTER_MODEL_PLUGIN(Mid360Sim)
}  // namespace gazebo
