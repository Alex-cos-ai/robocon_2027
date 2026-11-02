#include "mapping_registration.hpp"
#include "stamped_pose_buffer.hpp"
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

static void require(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
int main() {
  try {
    StampedPoseBuffer buffer;
    const auto end=gtsam::Pose3(gtsam::Rot3::Rz(.4),gtsam::Point3(1,2,3));
    buffer.add(1000000000,gtsam::Pose3());buffer.add(1100000000,end);
    gtsam::Pose3 middle;
    require(buffer.lookup(1050000000,.2,middle),"Missing bracketed interpolation");
    require((middle.translation()-gtsam::Point3(.5,1,1.5)).norm()<1e-8,"Wrong timestamp translation");
    require(std::abs(middle.rotation().yaw()-.2)<1e-8,"Wrong timestamp rotation");
    require(!buffer.lookup(900000000,.2,middle)&&!buffer.lookup(1200000000,.2,middle),"Extrapolated old/new pose");
    require(!buffer.lookup(1050000000,.05,middle),"Interpolated across excessive measurement gap");
    buffer.add(500000000,gtsam::Pose3());
    require(!buffer.lookup(1000000000,.2,middle),"Clock rollback retained stale measurements");
    robocon::RegistrationCloud::Ptr source(new robocon::RegistrationCloud),target(new robocon::RegistrationCloud);
    std::mt19937 rng(42);std::uniform_real_distribution<float> value(-2,2);
    for(int i=0;i<1500;++i){pcl::PointXYZI p;p.x=value(rng);p.y=value(rng);p.z=value(rng);p.intensity=0;source->push_back(p);}
    Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();pose.block<3,3>(0,0)=Eigen::AngleAxisf(.1,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    pose(0,3)=.15;pcl::transformPointCloud(*source,*target,pose);
    const auto match=robocon::matchMappingCloud(source,target,Eigen::Matrix4f::Identity(),.8,.01,.2,.7,.3,.15);
    require(match.accepted&&(match.pose-pose).norm()<.02,"Failed valid mapping registration");
    require(!robocon::matchMappingCloud(source,target,Eigen::Matrix4f::Identity(),.8,.01,.2,.7,.05,.15).accepted,
      "Ignored correction gate");
    robocon::RegistrationCloud::Ptr partial(new robocon::RegistrationCloud(*source));
    for(int i=0;i<4000;++i){auto p=source->points[i%source->size()];p.x+=20;partial->push_back(p);}
    require(!robocon::matchMappingCloud(partial,target,Eigen::Matrix4f::Identity(),.8,.01,.2,.7,.3,.15).accepted,
      "Accepted low overlap using only nearest matching subset");
    robocon::RegistrationCloud::Ptr unrelated(new robocon::RegistrationCloud);
    for(int i=0;i<1500;++i){pcl::PointXYZI p;p.x=value(rng);p.y=value(rng);p.z=value(rng);p.intensity=0;unrelated->push_back(p);}
    const auto false_loop=robocon::matchMappingCloud(source,unrelated,Eigen::Matrix4f::Identity(),.8,.01,.8,.7,1.,.35);
    require(!false_loop.accepted&&false_loop.mse>.01&&false_loop.overlap>.9,
      "Broad overlap alone accepted unrelated geometry");
    std::cout<<"PASS timestamp brackets/rollback, registration, correction and full-scan quality gates\n";
    return 0;
  }catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
