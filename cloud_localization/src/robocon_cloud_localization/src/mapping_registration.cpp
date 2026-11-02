#include "mapping_registration.hpp"
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace robocon {
MappingMatch matchMappingCloud(const RegistrationCloud::ConstPtr &source,
  const RegistrationCloud::ConstPtr &target,const Eigen::Matrix4f &guess,
  double correspondence,double mse_max,double inlier_distance,
  double overlap_min,double translation_max,double rotation_max) {
  MappingMatch result;result.mse=std::numeric_limits<double>::infinity();
  if(!source||!target||source->size()<50||target->size()<50||!guess.allFinite())return result;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> gicp;
  gicp.setMaximumIterations(45);gicp.setMaxCorrespondenceDistance(correspondence);
  gicp.setTransformationEpsilon(1e-5);gicp.setEuclideanFitnessEpsilon(1e-5);
  gicp.setInputSource(source);gicp.setInputTarget(target);
  RegistrationCloud aligned;gicp.align(aligned,guess);
  if(!gicp.hasConverged()||aligned.empty()||!gicp.getFinalTransformation().allFinite())return result;
  result.pose=gicp.getFinalTransformation();
  // Compare translation in target axes; the SE(3) relative translation would
  // couple a small rotation to the distance from a distant submap origin.
  result.translation=(result.pose.block<3,1>(0,3)-guess.block<3,1>(0,3)).norm();
  result.rotation=Eigen::AngleAxisf(result.pose.block<3,3>(0,0)*guess.block<3,3>(0,0).transpose()).angle();
  pcl::search::KdTree<pcl::PointXYZI> tree;tree.setInputCloud(target);
  double squared=0;size_t inliers=0;std::vector<int> ids(1);std::vector<float> distances(1);
  const double cap=correspondence*correspondence,cutoff=inlier_distance*inlier_distance;
  for(const auto &point:aligned) {
    if(!pcl::isFinite(point)||tree.nearestKSearch(point,1,ids,distances)<=0){squared+=cap;continue;}
    squared+=std::min<double>(distances[0],cap);
    if(distances[0]<=cutoff)++inliers;
  }
  result.mse=squared/aligned.size();result.overlap=double(inliers)/aligned.size();
  result.accepted=std::isfinite(result.mse)&&result.mse<=mse_max&&result.overlap>=overlap_min&&
    result.translation<=translation_max&&result.rotation<=rotation_max;
  return result;
}
}
