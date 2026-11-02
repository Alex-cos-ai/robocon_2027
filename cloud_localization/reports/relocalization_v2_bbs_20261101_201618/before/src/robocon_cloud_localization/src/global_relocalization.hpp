#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>

namespace robocon {
using RegistrationCloud = pcl::PointCloud<pcl::PointXYZI>;
struct GlobalOptions {
  double voxel = 0.25;
  double xy_step = 0.50;
  double z_step = 0.50;
  double yaw_step = 0.2617993878;
  int refine_candidates = 32;
  double fitness_max = 0.01;
  double inlier_distance = 0.20;
  double overlap_min = 0.60;
  double ambiguity_ratio = 1.25;
};
struct GlobalResult {
  bool found = false;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double fitness = 0.0;
  double overlap = 0.0;
  std::string detail;
};
// Search XYZ and yaw across the target bounds. Roll/pitch remain near the
// supplied gravity/mount estimate; GICP refines all six degrees of freedom.
GlobalResult globalRelocalize(const RegistrationCloud::ConstPtr &scan,
                             const RegistrationCloud::ConstPtr &map,
                             const Eigen::Matrix4f &guess,
                             const GlobalOptions &options);
}  // namespace robocon
