#pragma once
#include "global_relocalization.hpp"

namespace robocon {
struct MappingMatch {
  bool accepted = false;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double mse = 0, overlap = 0, translation = 0, rotation = 0;
};
// All-point capped MSE and tight inlier distance are deliberately separate
// from GICP's larger correspondence radius used to find correspondences.
MappingMatch matchMappingCloud(const RegistrationCloud::ConstPtr &source,
  const RegistrationCloud::ConstPtr &target, const Eigen::Matrix4f &guess,
  double correspondence, double mse_max, double inlier_distance,
  double overlap_min, double translation_max, double rotation_max);
}
