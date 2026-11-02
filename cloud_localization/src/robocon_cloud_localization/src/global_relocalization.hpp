#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>
#include <memory>
#include <vector>

namespace robocon {
using RegistrationCloud = pcl::PointCloud<pcl::PointXYZI>;
struct GlobalOptions {
  std::string backend = "grid_gicp";
  double voxel = 0.25;
  double xy_step = 0.50;
  double z_step = 0.50;
  double yaw_step = 0.2617993878;
  int refine_candidates = 32;
  double fitness_max = 0.01;
  double inlier_distance = 0.20;
  double overlap_min = 0.60;
  double ambiguity_ratio = 1.25;
  double bbs_resolution = 0.20;
  int bbs_levels = 5;
  double bbs_scan_voxel = 0.25;
  double bbs_roll_pitch_window = 0.035;
  double bbs_score_threshold = 0.60;
  int bbs_timeout_ms = 3000;
  int bbs_cpu_threads = 2;
  int bbs_gpu_batch_size = 10000;
};
struct GlobalResult {
  bool found = false;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double fitness = 0.0;
  double overlap = 0.0;
  std::string detail;
  std::string backend;
  bool timed_out = false;
  double map_build_ms = 0.0;
  double search_ms = 0.0;
  double refine_ms = 0.0;
  double total_ms = 0.0;
  double coarse_score = 0.0;  // BBS occupied voxel fraction, not GICP fitness
};
struct GlobalSeeds {
  std::vector<Eigen::Matrix4f> poses;
  std::string detail;
  bool timed_out = false;
  double map_build_ms = 0.0;
  double coarse_score = 0.0;
};
// Interchangeable coarse search interface. Map data are immutable. Instances
// cache map preparation and are called by one background worker at a time.
class GlobalSearchBackend {
 public:
  virtual ~GlobalSearchBackend() = default;
  virtual const char *name() const = 0;
  virtual GlobalSeeds search(const RegistrationCloud::ConstPtr &scan,
                            const Eigen::Matrix4f &guess) = 0;
};
std::shared_ptr<GlobalSearchBackend> makeGlobalSearchBackend(
  const RegistrationCloud::ConstPtr &map, const GlobalOptions &options);
GlobalResult refineGlobalCandidates(const RegistrationCloud::ConstPtr &scan,
                                   const RegistrationCloud::ConstPtr &map,
                                   const std::vector<Eigen::Matrix4f> &seeds,
                                   const GlobalOptions &options);
// Search XYZ and yaw across the target bounds. Roll/pitch remain near the
// supplied gravity/mount estimate; GICP refines all six degrees of freedom.
GlobalResult globalRelocalize(const RegistrationCloud::ConstPtr &scan,
                             const RegistrationCloud::ConstPtr &map,
                             const Eigen::Matrix4f &guess,
                             const GlobalOptions &options,
                             const std::shared_ptr<GlobalSearchBackend> &backend = {});
}  // namespace robocon
