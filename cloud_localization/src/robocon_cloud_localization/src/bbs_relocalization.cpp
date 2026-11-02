#include "global_relocalization.hpp"
#include <cpu_bbs3d/bbs3d.hpp>
#ifdef ROBOCON_HAS_BBS_CUDA
#include <gpu_bbs3d/bbs3d.cuh>
#include <cuda_runtime_api.h>
#endif
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <chrono>
#include <stdexcept>
#include <type_traits>

namespace robocon {
namespace {
template<typename Scalar,typename Engine>
class BbsBackend final : public GlobalSearchBackend {
 public:
  BbsBackend(RegistrationCloud::ConstPtr map,GlobalOptions opt)
    :map_(std::move(map)),options_(std::move(opt)) {}
  const char *name() const override {return options_.backend.c_str();}
  GlobalSeeds search(const RegistrationCloud::ConstPtr &scan,const Eigen::Matrix4f &guess) override {
    GlobalSeeds result;
    if(!engine_) {
      const auto begin=std::chrono::steady_clock::now();
#ifdef ROBOCON_HAS_BBS_CUDA
      if constexpr(std::is_same_v<Engine,gpu::BBS3D>) {
        int count=0;
        const auto error=cudaGetDeviceCount(&count);
        if(error!=cudaSuccess || count<1)
          throw std::runtime_error("bbs_gpu_gicp requires a working CUDA GPU; select bbs_cpu_gicp explicitly");
      }
#endif
      engine_=std::make_unique<Engine>();
      if constexpr(std::is_same_v<Engine,cpu::BBS3D>)
        engine_->set_num_threads(options_.bbs_cpu_threads);
#ifdef ROBOCON_HAS_BBS_CUDA
      else engine_->set_branch_copy_size(options_.bbs_gpu_batch_size);
#endif
      std::vector<Eigen::Matrix<Scalar,3,1>> target;
      for(const auto &p:map_->points)
        if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))target.emplace_back(p.x,p.y,p.z);
      if(target.size()<50)throw std::runtime_error("Too few finite map points for BBS");
      engine_->set_tar_points(target,options_.bbs_resolution,options_.bbs_levels);
      pcl::PointXYZI lo,hi;pcl::getMinMax3D(*map_,lo,hi);
      engine_->set_trans_search_range(Eigen::Matrix<Scalar,3,1>(lo.x-1,lo.y-1,lo.z-1),
                                     Eigen::Matrix<Scalar,3,1>(hi.x+1,hi.y+1,hi.z+1));
      result.map_build_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-begin).count();
    }
    // Gravity/mount prealignment: BBS residual roll/pitch remain centered on
    // zero even for a downward sensor or a map with downward Z. Compose the
    // returned pose with this rotation to map the original scan correctly.
    const float yaw=std::atan2(guess(1,0),guess(0,0));
    const Eigen::Matrix3f tilt=Eigen::AngleAxisf(-yaw,Eigen::Vector3f::UnitZ())*guess.block<3,3>(0,0);
    Eigen::Matrix4f prealignment=Eigen::Matrix4f::Identity();prealignment.block<3,3>(0,0)=tilt;
    RegistrationCloud filtered;
    pcl::VoxelGrid<pcl::PointXYZI> filter;filter.setInputCloud(scan);
    filter.setLeafSize(options_.bbs_scan_voxel,options_.bbs_scan_voxel,options_.bbs_scan_voxel);
    filter.filter(filtered);
    std::vector<Eigen::Matrix<Scalar,3,1>> source;
    for(const auto &p:filtered.points) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      const Eigen::Vector3f aligned=tilt*Eigen::Vector3f(p.x,p.y,p.z);
      source.push_back(aligned.template cast<Scalar>());
    }
    if(source.size()<50){result.detail="too few finite scan points for BBS";return result;}
    const Scalar window=options_.bbs_roll_pitch_window;
    engine_->set_src_points(source);
    engine_->set_angular_search_range(Eigen::Matrix<Scalar,3,1>(-window,-window,-M_PI),
                                      Eigen::Matrix<Scalar,3,1>(window,window,M_PI));
    engine_->set_score_threshold_percentage(options_.bbs_score_threshold);
    engine_->enable_timeout();engine_->set_timeout_duration_in_msec(options_.bbs_timeout_ms);
    engine_->localize();
    result.timed_out=engine_->has_timed_out();
    result.coarse_score=engine_->get_best_score_percentage();
    if(engine_->has_localized()&&!result.timed_out) {
      for(const auto &pose:engine_->get_candidate_poses())
        result.poses.push_back(pose.template cast<float>()*prealignment);
      if(result.poses.empty())result.poses.push_back(engine_->get_global_pose().template cast<float>()*prealignment);
    }
    result.detail="official 3D-BBS, occupied_voxel_fraction="+std::to_string(result.coarse_score)+
      ", candidates="+std::to_string(result.poses.size())+", search_ms="+
      std::to_string(engine_->get_elapsed_time());
    return result;
  }
 private:
  RegistrationCloud::ConstPtr map_;
  GlobalOptions options_;
  std::unique_ptr<Engine> engine_;
};
}  // namespace

std::shared_ptr<GlobalSearchBackend> makeBbsSearchBackend(
  const RegistrationCloud::ConstPtr &map,const GlobalOptions &opt) {
  if(opt.backend=="bbs_cpu_gicp")return std::make_shared<BbsBackend<double,cpu::BBS3D>>(map,opt);
#ifdef ROBOCON_HAS_BBS_CUDA
  if(opt.backend=="bbs_gpu_gicp")return std::make_shared<BbsBackend<float,gpu::BBS3D>>(map,opt);
#else
  if(opt.backend=="bbs_gpu_gicp")throw std::runtime_error("CUDA BBS backend was not built; select bbs_cpu_gicp");
#endif
  throw std::invalid_argument("Unknown BBS backend: "+opt.backend);
}
}  // namespace robocon
