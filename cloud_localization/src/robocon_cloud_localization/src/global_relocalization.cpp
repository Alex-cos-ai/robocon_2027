#include "global_relocalization.hpp"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace robocon {
namespace {
RegistrationCloud::Ptr filtered(const RegistrationCloud::ConstPtr &input, float leaf) {
  RegistrationCloud::Ptr out(new RegistrationCloud);
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(input); filter.setLeafSize(leaf, leaf, leaf); filter.filter(*out);
  return out;
}
struct Candidate {
  double score;
  Eigen::Matrix4f pose;
};
double yaw(const Eigen::Matrix4f &pose) { return std::atan2(pose(1,0),pose(0,0)); }
bool distinct(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b) {
  return (a.block<3,1>(0,3)-b.block<3,1>(0,3)).norm() > 0.75 ||
    std::abs(std::remainder(yaw(a)-yaw(b), 2*M_PI)) > 0.35;
}
}  // namespace

static GlobalSeeds gridSearch(const RegistrationCloud::ConstPtr &scan,
                             const RegistrationCloud::ConstPtr &map,
                             const Eigen::Matrix4f &guess, const GlobalOptions &opt) {
  GlobalSeeds result;
  auto target = filtered(map, opt.voxel);
  auto source = filtered(scan, opt.voxel);
  if (target->size()<50 || source->size()<50) {
    result.detail="too few points for global registration"; return result;
  }
  pcl::PointXYZI lo,hi; pcl::getMinMax3D(*target,lo,hi);
  const float cell=opt.voxel, radius=0.90f, max_cost=radius*radius;
  const Eigen::Vector3f origin(lo.x-radius,lo.y-radius,lo.z-radius);
  const Eigen::Vector3i dims(
    int(std::ceil((hi.x-lo.x+2*radius)/cell))+1,
    int(std::ceil((hi.y-lo.y+2*radius)/cell))+1,
    int(std::ceil((hi.z-lo.z+2*radius)/cell))+1);
  const int64_t cells=int64_t(dims.x())*dims.y()*dims.z();
  if(cells<=0 || cells>4000000) {
    result.detail="map exceeds global search grid budget; increase localization_global_voxel";
    return result;
  }
  std::vector<float> distance(cells,max_cost);
  const auto index=[&](int x,int y,int z){return (z*dims.y()+y)*dims.x()+x;};
  const int spread=int(std::ceil(radius/cell));
  for(const auto &p:target->points) {
    const Eigen::Vector3f v(p.x,p.y,p.z);
    const Eigen::Vector3i center=((v-origin)/cell).array().floor().cast<int>();
    for(int z=std::max(0,center.z()-spread);z<=std::min(dims.z()-1,center.z()+spread);++z)
      for(int y=std::max(0,center.y()-spread);y<=std::min(dims.y()-1,center.y()+spread);++y)
        for(int x=std::max(0,center.x()-spread);x<=std::min(dims.x()-1,center.x()+spread);++x) {
          const Eigen::Vector3f c=origin+cell*Eigen::Vector3f(x+.5f,y+.5f,z+.5f);
          auto &d=distance[index(x,y,z)]; d=std::min(d,(c-v).squaredNorm());
        }
  }
  // Sample spatially downsampled points deterministically, not the original
  // ray ordering. Equal voxel weighting reduces domination by dense floor hits.
  std::vector<Eigen::Vector3f> points;
  const size_t count=std::min<size_t>(source->size(),256);
  for(size_t i=0;i<count;++i) {
    const auto &p=source->points[i*source->size()/count];
    points.emplace_back(p.x,p.y,p.z);
  }
  // Preserve the initial roll/pitch while searching a full circle of yaw.
  const Eigen::Matrix3f tilt=Eigen::AngleAxisf(-yaw(guess),Eigen::Vector3f::UnitZ())*
    guess.block<3,3>(0,0);
  const double z_lo=lo.z-1.0, z_hi=hi.z+1.0;
  double step=opt.xy_step;
  const double estimate=(hi.x-lo.x+2.0)/step*(hi.y-lo.y+2.0)/step*
    ((z_hi-z_lo)/opt.z_step+1)*(2*M_PI/opt.yaw_step+1);
  // Bound search work on unusually large maps instead of blocking indefinitely.
  if(estimate>250000) step*=std::sqrt(estimate/250000);
  std::vector<Candidate> candidates;
  candidates.reserve(256);
  size_t hypotheses=0;
  for(double angle=-M_PI;angle<M_PI-1e-6;angle+=opt.yaw_step) {
    const Eigen::Matrix3f rot=Eigen::AngleAxisf(angle,Eigen::Vector3f::UnitZ())*tilt;
    std::vector<Eigen::Vector3f> rotated;rotated.reserve(points.size());
    for(const auto &p:points) rotated.push_back(rot*p);
    for(double z=z_lo;z<=z_hi;z+=opt.z_step)
      for(double y=lo.y-1.0;y<=hi.y+1.0;y+=step)
        for(double x=lo.x-1.0;x<=hi.x+1.0;x+=step) {
          ++hypotheses;
          const Eigen::Vector3f t(x,y,z);
          double cost=0;
          for(const auto &p:rotated) {
            const Eigen::Vector3i c=((p+t-origin)/cell).array().floor().cast<int>();
            cost+=(c.x()<0 || c.y()<0 || c.z()<0 || c.x()>=dims.x() ||
              c.y()>=dims.y() || c.z()>=dims.z()) ? max_cost : distance[index(c.x(),c.y(),c.z())];
          }
          cost/=points.size();
          if(candidates.size()>=192 && cost>=candidates.back().score) continue;
          Candidate c{cost,Eigen::Matrix4f::Identity()};
          c.pose.block<3,3>(0,0)=rot;c.pose.block<3,1>(0,3)=t;
          auto at=std::lower_bound(candidates.begin(),candidates.end(),cost,
            [](const Candidate &a,double score){return a.score<score;});
          candidates.insert(at,c);
          if(candidates.size()>192)candidates.pop_back();
        }
  }
  // Retain candidates in different basins rather than spending all refinements
  // on neighbors around a single grid point.
  std::vector<Candidate> seeds;
  for(const auto &candidate:candidates) {
    if(std::all_of(seeds.begin(),seeds.end(),[&](const Candidate &s){return distinct(candidate.pose,s.pose);}))
      seeds.push_back(candidate);
    if(seeds.size()>=static_cast<size_t>(opt.refine_candidates)) break;
  }
  for (const auto &seed : seeds) result.poses.push_back(seed.pose);
  result.detail=std::to_string(hypotheses)+" grid poses, "+std::to_string(seeds.size())+" GICP refinements";
  return result;
}

GlobalResult refineGlobalCandidates(const RegistrationCloud::ConstPtr &scan,
  const RegistrationCloud::ConstPtr &map, const std::vector<Eigen::Matrix4f> &seeds,
  const GlobalOptions &opt) {
  GlobalResult result;
  const double max_cost=0.90*0.90;
  pcl::search::KdTree<pcl::PointXYZI> tree;tree.setInputCloud(map);
  auto refine_source=filtered(scan,0.12f);
  std::vector<Candidate> accepted;
  std::vector<double> overlaps;
  for(const auto &seed:seeds) {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> gicp;
    gicp.setMaximumIterations(45);gicp.setMaxCorrespondenceDistance(0.90);
    gicp.setTransformationEpsilon(1e-5);gicp.setEuclideanFitnessEpsilon(1e-5);
    gicp.setInputSource(refine_source);gicp.setInputTarget(map);
    RegistrationCloud aligned;gicp.align(aligned,seed);
    if(!gicp.hasConverged() || !gicp.getFinalTransformation().allFinite())continue;
    // Score ALL points, counting misses. Fitness on just the nearest subset
    // otherwise lets a small overlapping floor patch look like a good match.
    double squared=0;size_t inliers=0;
    std::vector<int> ids(1);std::vector<float> distances(1);
    for(const auto &p:aligned.points) {
      if(tree.nearestKSearch(p,1,ids,distances)>0) {
        squared+=std::min<double>(distances[0],max_cost);
        if(distances[0]<=opt.inlier_distance*opt.inlier_distance)++inliers;
      } else squared+=max_cost;
    }
    const double score=squared/aligned.size(),overlap=double(inliers)/aligned.size();
    if(!std::isfinite(score) || score>opt.fitness_max || overlap<opt.overlap_min)continue;
    accepted.push_back({score,gicp.getFinalTransformation()});overlaps.push_back(overlap);
  }
  if(accepted.empty()) {
    result.detail="no validated global candidate";return result;
  }
  const size_t best=std::min_element(accepted.begin(),accepted.end(),
    [](const Candidate &a,const Candidate &b){return a.score<b.score;})-accepted.begin();
  for(size_t i=0;i<accepted.size();++i) {
    if(i!=best && distinct(accepted[i].pose,accepted[best].pose) &&
       accepted[i].score<=accepted[best].score*opt.ambiguity_ratio+1e-5) {
      result.detail="ambiguous global matches; use RViz 2D Pose Estimate to identify the field location";
      return result;
    }
  }
  result.found=true;result.pose=accepted[best].pose;result.fitness=accepted[best].score;
  result.overlap=overlaps[best];
  result.detail="GICP validation passed";
  return result;
}


class GridSearchBackend final : public GlobalSearchBackend {
 public:
  GridSearchBackend(RegistrationCloud::ConstPtr map, GlobalOptions options)
    : map_(std::move(map)),options_(std::move(options)) {}
  const char *name() const override {return "grid_gicp";}
  GlobalSeeds search(const RegistrationCloud::ConstPtr &scan, const Eigen::Matrix4f &guess) override {
    return gridSearch(scan,map_,guess,options_);
  }
 private:
  RegistrationCloud::ConstPtr map_;
  GlobalOptions options_;
};
std::shared_ptr<GlobalSearchBackend> makeBbsSearchBackend(
  const RegistrationCloud::ConstPtr &map,const GlobalOptions &options);
std::shared_ptr<GlobalSearchBackend> makeGlobalSearchBackend(
  const RegistrationCloud::ConstPtr &map,const GlobalOptions &options) {
  if(!map || map->empty())throw std::invalid_argument("Global registration requires a nonempty map");
  if(!std::isfinite(options.bbs_resolution)||options.bbs_resolution<=0 ||
     !std::isfinite(options.bbs_scan_voxel)||options.bbs_scan_voxel<=0 ||
     !std::isfinite(options.bbs_roll_pitch_window)||options.bbs_roll_pitch_window<=0 ||
     !std::isfinite(options.bbs_score_threshold)||options.bbs_score_threshold<=0||options.bbs_score_threshold>=1 ||
     options.bbs_levels<1 || options.bbs_levels>8 || options.bbs_timeout_ms<1 ||
     options.bbs_cpu_threads<1 || options.bbs_gpu_batch_size<1)
    throw std::invalid_argument("Invalid BBS configuration");
  if (options.backend=="grid_gicp") return std::make_shared<GridSearchBackend>(map,options);
  if (options.backend=="bbs_cpu_gicp" || options.backend=="bbs_gpu_gicp")
    return makeBbsSearchBackend(map,options);
  throw std::invalid_argument("Unknown relocalization backend: "+options.backend);
}
GlobalResult globalRelocalize(const RegistrationCloud::ConstPtr &scan,
  const RegistrationCloud::ConstPtr &map,const Eigen::Matrix4f &guess,
  const GlobalOptions &options,const std::shared_ptr<GlobalSearchBackend> &prepared) {
  using Clock=std::chrono::steady_clock;
  const auto begin=Clock::now();
  if(!scan || scan->size()<50 || !guess.allFinite()) {
    GlobalResult invalid;invalid.backend=options.backend;
    invalid.detail="Invalid/insufficient scan or nonfinite initial rotation";return invalid;
  }
  auto backend=prepared ? prepared : makeGlobalSearchBackend(map,options);
  if(options.backend!=backend->name())
    throw std::invalid_argument("Cached search backend differs from GlobalOptions.backend");
  auto seeds=backend->search(scan,guess);
  const auto searched=Clock::now();
  auto result=refineGlobalCandidates(scan,map,seeds.poses,options);
  const auto end=Clock::now();
  result.backend=backend->name();result.timed_out=seeds.timed_out;
  result.map_build_ms=seeds.map_build_ms;result.coarse_score=seeds.coarse_score;
  result.search_ms=std::chrono::duration<double,std::milli>(searched-begin).count()-seeds.map_build_ms;
  result.refine_ms=std::chrono::duration<double,std::milli>(end-searched).count();
  result.total_ms=std::chrono::duration<double,std::milli>(end-begin).count();
  if(seeds.timed_out) {result.found=false;result.detail="BBS timeout; no global solution accepted";}
  result.detail=std::string(backend->name())+": "+seeds.detail+"; "+result.detail;
  return result;
}
}  // namespace robocon
