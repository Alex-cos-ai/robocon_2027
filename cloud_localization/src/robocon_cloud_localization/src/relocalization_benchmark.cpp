#include "global_relocalization.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <stdexcept>

static robocon::RegistrationCloud::Ptr load(const std::string &path) {
  robocon::RegistrationCloud::Ptr source(new robocon::RegistrationCloud),out(new robocon::RegistrationCloud);
  if(pcl::io::loadPCDFile(path,*source)<0 || source->empty())
    throw std::runtime_error("Cannot read PCD: "+path);
  pcl::VoxelGrid<pcl::PointXYZI> filter;filter.setInputCloud(source);
  filter.setLeafSize(.10f,.10f,.10f);filter.filter(*out);return out;
}
int main(int argc,char **argv) {
  if(argc!=4 && argc!=5 && argc!=8) {
    std::cerr<<"Usage: relocalization_benchmark MAP.pcd SCAN.pcd BACKEND [REPEATS [ROLL PITCH YAW]]\n"
      <<"BACKEND: grid_gicp | bbs_cpu_gicp | bbs_gpu_gicp; angles are radians\n";
    return 2;
  }
  try {
    auto map=load(argv[1]),scan=load(argv[2]);
    robocon::GlobalOptions options;options.backend=argv[3];
    auto backend=robocon::makeGlobalSearchBackend(map,options);
    Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();
    if(argc==8)guess.block<3,3>(0,0)=(Eigen::AngleAxisf(std::stof(argv[7]),Eigen::Vector3f::UnitZ())*
      Eigen::AngleAxisf(std::stof(argv[6]),Eigen::Vector3f::UnitY())*
      Eigen::AngleAxisf(std::stof(argv[5]),Eigen::Vector3f::UnitX())).toRotationMatrix();
    const int repeats=argc>=5?std::stoi(argv[4]):1;
    if(repeats<1 || repeats>100)throw std::invalid_argument("REPEATS must be 1..100");
    bool success=true;
    for(int i=0;i<repeats;++i) {
      const auto result=robocon::globalRelocalize(scan,map,guess,options,backend);
      const Eigen::Vector3f rpy(std::atan2(result.pose(2,1),result.pose(2,2)),
        std::atan2(-result.pose(2,0),std::hypot(result.pose(0,0),result.pose(1,0))),
        std::atan2(result.pose(1,0),result.pose(0,0)));
      std::cout<<std::fixed<<std::setprecision(6)
        <<"RESULT {\"backend\":\""<<result.backend<<"\",\"run\":"<<i
        <<",\"found\":"<<(result.found?"true":"false")<<",\"timed_out\":"<<(result.timed_out?"true":"false")
        <<",\"map_build_ms\":"<<result.map_build_ms<<",\"search_ms\":"<<result.search_ms
        <<",\"refine_ms\":"<<result.refine_ms<<",\"total_ms\":"<<result.total_ms
        <<",\"coarse_score\":"<<result.coarse_score<<",\"fitness_m2\":"<<result.fitness
        <<",\"overlap\":"<<result.overlap<<",\"xyz\":["<<result.pose(0,3)<<","<<result.pose(1,3)<<","<<result.pose(2,3)
        <<"],\"rpy\":["<<rpy.x()<<","<<rpy.y()<<","<<rpy.z()<<"]}"<<std::endl;
      std::cerr<<result.detail<<"\n";success=success&&result.found;
    }
    return success?0:1;
  } catch(const std::exception &e) {std::cerr<<e.what()<<"\n";return 2;}
}
