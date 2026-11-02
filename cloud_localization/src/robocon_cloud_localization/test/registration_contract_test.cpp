#include "global_relocalization.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <random>

static void require(bool condition,const char *message) {
  if(!condition)throw std::runtime_error(message);
}
int main(int argc,char **argv) {
  try {
    const std::string backend=argc>1?argv[1]:"bbs_cpu_gicp";
    robocon::RegistrationCloud::Ptr source(new robocon::RegistrationCloud),map(new robocon::RegistrationCloud);
    std::mt19937 random(19);std::uniform_real_distribution<float> value(-2,2);
    const Eigen::Matrix3f tilt=Eigen::AngleAxisf(float(M_PI),Eigen::Vector3f::UnitX()).toRotationMatrix();
    const Eigen::Matrix3f rotation=Eigen::AngleAxisf(.40f,Eigen::Vector3f::UnitZ())*tilt;
    const Eigen::Vector3f translation(.3f,-.4f,.6f);
    for(int i=0;i<1500;++i) {
      Eigen::Vector3f p(value(random),value(random),value(random));
      // Asymmetric dense volume supplies an unambiguous six-DoF constraint.
      p.z()*=.6f;p.x()+=.2f*p.y()*p.y();
      pcl::PointXYZI a;a.x=p.x();a.y=p.y();a.z=p.z();a.intensity=0;source->push_back(a);
      const Eigen::Vector3f q=rotation*p+translation;
      pcl::PointXYZI b;b.x=q.x();b.y=q.y();b.z=q.z();b.intensity=0;map->push_back(b);
    }
    robocon::GlobalOptions options;options.backend=backend;
    auto search=robocon::makeGlobalSearchBackend(map,options);
    Eigen::Matrix4f guess=Eigen::Matrix4f::Identity();guess.block<3,3>(0,0)=tilt;
    const auto aligned=robocon::globalRelocalize(source,map,guess,options,search);
    require(aligned.found,"Downward-axis registration failed");
    require((aligned.pose.block<3,1>(0,3)-translation).norm()<.03,"Incorrect XYZ");
    require(Eigen::AngleAxisf(aligned.pose.block<3,3>(0,0)*rotation.transpose()).angle()<.03,"Incorrect tilt/yaw composition");
    const auto cached=robocon::globalRelocalize(source,map,guess,options,search);
    require(cached.found&&cached.map_build_ms==0,"Map was rebuilt on repeated query");
    require(cached.backend==backend&&cached.total_ms>=cached.refine_ms,"Missing interface metrics");
    robocon::RegistrationCloud::Ptr tiny(new robocon::RegistrationCloud);
    require(!robocon::globalRelocalize(tiny,map,guess,options,search).found,"Accepted empty scan");
    guess(0,0)=std::numeric_limits<float>::quiet_NaN();
    require(!robocon::globalRelocalize(source,map,guess,options,search).found,"Accepted nonfinite guess");
    options.bbs_timeout_ms=1;auto timeout_backend=robocon::makeGlobalSearchBackend(map,options);
    const auto timeout=robocon::globalRelocalize(source,map,Eigen::Matrix4f::Identity(),options,timeout_backend);
    require(timeout.timed_out&&!timeout.found,"Search timeout was not rejected");
    options.backend="does_not_exist";
    bool rejected=false;
    try{robocon::makeGlobalSearchBackend(map,options);}catch(const std::invalid_argument &){rejected=true;}
    require(rejected,"Unknown backend silently accepted");
    rejected=false;
    try{robocon::globalRelocalize(source,map,Eigen::Matrix4f::Identity(),options,search);}
    catch(const std::invalid_argument &){rejected=true;}
    require(rejected,"Cached backend mismatch silently accepted");
    std::cout<<"PASS "<<backend<<": downward tilt, cached map, invalid inputs, timeout, unknown backend\n";
    return 0;
  } catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}
}
