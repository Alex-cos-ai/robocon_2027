#pragma once
#include <gtsam/geometry/Pose3.h>
#include <deque>

// Interpolate only bracketed measurements. Never silently attach a scan to
// the newest (possibly much older/newer) odometry measurement.
class StampedPoseBuffer {
 public:
  struct Entry {int64_t stamp;gtsam::Pose3 pose;};
  void add(int64_t stamp,const gtsam::Pose3 &pose) {
    if(!data_.empty()&&stamp<data_.back().stamp)data_.clear();
    if(!data_.empty()&&stamp==data_.back().stamp){data_.back().pose=pose;return;}
    data_.push_back({stamp,pose});
    while(data_.size()>2&&stamp-data_.front().stamp>30000000000LL)data_.pop_front();
  }
  bool lookup(int64_t stamp,double max_gap,gtsam::Pose3 &pose) const {
    for(size_t i=0;i<data_.size();++i) {
      if(data_[i].stamp==stamp){pose=data_[i].pose;return true;}
      if(i==0||data_[i].stamp<stamp)continue;
      const auto &a=data_[i-1],&b=data_[i];
      if(stamp<a.stamp||(b.stamp-a.stamp)*1e-9>max_gap)return false;
      const double ratio=double(stamp-a.stamp)/(b.stamp-a.stamp);
      pose=gtsam::Pose3(gtsam::Rot3(a.pose.rotation().toQuaternion().slerp(ratio,b.pose.rotation().toQuaternion())),
        (1-ratio)*a.pose.translation()+ratio*b.pose.translation());
      return true;
    }
    return false;
  }
  int64_t newest() const {return data_.empty()?0:data_.back().stamp;}
 private:
  std::deque<Entry> data_;
};
