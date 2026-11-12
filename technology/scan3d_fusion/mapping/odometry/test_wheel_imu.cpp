#include "wheel_imu.hpp"
#include <cassert>
#include <limits>
int main() {
  rb_odometry::WheelImu straight;
  straight.update(0,0,0,0,.24,true);
  for(int i=1;i<=100;++i) assert(straight.update(i*.02,i*.01,i*.01,0,.24,true));
  assert(std::abs(straight.x-1)<1e-9 && std::abs(straight.y)<1e-9);
  assert(straight.covariance[0]>0 && straight.covariance[8]>0);
  auto saved=straight.x;
  assert(!straight.update(1,9,9,0,.24,true));
  assert(straight.x==saved);
  assert(!straight.update(2.02,1.1,1.1,0,.24,false));
  assert(!straight.update(2.04,1.2,1.2,0,.24,true));
  assert(straight.x==saved); // suspended wheel motion never enters the pose
  assert(!straight.update(3,1.3,1.3,0,.24,true));
  assert(straight.x==saved); // gap reseeds counters
  assert(!straight.update(4,std::numeric_limits<double>::quiet_NaN(),0,0,.24,true));
  rb_odometry::WheelImu turn;
  turn.update(0,0,0,0,.24,true);
  for(int i=1;i<=100;++i) turn.update(i*.02,-i*.0012,i*.0012,i*.01,.24,true);
  assert(std::abs(turn.yaw-1)<1e-9 && std::abs(turn.x)<1e-9);
  rb_odometry::WheelImu slipping;
  slipping.update(0,0,0,0,.24,true);
  slipping.update(.02,-.05,.05,0,.24,true);
  assert(slipping.slip && std::abs(slipping.yaw)<1e-9);
}
