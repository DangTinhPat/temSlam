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

  rb_odometry::WheelImu drifting;
  drifting.update(0,0,0,0,.24,true);
  for(int i=1;i<=6000;++i)
    assert(drifting.update(i*.02,0,0,.003*i*.02,.24,true));
  assert(std::abs(drifting.yaw)<.02); // residual gyro bias must not accumulate for minutes

  rb_odometry::WheelImu scale_error;
  scale_error.update(0,0,0,0,.24,true);
  for(int i=1;i<=3000;++i) {
    const double angle=.5*i*.02;
    assert(scale_error.update(i*.02,-.12*angle,.12*angle,1.02*angle,.24,true));
  }
  assert(std::abs(scale_error.yaw-30)<.055); // wheel agreement tracks slow gyro rate error

  rb_odometry::WheelImu repeated_slip;
  repeated_slip.update(0,0,0,0,.24,true);
  for(int i=1;i<=100;++i) {
    assert(repeated_slip.update(i*.02,-i*.02,i*.02,0,.24,true));
    assert(repeated_slip.slip && std::abs(repeated_slip.yaw)<1e-9);
  }
  assert(repeated_slip.update(2.02,-2,2,0,.24,true));
  assert(std::abs(repeated_slip.yaw)<1e-9); // slipping wheels must not teach a false gyro bias

  rb_odometry::WheelImu slow_slip;
  slow_slip.update(0,0,0,0,.24,true);
  for(int i=1;i<=6000;++i) {
    const double wheel_angle=.02*i*.02;
    assert(slow_slip.update(i*.02,-.12*wheel_angle,.12*wheel_angle,0,.24,true));
  }
  assert(std::abs(slow_slip.yaw)<.06); // wheel creep exceeds the gyro-error budget, not a bias sample
}
