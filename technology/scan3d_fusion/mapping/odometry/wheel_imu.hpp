#pragma once
#include <array>
#include <algorithm>
#include <cmath>

namespace rb_odometry {
// Planar axle odometry. Inputs are wheel travel (m) and unwrapped, bias-corrected gyro yaw (rad).
class WheelImu {
 public:
  double x=0, y=0, yaw=0;
  std::array<double,9> covariance{}; // row-major x,y,yaw covariance
  bool slip=false;
  bool update(double t,double left,double right,double gyro,double track,bool supported) {
    for(double v:{t,left,right,gyro,track}) if(!std::isfinite(v)) return false;
    if(t<0 || t<=time_ || track<=.05 || track>1) return false;
    const double dt=time_<0?0:t-time_;
    const double dl=left-left_,dr=right-right_,dg=gyro-gyro_;
    const bool ready=seeded_;
    time_=t;left_=left;right_=right;gyro_=gyro;
    seeded_=supported;slip=false;
    if(!supported || !ready || dt>.5 || std::abs(dl)>.5 || std::abs(dr)>.5 || std::abs(dg)>1) {
      if(dt>0) {covariance[0]+=.01*dt;covariance[4]+=.01*dt;covariance[8]+=.01*dt;}
      return false;
    }
    const double dw=(dr-dl)/track, ds=(dl+dr)*.5;
    // shortcut: simulation noise defaults; calibrate these variances on hardware before navigation.
    const double wheel_var=std::pow(.001+.03*std::abs(dl),2)+std::pow(.001+.03*std::abs(dr),2);
    const double rw=wheel_var/(track*track), rg=.006*.006*dt+.003*.003*dt*dt;
    slip=std::abs(dw-dg)>std::max(.04,5*std::sqrt(rw+rg));
    const double gain=slip?0:rg/(rw+rg);
    const double da=dg+gain*(dw-dg), va=slip?rg+.01*dt:rg*rw/(rg+rw);
    const double vd=wheel_var*.25+(slip?.04*dt:0);
    const double c=std::cos(yaw+da*.5),s=std::sin(yaw+da*.5);
    const std::array<double,9> F{1,0,-ds*s,0,1,ds*c,0,0,1};
    std::array<double,9> next{};
    const std::array<double,3> gd{c,s,0},ga{-.5*ds*s,.5*ds*c,1};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
      for(int k=0;k<3;++k) for(int l=0;l<3;++l)
        next[i*3+j]+=F[i*3+k]*covariance[k*3+l]*F[j*3+l];
      next[i*3+j]+=gd[i]*vd*gd[j]+ga[i]*va*ga[j];
    }
    covariance=next;x+=ds*c;y+=ds*s;yaw+=da;
    return true;
  }
 private:
  double time_=-1,left_=0,right_=0,gyro_=0;
  bool seeded_=false;
};
}
