#pragma once
#include <mujoco/mujoco.h>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include "tof_calibration.hpp"

// Simulator-only measurement adapter. World coordinates never leave this class.
class TofSensor {
 public:
  explicit TofSensor(unsigned seed): rng_(seed) {}
  std::array<double,64> sample(const mjModel* m, mjData* d) {
    std::array<double,64> result{};
    const int camera=mj_name2id(m,mjOBJ_CAMERA,"tof_camera");
    const int body=mj_name2id(m,mjOBJ_BODY,"body_link");
    if(camera<0 || body<0) return result;
    // Temporarily mask robot geoms, including articulated legs, from sensor rays.
    std::vector<int> saved(m->ngeom);
    for(int g=0;g<m->ngeom;++g) {
      saved[g]=m->geom_group[g];
      int b=m->geom_bodyid[g];
      while(b>0 && b!=body) b=m->body_parentid[b];
      if(b==body) m->geom_group[g]=5;
    }
    const mjtByte groups[6]={1,1,1,1,1,0};
    const auto* r=d->cam_xmat+9*camera;
    const double h=std::tan(tof::fov*3.141592653589793/360);
    for(int i=0;i<64;++i) {
      const int row=i/8,col=i%8;
      const double u=(2*(col+.5)/8-1)*h,v=(1-2*(row+.5)/8)*h;
      mjtNum direction[3];
      double norm=0;
      for(int j=0;j<3;++j) { direction[j]=-r[j*3+2]+u*r[j*3]+v*r[j*3+1]; norm+=direction[j]*direction[j]; }
      for(auto& x:direction) x/=std::sqrt(norm);
      int geom=-1;
      double range=mj_ray(m,d,d->cam_xpos+3*camera,direction,groups,1,body,&geom,nullptr);
      if(range<tof::minimum || range>tof::maximum) continue;
      double bound=range<=.2?tof::near_noise:range*tof::far_noise;
      if((row==0||row==7)&&(col==0||col==7)) bound+=range*tof::corner_noise;
      range+=std::clamp(normal_(rng_)*bound*tof::sigma,-bound,bound);
      result[i]=std::clamp(range,tof::minimum,tof::maximum);
    }
    for(int g=0;g<m->ngeom;++g) m->geom_group[g]=saved[g];
    return result;
  }
 private:
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0,1};
};
