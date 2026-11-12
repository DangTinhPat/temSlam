#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "tof_calibration.hpp"
#include "../odometry/wheel_imu.hpp"
namespace fs=std::filesystem;
constexpr double RES=.025,ORIGIN=-6.4;
constexpr int W=static_cast<int>(12.8/RES),H=W,Z=static_cast<int>(.8/RES);
using Vec=std::array<double,3>;
Vec rotate(Vec p,double pitch,double roll,double yaw) {
  Vec r{p[0],std::cos(roll)*p[1]-std::sin(roll)*p[2],std::sin(roll)*p[1]+std::cos(roll)*p[2]};
  Vec q{std::cos(pitch)*r[0]+std::sin(pitch)*r[2],r[1],-std::sin(pitch)*r[0]+std::cos(pitch)*r[2]};
  return {std::cos(yaw)*q[0]-std::sin(yaw)*q[1],std::sin(yaw)*q[0]+std::cos(yaw)*q[1],q[2]};
}
Vec pan(Vec p,double yaw) {
  constexpr double px=.025;
  double x=p[0]-px,y=p[1];
  return {px+std::cos(yaw)*x-std::sin(yaw)*y,
          std::sin(yaw)*x+std::cos(yaw)*y,p[2]};
}
Vec panDirection(Vec p,double yaw) {
  return {std::cos(yaw)*p[0]-std::sin(yaw)*p[1],
          std::sin(yaw)*p[0]+std::cos(yaw)*p[1],p[2]};
}
int index(Vec p) {
  int x=int(std::floor((p[0]-ORIGIN)/RES)),y=int(std::floor((p[1]-ORIGIN)/RES)),z=int(std::floor(p[2]/RES));
  if(x<0||x>=W||y<0||y>=H||z<1||z>=Z) return -1;
  return (z*H+y)*W+x;
}
int main(int argc,char** argv) {
  if(argc!=2) {std::cerr<<"usage: rb_tof_mapper OUTPUT_DIRECTORY < sensor.log\n";return 2;}
  fs::path out=argv[1]; fs::create_directories(out);
  std::vector<int8_t> voxels(W*H*Z,0);
  rb_odometry::WheelImu odom;
  double &x=odom.x,&y=odom.y,&yaw=odom.yaw;
  double last_t=-1; unsigned frame=0,rejected=0;
  auto last_snapshot=std::chrono::steady_clock::now();
  auto snapshot=[&](double t,int valid,int count) {
    int occupied=0,free=0;
    std::vector<unsigned char> pixels(W*H,127);
    for(int row=0;row<H;++row) for(int col=0;col<W;++col) {
      bool occ=false,observed=false;
      for(int z=1;z<Z;++z) {int v=voxels[(z*H+row)*W+col];occ|=v>=6;observed|=v<0;}
      unsigned char p=occ?20:observed?245:127;
      pixels[(H-1-row)*W+col]=p; occupied+=occ;free+=!occ&&observed;
    }
    {std::ofstream f(out/"map.tmp",std::ios::binary); f<<"P5\n"<<W<<' '<<H<<"\n255\n";f.write(reinterpret_cast<char*>(pixels.data()),pixels.size());}
    fs::rename(out/"map.tmp",out/"map.pgm");
    std::vector<std::array<double,3>> points;
    for(int z=1;z<Z;++z) for(int row=0;row<H;++row) for(int col=0;col<W;++col)
      if(voxels[(z*H+row)*W+col]>=6)
        points.push_back({ORIGIN+(col+.5)*RES,ORIGIN+(row+.5)*RES,(z+.5)*RES});
    {std::ofstream f(out/"map.ply.tmp");
      f<<"ply\nformat ascii 1.0\nelement vertex "<<points.size()
       <<"\nproperty float x\nproperty float y\nproperty float z\n"
         "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
      for(const auto& p:points) {
        const int level=std::clamp(int(p[2]/(Z*RES)*255),0,255);
        f<<p[0]<<' '<<p[1]<<' '<<p[2]<<' '<<level<<' '<<80+level/2<<' '<<255-level<<'\n';
      }
    }
    fs::rename(out/"map.ply.tmp",out/"map.ply");
    {std::ofstream f(out/"state.tmp");f<<"{\"t\":"<<t<<",\"frame\":"<<frame<<",\"x\":"<<x<<",\"y\":"<<y<<",\"yaw\":"<<yaw
      <<",\"valid\":"<<valid<<",\"returns\":"<<count<<",\"occupied\":"<<occupied<<",\"free\":"<<free
      <<",\"rejected\":"<<rejected<<",\"resolution\":"<<RES<<",\"origin\":["<<ORIGIN<<','<<ORIGIN<<"],\"width\":"<<W<<",\"height\":"<<H
      <<",\"occupied_voxels_3d\":"<<points.size()<<",\"voxel_z_m\":"<<RES<<'}';}
    fs::rename(out/"state.tmp",out/"state.json");
  };
  std::ofstream odometry(out/"odometry.jsonl");
  odometry<<std::setprecision(17);
  std::ofstream trajectory(out/"trajectory.csv");trajectory<<"t,x,y,yaw,valid\n";
  snapshot(0,0,0);
  std::string line;
  while(std::getline(std::cin,line)) {
    if(line=="MAPRESET") {
      std::fill(voxels.begin(),voxels.end(),0); odom=rb_odometry::WheelImu{};last_t=-1;frame=0;
      trajectory<<"# reset\n";odometry<<"{\"reset\":true}\n";snapshot(0,0,0);continue;
    }
    if(line.rfind("TOFMAP ",0)!=0) continue;
    if(line.size()>8192) {++rejected;continue;}
    std::istringstream s(line.substr(7));
    double t=0,l=0,r=0,g=0,pitch=0,roll=0,ax=0,az=0,radius=0,track=0,head=0;int valid=0;
    std::array<double,64> ranges{};
    bool good=bool(s>>t>>valid>>l>>r>>g>>pitch>>roll>>ax>>az>>radius>>track);
    std::vector<double> frame_values;double value=0;
    while(s>>value)frame_values.push_back(value);
    good=good&&(frame_values.size()==64||frame_values.size()==65);
    if(frame_values.size()==65){head=frame_values.front();frame_values.erase(frame_values.begin());}
    if(frame_values.size()==64)std::copy(frame_values.begin(),frame_values.end(),ranges.begin());
    for(double v:{t,l,r,g,pitch,roll,ax,az,radius,track,head}) good=good&&std::isfinite(v);
    for(double v:ranges) good=good&&std::isfinite(v)&&v>=0&&v<=tof::maximum;
    if(!good||t<0||radius<=0||radius>.2||track<=.05||track>1||std::abs(ax)>.5||std::abs(az)>.5||
       (valid!=0&&valid!=1)||t<=last_t) {++rejected;continue;}
    if(std::abs(pitch)>.45||std::abs(roll)>.15) valid=0;
    if(!odom.update(t,l,r,g,track,valid)) valid=0;
    last_t=t;++frame;
    odometry<<"{\"time_s\":"<<t<<",\"x\":"<<x<<",\"y\":"<<y<<",\"yaw\":"<<yaw
      <<",\"valid\":"<<valid<<",\"slip\":"<<(odom.slip?"true":"false")
      <<",\"pitch\":"<<pitch<<",\"roll\":"<<roll<<",\"axle_body_x\":"<<ax
      <<",\"axle_body_z\":"<<az<<",\"wheel_radius\":"<<radius<<",\"head_yaw\":"<<head<<",\"covariance\":[";
    for(int i=0;i<9;++i) odometry<<(i?",":"")<<odom.covariance[i];
    odometry<<"]}\n";odometry.flush();
    if(std::abs(pitch)>.45||std::abs(roll)>.15||std::abs(x)>-ORIGIN-.5||std::abs(y)>-ORIGIN-.5) valid=0;
    int count=0;
    if(valid) {
      Vec right{tof::axes[0],tof::axes[1],tof::axes[2]},up{tof::axes[3],tof::axes[4],tof::axes[5]};
      Vec forward{-(right[1]*up[2]-right[2]*up[1]),-(right[2]*up[0]-right[0]*up[2]),-(right[0]*up[1]-right[1]*up[0])};
      Vec origin=pan({tof::position[0]+forward[0]*tof::window_offset-ax,
                      tof::position[1]+forward[1]*tof::window_offset,
                      tof::position[2]+forward[2]*tof::window_offset-az},head);
      origin=rotate(origin,pitch,roll,yaw);origin[0]+=x;origin[1]+=y;origin[2]+=radius;
      // Fit the observed support plane from lower ToF zones. IMU mounting/bias
      // error otherwise turns floor returns into concentric false obstacles.
      std::array<Vec,64> directions{}, endpoints{};
      std::vector<int> ground_candidates;
      double h=std::tan(tof::fov*3.141592653589793/360);
      for(int i=0;i<64;++i) {
        double u=(2*(i%8+.5)/8-1)*h,v=(1-2*(i/8+.5)/8)*h,norm=0;
        Vec ray;for(int j=0;j<3;++j){ray[j]=forward[j]+u*right[j]+v*up[j];norm+=ray[j]*ray[j];}
        for(auto& a:ray) a/=std::sqrt(norm);
        ray=panDirection(ray,head);
        directions[i]=rotate(ray,pitch,roll,yaw);
        for(int j=0;j<3;++j) endpoints[i][j]=origin[j]+ranges[i]*directions[i][j];
        if(i>=32 && ranges[i]>=tof::minimum && std::abs(endpoints[i][2])<.18)
          ground_candidates.push_back(i);
      }
      Vec plane{0,0,0};int best=0;
      for(size_t a=0;a<ground_candidates.size();++a)
      for(size_t b=a+1;b<ground_candidates.size();++b)
      for(size_t c=b+1;c<ground_candidates.size();++c) {
        auto p=endpoints[ground_candidates[a]],q=endpoints[ground_candidates[b]],r=endpoints[ground_candidates[c]];
        double ux=q[0]-p[0],uy=q[1]-p[1],uz=q[2]-p[2],vx=r[0]-p[0],vy=r[1]-p[1],vz=r[2]-p[2];
        double det=ux*vy-uy*vx;if(std::abs(det)<.003)continue;
        double pa=(uz*vy-uy*vz)/det,pb=(ux*vz-uz*vx)/det,pc=p[2]-pa*(p[0]-x)-pb*(p[1]-y);
        if(std::hypot(pa,pb)>.2 || std::abs(pc)>.10)continue;
        int n=0;for(int i:ground_candidates)if(std::abs(endpoints[i][2]-pa*(endpoints[i][0]-x)-pb*(endpoints[i][1]-y)-pc)<.015+.007*ranges[i])++n;
        if(n>best){best=n;plane={pa,pb,pc};}
      }
      std::unordered_map<int,int> updates;
      for(int i=0;i<64;++i) {
        double d=ranges[i]; if(d<tof::minimum||d>tof::maximum) continue;++count;
        Vec ray=directions[i];
        double margin=.025+d*tof::far_noise*tof::sigma;
        for(double distance=0;distance<d-margin;distance+=RES*.5) {
          Vec q;for(int j=0;j<3;++j)q[j]=origin[j]+distance*ray[j];int k=index(q);
          if(k>=0 && !updates.count(k)) updates[k]=-1;
        }
        Vec q;for(int j=0;j<3;++j)q[j]=origin[j]+d*ray[j];int k=index(q);
        bool ground=best>=12 && std::abs(q[2]-plane[0]*(q[0]-x)-plane[1]*(q[1]-y)-plane[2])<.025+.01*d;
        if(k>=0 && !ground)updates[k]=3;
      }
      for(auto [k,delta]:updates)voxels[k]=static_cast<int8_t>(std::clamp(int(voxels[k])+delta,-12,24));
    }
    trajectory<<t<<','<<x<<','<<y<<','<<yaw<<','<<valid<<'\n';
    const auto now=std::chrono::steady_clock::now();
    if(frame%3==0 && now-last_snapshot>=std::chrono::milliseconds(250)) {
      snapshot(t,valid,count);last_snapshot=std::chrono::steady_clock::now();
    }
  }
  snapshot(std::max(0.0,last_t),0,0);
}
