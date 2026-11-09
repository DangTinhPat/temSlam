#include <cassert>
#include <cstdio>
#include "rb1/control/balance/AddWifiBalanceProfile.hpp"
using namespace rb1::control;
void rollingHandoffSeedsPreview(bool enabled) {
  for (float speed : {-.4F, .4F}) {
    auto cfg=defaultRobotConfig();
    rb1::control::applyAddWifiBalanceProfile(cfg, enabled);
    cfg.standup.enabled=true;
    cfg.standup.fold_travel_rad=0;
    cfg.standup.kneel_hold_s=cfg.standup.pitch_settle_s=0;
    cfg.standup.catch_settle_s=0;
    // Prescribed transition contract; this is not physical capture evidence.
    cfg.standup.catch_lean_tol_rad=cfg.standup.catch_capture_tol_rad=10;
    cfg.tier.preview=true;
    RobotController c(cfg);
    RobotController::Input in{};
    in.legs_ready=in.legs_measured=in.sensors_ok=true;
    in.accel_z_m_s2=9.80665F;
    const auto pose=LegHeight::forKnee(LegHeight::kKneeMinRad);
    in.hip_rad={pose.hip_offset_rad,pose.hip_offset_rad};
    in.knee_rad={pose.knee_offset_rad,pose.knee_offset_rad};
    in.leg_chain_rad=pose.hip_offset_rad+pose.knee_offset_rad;
    in.wheel_rad_s={speed/cfg.wheel_radius_m,speed/cfg.wheel_radius_m};
    RobotController::Command cmd{};cmd.standup=true;
    RobotController::Output out{};
    for(int i=0;i<10;++i) {
      in.now_us+=2000;
      out=c.update(in,cmd);cmd.standup=false;
      if(out.state==RobotController::State::Balancing) break;
    }
    assert(out.state==RobotController::State::Balancing);
    // The hand-over keeps the speed the wheels already roll at (the controller's plan_v_ is seeded with it).
    assert(c.groundVelocity()*speed>0);
    cmd.disable=true; in.now_us+=2000;
    out=c.update(in,cmd);
    assert(out.state==RobotController::State::Idle);
    assert(out.wheel_cmd_rad_s[0]==0 && out.wheel_cmd_rad_s[1]==0);
  }
}
int main() {
  rollingHandoffSeedsPreview(false);
  rollingHandoffSeedsPreview(true);
  puts("rolling handoff preserves speed for both boot profiles");
}
