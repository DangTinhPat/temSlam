#include <cassert>
#include <cmath>
#include <cstdio>
#include "rb1/control/RobotController.hpp"
using rb1::control::RobotController;
int main() {
  auto cfg=rb1::control::defaultRobotConfig();
  cfg.split.enabled=true;cfg.engage.immediate=true;
  RobotController ctrl(cfg);
  RobotController::Input in{};RobotController::Command cmd{};
  in.legs_ready=in.legs_measured=true;
  cmd.start=true;(void)ctrl.update(in,cmd);cmd.start=false;
  in.now_us=1'300'000;(void)ctrl.update(in,cmd);
  in.now_us+=2000;(void)ctrl.update(in,cmd);
  float previous=ctrl.splitTelemetry().hip_a;
  for(int k=0;k<3000;++k) {
    in.now_us+=2000;
    const auto p=ctrl.legPose();
    in.hip_rad={ctrl.hipCommand(),ctrl.hipCommand()};
    in.knee_rad={ctrl.kneeCommand(),ctrl.kneeCommand()};
    in.leg_chain_rad=ctrl.hipCommand()+ctrl.kneeCommand();
    cmd.height_m=.110F;
    const auto out=ctrl.update(in,cmd);
    const auto s=ctrl.splitTelemetry();
    assert(std::isfinite(out.leg_hip_offset_rad));
    assert(std::abs(s.hip_a-previous)<=cfg.split.posture_rate_rad_s*in.dt_s+1e-6F);
    assert(std::abs(s.hip_a)<=cfg.tier.level_max_rad+1e-6F);
    assert(s.hip_b==0 && s.alpha_b==0); // Stage A cannot sneak fast reflex in.
    previous=s.hip_a;
    (void)p;
  }
  // Freshness failure keeps wheel shutdown authoritative and B inactive.
  in.now_us+=2000;in.sensors_ok=false;
  const auto fault=ctrl.update(in,cmd);
  assert(fault.state==RobotController::State::Fault);
  assert(!fault.wheels_enabled);
  assert(ctrl.splitTelemetry().frozen);
  cmd.disable=true;in.now_us+=2000;
  const auto stopped=ctrl.update(in,cmd);
  assert(stopped.state==RobotController::State::Idle);
  const auto s=ctrl.splitTelemetry();
  assert(s.hip_a==0 && s.hip_b==0 && s.alpha_b==0 && s.preview==0);
  std::puts("split stage A: bounded slew, bounded posture, no B, fault and disable passed");
}
