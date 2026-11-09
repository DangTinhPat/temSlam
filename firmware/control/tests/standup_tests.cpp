#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "rb1/control/legs/StandUp.hpp"
#include "rb1/control/RobotController.hpp"
#include "rb1/core/BodyClearance.hpp"

using namespace rb1::control;

namespace {
StandUp::Input sample(float dt = .01F) {
  StandUp::Input in;
  in.dt_s = dt;
  return in;
}

StandUp catching() {
  StandUp::Config cfg;
  cfg.enabled = true;
  cfg.fold_travel_rad = 0;
  cfg.kneel_hold_s = .02F;
  cfg.pitch_settle_s = .02F;
  StandUp su(cfg);
  su.begin(0, 0);
  for (int i = 0; i < 20 && su.phase() != StandUp::Phase::Catching; ++i)
    (void)su.update(sample());
  assert(su.phase() == StandUp::Phase::Catching);
  return su;
}

void headDropThenTorqueRelease() {
  StandUp::Config cfg;
  cfg.enabled = true;
  cfg.fold_travel_rad = .1F;
  cfg.fold_rate_rad_s = 1;
  cfg.slump_travel_rad = .2F;
  cfg.slump_rate_rad_s = .5F;
  StandUp su(cfg);
  su.begin(.1F, 0);
  auto in = sample();
  auto out = su.update(in);
  for (int i=0; i<200 && su.phase()==StandUp::Phase::Folding; ++i) out = su.update(in);
  assert(su.phase()==StandUp::Phase::Kneeling);
  const float folded = out.hip_target_rad;
  assert(!out.released && out.wheels_on);
  float driven = folded;
  for (int i=0; i<60 && !out.released; ++i) {    // 0.2 rad at 0.5 rad/s needs 0.4 s
    out = su.update(in);
    if (!out.released) driven = out.hip_target_rad;
  }
  assert(std::abs((folded - driven) - .2F) < 1e-3F);   // fold direction is negative
  assert(su.phase()==StandUp::Phase::Kneeling && out.released && !out.wheels_on);
  // Released: the hip command follows the measured hip, so the next engage starts where the body rests.
  in.hip_measured_rad = -.37F;
  out = su.update(in);
  assert(out.released && out.hip_target_rad == -.37F);
  for (int i=0; i<200 && su.phase()==StandUp::Phase::Kneeling; ++i) out = su.update(in);
  assert(su.phase()==StandUp::Phase::Catching && !out.released && out.wheels_on);
}

void comProbeMeasuresAndReturns() {
  // Kneel hold, then the probe: lift the front by probe_lift (sample 1) and probe_lift2 (sample 2), lower back,
  // go limp for probe_relax, then catch. The plant here: the pitch comes up one-for-one with the hip, and the hip
  // torque is a body moment A*cos(p) + B*sin(p).
  StandUp::Config cfg;
  cfg.enabled = true;
  cfg.fold_travel_rad = .1F;
  cfg.fold_rate_rad_s = 1;
  cfg.slump_travel_rad = 0;
  cfg.kneel_hold_s = .1F;
  cfg.probe_rate_rad_s = .5F;
  cfg.probe_lift_rad = .03F;
  cfg.probe_lift2_rad = .08F;
  cfg.probe_settle_s = cfg.probe_average_s = cfg.probe_relax_s = .05F;
  const auto moment = [](float p) { return .2F * std::cos(p) + .1F * std::sin(p); };
  for (const bool reported : {true, false}) {
    StandUp su(cfg);
    su.begin(.1F, 0);
    auto in = sample();
    in.hip_torque_valid = reported;
    auto out = su.update(in);
    float hip0 = 0, pitch0 = .2F;
    bool probing = false, limp_seen = false;
    for (int i = 0; i < 2000 && su.phase() != StandUp::Phase::Catching; ++i) {
      in.hip_measured_rad = out.hip_target_rad;
      if (!probing && out.probe_stage == 1) { probing = true; hip0 = out.hip_target_rad; }
      in.pitch_rad = probing ? pitch0 - std::max(0.0F, out.hip_target_rad - hip0) : pitch0;
      in.hip_torque_nm = moment(in.pitch_rad);
      out = su.update(in);
      if (out.probe_stage >= 1 && out.probe_stage <= 3) assert(!out.released);
      if (out.probe_stage == 4) { assert(out.released); limp_seen = true; }
    }
    assert(su.phase() == StandUp::Phase::Catching);
    if (!reported) {
      assert(!probing && out.probe_stage == 0 && !out.probe_valid);
      continue;
    }
    assert(limp_seen && out.probe_valid && out.probe_two_point);
    assert(std::abs(out.probe_torque_nm - moment(out.probe_pitch_rad)) < 1e-3F);
    assert(std::abs(out.probe_torque2_nm - moment(out.probe_pitch2_rad)) < 1e-3F);
    assert(pitch0 - out.probe_pitch_rad >= .03F - 1e-3F && pitch0 - out.probe_pitch2_rad >= .08F - 1e-3F);
    assert(std::abs(out.hip_target_rad - hip0) < 1e-3F);   // the catch starts from the kneel it was tuned for
  }
}

void singleCommandAndDiagnosticHolds() {
  StandUp::Config cfg;
  cfg.enabled = true;
  cfg.fold_travel_rad = .1F;
  cfg.fold_rate_rad_s = 1;
  cfg.pitch_settle_s = .02F;
  cfg.slump_travel_rad = 0;
  cfg.kneel_limp = false;
  StandUp su(cfg);
  su.begin(.1F, 0);
  auto in = sample();
  auto out = su.update(in);
  assert(out.phase == StandUp::Phase::Folding && out.hip_target_rad < .1F);
  for (int i=0; i<20 && su.phase()==StandUp::Phase::Folding; ++i) out=su.update(in);
  assert(su.phase()==StandUp::Phase::Kneeling);
  const float held=out.hip_target_rad;
  for(int i=0;i<99;++i) {
    out=su.update(in);
    assert(out.phase==StandUp::Phase::Kneeling);
    assert(out.hip_target_rad==held && out.wheel_rad_s==0);
  }
  // Float accumulation can put the transition one tick beyond exactly 1 s.
  for(int i=0;i<2 && su.phase()==StandUp::Phase::Kneeling;++i) out=su.update(in);
  for(int i=0;i<10 && su.phase()!=StandUp::Phase::Catching;++i) out=su.update(in);
  assert(su.phase()==StandUp::Phase::Catching);

  cfg.fold_travel_rad=0; cfg.hold_at_kneel=true;
  StandUp manual(cfg);manual.begin(0,0);
  for(int i=0;i<200;++i) out=manual.update(in);
  assert(out.phase==StandUp::Phase::Kneeling);
  assert(manual.advanceFromKneeling());
  for(int i=0;i<3;++i)out=manual.update(in);
  assert(out.phase==StandUp::Phase::Catching);
  assert(!manual.advanceFromKneeling());
}

void feedbackCanReverseAndIsBounded() {
  auto su=catching();auto in=sample();in.capture_lean_rad=-.30F;
  float previous=0,previous_rate=0;
  for(int i=0;i<30;++i) {
    in.hip_measured_rad=previous;
    const auto out=su.update(in);
    assert(out.phase==StandUp::Phase::Catching);
    assert(std::abs(out.hip_rate_rad_s)<=su.config().catch_hip_rate_max_rad_s+1e-5F);
    assert(std::abs(out.hip_rate_rad_s-previous_rate)<=su.config().catch_hip_accel_rad_s2*in.dt_s+1e-4F);
    assert(std::abs(out.wheel_rad_s)<=su.config().catch_wheel_max_rad_s);
    assert(out.hip_target_rad>=previous);
    previous=out.hip_target_rad;previous_rate=out.hip_rate_rad_s;
  }
  in.capture_lean_rad=.30F;
  bool reversed=false;
  for(int i=0;i<30;++i) {
    in.hip_measured_rad=previous;
    const auto out=su.update(in);
    if(out.hip_target_rad<previous)reversed=true;
    previous=out.hip_target_rad;
  }
  assert(reversed);
}

void crossingZeroIsNotCapture() {
  auto su=catching();auto in=sample();
  in.capture_rate_rad_s=1.0F; // angle is zero but the divergent mode is 0.10 rad
  for(int i=0;i<20;++i) {
    const auto out=su.update(in);
    assert(out.phase==StandUp::Phase::Catching);
    in.hip_measured_rad=out.hip_target_rad;
  }
  in.capture_rate_rad_s=0;
  for(int i=0;i<20 && su.phase()!=StandUp::Phase::Support;++i) {
    const auto out=su.update(in);in.hip_measured_rad=out.hip_target_rad;
  }
  assert(su.phase()==StandUp::Phase::Support);
  const auto held=su.update(in);
  in.pitch_rad=.1F;
  const auto again=su.update(in);
  assert(again.phase==StandUp::Phase::Support && again.hip_target_rad==held.hip_target_rad);
}

void carrierAndSafety() {
  auto su=catching();auto in=sample();in.carrier_rate_rad_s=1.25F;
  auto out=su.update(in);
  assert(std::abs(out.wheel_rad_s+1.25F)<1e-5F);
  // Faster-than-command rolling must not invert the anti-windup clamp bounds.
  su=catching();in=sample();in.ground_speed_m_s=2;
  out=su.update(in);
  assert(std::isfinite(out.wheel_rad_s) && std::abs(out.wheel_rad_s)<=10);
  su=catching();in=sample();in.sensors_ok=false;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed && !out.wheels_on && out.wheel_rad_s==0);
  su=catching();in=sample();in.capture_rate_rad_s=std::numeric_limits<float>::quiet_NaN();out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed && !out.wheels_on);
  su=catching();in=sample();in.hip_jacobian=0;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed);
  su=catching();su.config().timeout_s=.015F;in=sample();in.capture_lean_rad=.3F;
  for(int i=0;i<3;++i)out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed);
  su=catching();su.config().catch_wheel_max_rad_s=-1;out=su.update(sample());
  assert(out.phase==StandUp::Phase::Failed);
}

void controllerDisableWins() {
  auto cfg=defaultRobotConfig();assert(!cfg.standup.enabled);
  cfg.standup.enabled=true;
  RobotController c(cfg);RobotController::Input in{};
  in.legs_measured=in.legs_ready=in.sensors_ok=true;
  in.hip_rad={1.3F,1.3F};in.leg_chain_rad=1.3F;
  RobotController::Command cmd{};cmd.standup=true;
  auto out=c.update(in,cmd);assert(out.state==RobotController::State::StandUp);
  cmd.disable=true;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==RobotController::State::Idle && !out.legs_locked && !out.wheels_enabled);
  cmd={};in.now_us+=2000;out=c.update(in,cmd);assert(out.state==RobotController::State::Idle);
  cmd.standup=true;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==RobotController::State::StandUp);
  cmd={};in.sensors_ok=false;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==RobotController::State::Fault && !out.wheels_enabled);
  in.sensors_ok=true;in.now_us+=2000;out=c.update(in,cmd);
  assert(out.state==RobotController::State::Fault); // explicit restart required
}

void liftOverlapAndClearance() {
  // The separate pitch-to-level phase is removed: Kneeling hands over to Catching, blending the catch in.
  StandUp::Config cfg;
  cfg.enabled=true; cfg.overlap_after_lift=true;
  cfg.fold_travel_rad=0; cfg.kneel_hold_s=0;
  StandUp su(cfg);su.begin(0,0);
  auto in=sample();in.pitch_rad=.4F;in.capture_lean_rad=-.3F;
  StandUp::Output out{};
  for(int i=0;i<4 && su.phase()!=StandUp::Phase::Catching;++i) out=su.update(in);
  assert(out.phase==StandUp::Phase::Catching);
  assert(out.catch_blend==0 && out.wheel_rad_s==0);
  float last_rate=out.hip_rate_rad_s, last_blend=0;
  for(int i=0;i<40;++i) {
    in.hip_measured_rad=out.hip_target_rad;
    out=su.update(in);
    assert(out.phase==StandUp::Phase::Catching);
    assert(out.catch_blend>=last_blend && out.catch_blend<=1);
    assert(std::abs(out.hip_rate_rad_s-last_rate)<=cfg.catch_hip_accel_rad_s2*in.dt_s+1e-4F);
    assert(std::abs(out.wheel_rad_s)<=cfg.catch_wheel_max_rad_s);
    last_rate=out.hip_rate_rad_s;last_blend=out.catch_blend;
  }
  assert(out.catch_blend==1);

  // Catching is still a moving phase: the stand-up times out instead of waiting forever.
  su=StandUp(cfg);su.begin(0,0);in=sample();
  for(int i=0;i<5;++i)out=su.update(in);
  assert(out.phase==StandUp::Phase::Catching);
  su.config().timeout_s=.001F;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed);

  const float flat=rb1::core::bodyClearance(0,0,{0,0},{0,0},.035F);
  // RB v1.3, independent of the code: CAD-zero axle z=-42.2856 mm below the hip (knee -0.2569 mm,
  // wheel -42.0287 mm), tyre radius 35 mm, shell bottom (Body 2) -50.0 mm - 0.1 mm padding:
  // 0.035 + 0.0422856 - 0.0501 = 0.0271856 m.
  assert(std::abs(flat-.0271856F)<3e-6F);
  const float l=rb1::core::bodyClearance(.2F,.04F,{0,0},{0,0},.04F);
  const float r=rb1::core::bodyClearance(.2F,-.04F,{0,0},{0,0},.04F);
  assert(std::isfinite(l) && std::abs(l-r)<1e-5F);
  assert(std::isnan(rb1::core::bodyClearance(0,0,{0,0},{0,0},-1)));
}

void peakWindowExpires() {
  auto su=catching();su.config().catch_wheel_peak_s=.03F;
  auto in=sample();in.capture_lean_rad=-.3F;
  StandUp::Output out{};
  for(int i=0;i<50;++i) {
    in.hip_measured_rad=out.hip_target_rad;
    out=su.update(in);
    if(i==0) assert(out.wheel_peak_requested);
    if(i>=3) assert(!out.wheel_peak_requested);
  }
  su.reset();out=su.update(in);
  assert(!out.wheel_peak_requested && !out.wheels_on);
  su=catching();su.config().catch_wheel_peak_s=1.01F;out=su.update(in);
  assert(out.phase==StandUp::Phase::Failed && !out.wheel_peak_requested);
}

// Prescribed encoder/gyro noise, not a physical stand-up simulation.
void stoppedHandoffPreparesActualCommand() {
  for (bool enabled : {false, true}) {
    auto su = catching();
    auto& cfg = su.config();
    cfg.guarded_handoff = true;
    cfg.handoff_prepare = enabled;
    cfg.catch_measured_braking = true;
    cfg.catch_wheel_prediction = cfg.catch_capture_velocity = true;
    cfg.catch_hip_gain = cfg.catch_hip_damping_s = 0;
    cfg.catch_accel_gain = 0;
    cfg.catch_accel_damping = 25;
    cfg.catch_accel_max = 5;
    cfg.catch_wheel_max_rad_s = 20;
    cfg.catch_wheel_lead_s = .075F;
    cfg.catch_blend_s = .001F;
    auto in = sample(.002F);
    in.body_clearance_valid = true;
    in.body_clearance_m = .1F;
    StandUp::Output out{};
    bool saw_lead_error = false;
    float previous_weight = 0;
    for (int i=0; i<500 && su.phase()!=StandUp::Phase::Support; ++i) {
      in.capture_rate_rad_s = (i/4)%2 ? -.2F : .2F;
      out = su.update(in);
      in.hip_measured_rad = out.hip_target_rad;
      saw_lead_error |= out.wheel_tracking_error_rad_s > 4;
      assert(std::abs(out.handoff_prepare_weight-previous_weight)<=.01001F);
      if (out.handoff_prepare_weight < .999F && enabled)
        assert(out.phase != StandUp::Phase::Support);
      previous_weight = out.handoff_prepare_weight;
    }
    assert(saw_lead_error);  // negative control sees the original gate blocker
    assert((su.phase()==StandUp::Phase::Support)==enabled);
    if (enabled) {
      assert(out.settle_s >= .20F);
      assert(out.wheel_tracking_error_rad_s <= 4);
      assert(out.wheel_lead_s < .0001F);
    } else assert(out.handoff_prepare_weight==0);
  }

  // Preparation must not veto the calm timer that starts the knee rise.
  // It must wait for completion, and restore normal lead after lost calm.
  {
    auto su=catching();auto& cfg=su.config();
    cfg.handoff_prepare=cfg.guarded_handoff=cfg.catch_measured_braking=true;
    cfg.catch_wheel_prediction=cfg.catch_capture_velocity=true;
    cfg.catch_rise=true;cfg.catch_rise_knee_rad=.02F;
    cfg.catch_rise_start_dwell_s=.02F;
    auto in=sample(.002F);in.body_clearance_valid=true;in.body_clearance_m=.1F;
    StandUp::Output out{};
    for(int i=0;i<500;++i) {
      out=su.update(in);
      in.hip_measured_rad=out.hip_target_rad;in.knee_measured_rad=out.knee_target_rad;
      if(out.knee_target_rad<.019F)assert(out.handoff_prepare_weight==0);
      if(out.handoff_prepare_weight>=.4F)break;
    }
    assert(out.knee_target_rad>=.019F && out.handoff_prepare_weight>=.4F);
    in.pitch_rate_rad_s=.5F;
    const float previous_weight=out.handoff_prepare_weight;
    out=su.update(in);
    assert(out.handoff_prepare_weight<previous_weight && !out.handoff_ready);
    for(int i=0;i<100;++i) {
      out=su.update(in);in.hip_measured_rad=out.hip_target_rad;
      assert(out.phase!=StandUp::Phase::Support);
    }
    assert(out.handoff_prepare_weight==0 && out.wheel_lead_s>=.074F);
  }

  // Each bad physical signal still prevents handoff. A prepared command is
  // never a substitute for measured tracking, valid clearance or safe rates.
  for (int bad=0; bad<6; ++bad) {
    auto su=catching();auto& cfg=su.config();
    cfg.handoff_prepare=cfg.guarded_handoff=cfg.catch_measured_braking=true;
    cfg.catch_wheel_prediction=cfg.catch_capture_velocity=true;
    cfg.catch_hip_gain=cfg.catch_hip_damping_s=0;
    cfg.catch_accel_gain=cfg.catch_accel_damping=0;
    auto in=sample(.002F);
    in.body_clearance_valid=true;in.body_clearance_m=.1F;
    if(bad==0)in.wheel_measured_rad_s={8,-8};
    if(bad==1)in.body_clearance_valid=false;
    if(bad==2)in.pitch_rate_rad_s=.5F;
    if(bad==3)in.hip_measured_rate_rad_s=.6F;
    if(bad==4)in.ground_speed_m_s=.2F;
    if(bad==5)in.capture_lean_rad=.3F;
    for(int i=0;i<500;++i) {
      const auto out=su.update(in);in.hip_measured_rad=out.hip_target_rad;
      assert(out.phase!=StandUp::Phase::Support);
    }
    su.reset();su.begin(0,0);
    assert(su.update(sample()).handoff_prepare_weight==0);
  }
}

void rollingHandoffSeedsPreview() {
  for (float speed : {-.4F, .4F}) {
    auto cfg=defaultRobotConfig();
    cfg.standup.enabled=true;
    cfg.standup.fold_travel_rad=0;
    cfg.standup.kneel_hold_s=cfg.standup.pitch_settle_s=0;
    cfg.standup.catch_settle_s=0;
    // Synthetic transition contract; physical capture is covered in MuJoCo.
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
  }
}

}

int main() {
  headDropThenTorqueRelease();comProbeMeasuresAndReturns();singleCommandAndDiagnosticHolds();feedbackCanReverseAndIsBounded();
  crossingZeroIsNotCapture();carrierAndSafety();controllerDisableWins();
  liftOverlapAndClearance();
  peakWindowExpires();
  rollingHandoffSeedsPreview();
  stoppedHandoffPreparesActualCommand();
  std::puts("stand-up: autonomous sequencing, bidirectional capture, bounded commands and stop priority passed");
}
