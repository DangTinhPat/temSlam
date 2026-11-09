#include <mujoco/mujoco.h>

#include "rb1/core/GeneratedGeometry.hpp"
#include "rb1/core/ImuStateEstimator.hpp"
#include "rb1/core/WholeBodyModel.hpp"
#include "rb1/control/board/BoardConfig.hpp"
#include "rb1/control/legs/LegHeight.hpp"
#include "rb1/control/RobotController.hpp"
#include "SimActuatorBus.hpp"
#include "Mpu6500Model.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace {
using rb1::control::RobotController;
namespace geometry = rb1::core::geometry;
namespace motor_transport = rb1::hal::motor_transport;

struct Options {
  std::string model;
  double physics_hz = 3000.0;
  double sensor_hz = 1.0e6 / rb1::control::board::kControlPeriodUs;  // the firmware control rate (--sensor-rate)
  double state_hz = 50.0;  // diagnostics only; VIEW remains 50 Hz
  std::uint16_t command_port = 0;
  double run_for_s = 0.0;
  double wheel_delay_s = 0.050;
  double wheel_lag_s = 0.075;
  double sensor_noise = 0.0;  // scale of documented synthetic sensor standard deviations
  unsigned noise_seed = 1;
  bool allow_model_mismatch = false;  // explicit plant-only robustness experiments
  bool realtime = false;
  bool lqr_prediction = true;
  bool gyro_hip_lead = true;
  std::string control_spec;  // explicit simulation-only A/B tuning overrides
  // Speed ceilings [m/s], 0 = the controller defaults: DRIVE clamp, planned speed cap, LQR ground speed cap.
  double drive_max = 4.1888, plan_vmax = 0.0, ground_vmax = 0.0, track_margin = -1.0;
  std::string start_key = "spawn_extended";   // --start-key NAME: the MJCF keyframe the robot starts in ("home" = the lying pose)
  double wheel_torque = 0.13;   // regular/rated cap; --wheel-torque overrides every phase [N.m]
  bool wheel_torque_override = false;
  bool wheel_torque_whatif = false;  // --wheel-torque-whatif: allow > 0.20 N.m for hypothetical trials
  // --leg-actuator pos|delta_i: how the four leg joints are driven. pos (default) models the GDS68
  // position loop the firmware commands with Set_Input_Pos (no Kt, no Iq feedback, no gravity model in the
  // controller); delta_i models the older current-compensated torque path.
  bool leg_pos = true;
  bool plain_balance = false;
  // --standup-profile fw|sim: fw = the StandUp::Config defaults the firmware runs (enabled by SET standup 1); sim = the
  // faster, overlapping recovery tuned in this simulator.
  bool standup_sim_profile = true;
  // --standup-set key=value[,key=value]: override StandUp::Config fields for A/B runs (lift, lift_hip_min, ...).
  std::vector<std::pair<std::string, double>> standup_overrides;
  double stand_height_mm = 0.0;   // --stand-height MM: height after the stand-up hand-over (0 = full supported height, as the firmware)
  std::vector<std::pair<std::string, double>> balance_overrides;   // --balance-set key=value[,key=value]: LQR / tier A/B runs
  // --leg-delay SEC: dead time between the controller's leg targets and the driver position loop (the real GDS68
  // path measures ~22 ms). --imu-hw 1: the MPU6500 as configured on the robot (accelerometer +-2 g per axis,
  // ~44 Hz digital low-pass); off = the ideal sensor of the other tests.
  double leg_delay_s = 0.0;
  // --leg-model KP_S,KV,ARMATURE,CURRENT_LAG: the position loop of the sim (default 15/s, 0.8 N.m.s/rad, 1e-4 kg.m^2,
  // 5 ms = provisional). With Kv/J ~ 400 1/s the loop is only stable if the current lag is <~ 10 ms; a 20 ms lag made the
  // light legs chatter (hip +-0.1 rad, knee +-0.15 rad in the air during the fold). Stand-up logs fit dead time ~25 ms + lag ~30 ms on the hip.
  double leg_kp_s = 15.0, leg_kv = 0.8, leg_armature = 1.0e-4, leg_current_lag_s = 0.005;
  bool imu_hw = false;
  double imu_range_g = 2.0;
  // --imu-model 1: the MPU-6500 imperfections of model/noise.yaml (Mpu6500Model) instead of the
  // ideal IMU, and the firmware's estimator switching (base config, gyro-dominant in Balancing / stand-up). A unit
  // (scale, cross-axis, turn-on bias) is drawn from --imu-unit (default: --noise-seed); --imu-noise scales the white
  // noise and bias walk; --imu-vib A,G adds motor vibration [mg, dps per rad/s of mean wheel speed] (assumed).
  bool imu_model = false;
  long imu_unit_seed = -1;
  double imu_noise_scale = 1.0;
  double imu_vib_accel = 0.0, imu_vib_gyro = 0.0;
  // Commands run at exact simulated times (--at SEC:COMMAND); stdin timing depends on
  // process scheduling, so headless tests use this to be reproducible.
  std::vector<std::pair<double, std::string>> schedule;
};

double number(const char* text, const char* option) {
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(value))
    throw std::invalid_argument(std::string("invalid ") + option);
  return value;
}

Options parseOptions(int argc, char** argv) {
  Options out;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + arg);
      return argv[++i];
    };
    if (arg == "--model") out.model = next();
    else if (arg == "--rate") out.physics_hz = number(next(), "--rate");
    else if (arg == "--sensor-rate") out.sensor_hz = number(next(), "--sensor-rate");
    else if (arg == "--state-hz") out.state_hz = number(next(), "--state-hz");
    else if (arg == "--wheel-delay") out.wheel_delay_s = number(next(), "--wheel-delay");
    else if (arg == "--wheel-lag") out.wheel_lag_s = number(next(), "--wheel-lag");
    else if (arg == "--sensor-noise") out.sensor_noise = number(next(), "--sensor-noise");
    else if (arg == "--allow-model-mismatch") out.allow_model_mismatch = true;
    else if (arg == "--noise-seed") {
      const double value = number(next(), "--noise-seed");
      if (value < 0 || value > 4294967295.0 || value != std::floor(value))
        throw std::invalid_argument("invalid --noise-seed");
      out.noise_seed = static_cast<unsigned>(value);
    }
    else if (arg == "--command-port") {
      const auto port = std::stoul(next());
      if (port > 65535U) throw std::invalid_argument("invalid --command-port");
      out.command_port = static_cast<std::uint16_t>(port);
    } else if (arg == "--run-for") out.run_for_s = number(next(), "--run-for");
    else if (arg == "--realtime") out.realtime = true;
    else if (arg == "--no-lqr-prediction") out.lqr_prediction = false;
    else if (arg == "--virtual-hip-lead") out.gyro_hip_lead = false;
    else if (arg == "--control") out.control_spec = next();
    else if (arg == "--drive-max") out.drive_max = number(next(), "--drive-max");
    else if (arg == "--plan-vmax") out.plan_vmax = number(next(), "--plan-vmax");
    else if (arg == "--ground-vmax") out.ground_vmax = number(next(), "--ground-vmax");
    else if (arg == "--track-margin") out.track_margin = number(next(), "--track-margin");
    else if (arg == "--start-key") out.start_key = next();
    else if (arg == "--wheel-torque") {
      out.wheel_torque = number(next(), "--wheel-torque");
      out.wheel_torque_override = true;
    }
    else if (arg == "--wheel-torque-whatif") out.wheel_torque_whatif = true;
    else if (arg == "--leg-actuator") {
      const std::string kind = next();
      if (kind != "pos" && kind != "delta_i") throw std::invalid_argument("--leg-actuator must be pos or delta_i");
      out.leg_pos = kind == "pos";
    }
    else if (arg == "--standup-profile") out.standup_sim_profile = std::string(next()) != "fw";
    else if (arg == "--standup-set") {
      std::stringstream list(next());
      std::string item;
      while (std::getline(list, item, ',')) {
        const auto eq = item.find('=');
        if (eq == std::string::npos) throw std::invalid_argument("--standup-set expects key=value");
        out.standup_overrides.emplace_back(item.substr(0, eq), number(item.substr(eq + 1).c_str(), "--standup-set"));
      }
    }
    else if (arg == "--stand-height") out.stand_height_mm = number(next(), "--stand-height");
    else if (arg == "--balance-set") {
      std::stringstream list(next());
      std::string item;
      while (std::getline(list, item, ',')) {
        const auto eq = item.find('=');
        if (eq == std::string::npos) throw std::invalid_argument("--balance-set expects key=value");
        out.balance_overrides.emplace_back(item.substr(0, eq), number(item.substr(eq + 1).c_str(), "--balance-set"));
      }
    }
    else if (arg == "--leg-model") {
      std::stringstream list(next());
      std::string item;
      std::vector<double> v;
      while (std::getline(list, item, ',')) v.push_back(number(item.c_str(), "--leg-model"));
      if (v.size() != 4) throw std::invalid_argument("--leg-model expects KP_S,KV,ARMATURE,CURRENT_LAG");
      out.leg_kp_s = v[0]; out.leg_kv = v[1]; out.leg_armature = v[2]; out.leg_current_lag_s = v[3];
    }
    else if (arg == "--leg-delay") out.leg_delay_s = number(next(), "--leg-delay");
    else if (arg == "--imu-hw") out.imu_hw = number(next(), "--imu-hw") != 0.0;
    else if (arg == "--imu-range") out.imu_range_g = number(next(), "--imu-range");
    else if (arg == "--imu-model") out.imu_model = number(next(), "--imu-model") != 0.0;
    else if (arg == "--imu-unit") out.imu_unit_seed = static_cast<long>(number(next(), "--imu-unit"));
    else if (arg == "--imu-noise") out.imu_noise_scale = number(next(), "--imu-noise");
    else if (arg == "--imu-vib") {
      const std::string value = next();
      const auto comma = value.find(',');
      out.imu_vib_accel = number(value.substr(0, comma).c_str(), "--imu-vib");
      out.imu_vib_gyro = comma == std::string::npos ? 0.0 : number(value.substr(comma + 1).c_str(), "--imu-vib");
    }
    else if (arg == "--plain-balance") out.plain_balance = number(next(), "--plain-balance") != 0.0;
    else if (arg == "--at") {
      const std::string value = next();
      const auto colon = value.find(':');
      if (colon == std::string::npos) throw std::invalid_argument("--at expects SEC:COMMAND");
      out.schedule.emplace_back(number(value.substr(0, colon).c_str(), "--at"),
                                value.substr(colon + 1));
    }
    else if (arg == "--help") {
      std::cout << "rb1_sim --model FILE [--rate HZ] [--sensor-rate HZ] "
                   "[--command-port PORT] [--wheel-delay SEC] [--wheel-lag SEC] "
                   "[--wheel-torque 0..0.20] [--leg-actuator pos|delta_i] "
                   "[--sensor-noise SCALE] [--noise-seed UINT] [--allow-model-mismatch] "
                   "[--state-hz HZ] [--control key=val,...] "
                   "[--no-lqr-prediction] [--virtual-hip-lead] [--at SEC:CMD] "
                   "[--realtime] [--run-for SEC]\n";
      std::exit(0);
    } else throw std::invalid_argument("unknown option: " + arg);
  }
  if (out.model.empty()) throw std::invalid_argument("--model is required");
  if (out.sensor_noise < 0 || out.sensor_noise > 10)
    throw std::invalid_argument("invalid --sensor-noise");
  if (out.physics_hz < 1000.0 || out.physics_hz > 4000.0 ||
      out.sensor_hz <= 0.0 || out.sensor_hz > out.physics_hz || out.run_for_s < 0.0 ||
      out.state_hz <= 0.0 || out.state_hz > out.sensor_hz ||
      out.wheel_delay_s < 0 || out.wheel_delay_s > .1 || out.wheel_lag_s < 0 || out.wheel_lag_s > .12 ||
      out.wheel_torque <= 0.0 || out.wheel_torque > (out.wheel_torque_whatif ? 2.0 : 0.20))
    throw std::invalid_argument("invalid simulation rate or duration");
  return out;
}

class RbV3Runner {
 public:
  explicit RbV3Runner(Options options)
      : options_(std::move(options)), controller_(simulationControllerConfig(options_)) {
    noise_rng_.seed(options_.noise_seed);
    if (options_.imu_model) {
      rb1::sim::Mpu6500Model::Params p{};
      p.noise_scale = static_cast<float>(options_.imu_noise_scale);
      p.vib_accel_mg_per_rad_s = static_cast<float>(options_.imu_vib_accel);
      p.vib_gyro_dps_per_rad_s = static_cast<float>(options_.imu_vib_gyro);
      const unsigned unit = options_.imu_unit_seed >= 0 ? static_cast<unsigned>(options_.imu_unit_seed)
                                                        : options_.noise_seed;
      mpu_.configure(p, 1000U + unit, 2000U + options_.noise_seed);
    }
    char error[1024]{};
    model_ = mj_loadXML(options_.model.c_str(), nullptr, error, sizeof(error));
    if (!model_) throw std::runtime_error(std::string("MuJoCo load failed: ") + error);
    model_->opt.timestep = 1.0 / options_.physics_hz;
    data_ = mj_makeData(model_);
    if (!data_) throw std::runtime_error("MuJoCo data allocation failed");
    // Base cap 0.13 N.m; recovery can request a bounded 0.20 N.m window.
    // --wheel-torque explicitly overrides both base and recovery phases.
    for (std::size_t side = 0; side < wheels_.size(); ++side) {
      rb1::sim::SimBl4012Wheel::Config wheel_config{static_cast<std::uint8_t>(side + 1),
                                                         static_cast<float>(options_.wheel_torque)};
      // What-if: no torque fall-off above the rated 400 rpm either (flat torque up to the 1000 rpm cap).
      if (options_.wheel_torque_whatif) wheel_config.rated_speed_rad_s = wheel_config.max_speed_rad_s;
      wheels_[side] = rb1::sim::SimBl4012Wheel(wheel_config);
    }
    bindModel();
    for (const std::size_t j : {0U, 1U, 3U, 4U}) model_->dof_armature[joints_[j].qvel] = options_.leg_armature;
    wheel_delay_ticks_ = static_cast<std::size_t>(std::lround(options_.wheel_delay_s * options_.sensor_hz));
    leg_delay_ticks_ = std::min<std::size_t>(leg_delay_buf_.size(),
        static_cast<std::size_t>(std::lround(options_.leg_delay_s * options_.sensor_hz)));
    configureImu();
    reset();
    setNonBlockingStdin();
    openCommandSocket();
    std::cout << "READY|backend=mujoco_rb_v3|model=RB_v1.3|controller=RobotController"
              << "|leg_actuator=" << (options_.leg_pos ? "driver_pos" : "current_delta_i") << "|physics_hz=" << options_.physics_hz
              << "|control_hz=" << options_.sensor_hz
              << "|ground_truth_to_controller=0|wheel_regular_limit_nm=" << options_.wheel_torque
              << "|wheel_peak_limit_nm=" << (options_.wheel_torque_override ? options_.wheel_torque : 0.20)
              << "|mass_kg=" << geometry::total_mass_kg
              << "|plant_mass_kg=" << mj_getTotalmass(model_)
              << "|sensor_noise=" << options_.sensor_noise << "|noise_seed=" << options_.noise_seed
              << std::endl;
  }

  ~RbV3Runner() {
    if (socket_ >= 0) ::close(socket_);
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
  }

  int run() {
    const auto wall_start = std::chrono::steady_clock::now();
    while (!quit_) {
      step();
      if (options_.run_for_s > 0.0 && sim_time_s_ >= options_.run_for_s) break;
      if (options_.realtime) {
        const auto deadline = wall_start + std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(std::chrono::duration<double>(sim_time_s_));
        std::this_thread::sleep_until(deadline);
      }
    }
    std::cout << "EXIT|sim_time_s=" << sim_time_s_ << std::endl;
    return 0;
  }

 private:
  struct Joint {
    int joint = -1, qpos = -1, qvel = -1, position_sensor = -1, velocity_sensor = -1;
    int actuator = -1;
  };

  static RobotController::Config simulationControllerConfig(const Options& options) {
    auto config = rb1::control::defaultRobotConfig();
    config.lqr.sampling_period_s = static_cast<float>(1.0 / options.sensor_hz);
    // The extended startup fixture replaces the operator holding the robot.
    config.engage.immediate = true;
    config.engage.automatic = true;
    config.engage.wb_armed_only = false;
    config.lqr_inertia_length = true;
    config.lqr.max_pendulum_length_m = 0.24F;
    // Keep normal balance authority at the 0.13 N.m rated/base cap; only the
    // short rocking phases below are allowed the separately confirmed peak.
    config.lqr.max_ground_accel_m_s2 = std::min(config.lqr.wheel_ground_mu * 9.81F,
        2.0F * 0.13F / (geometry::wheel_radius_m * geometry::total_mass_kg));
    // The 15:20 balance controller has no actuation predictor (options.lqr_prediction is ignored).
    config.lqr_lead_adaptive = false;
    config.lqr_lead_s = 0.10F;   // App: lqr_lead_tall_s = lqr_lead_low_s = 0.10 (the controller does not know the true lag)
    config.tier.reflex_gate = true;
    // Virtual-rate lead feeds the commanded hip motion back into that same hip.
    config.tier.lead_gyro_only = options.gyro_hip_lead;
    // The current-controlled hip plant rings at ~31 Hz with 60 ms gyro lead.
    // Keep planned-lean preview and filtered body damping intact. Direct gyro
    // lead excites the fast loaded joint loop; its old 60 ms position-servo
    // tuning does not transfer to this current/impedance actuator model.
    config.tier.lead_s = 0.0F;
    config.tier.plan_accel_m_s2 = .9F;
    config.tier.plan_jerk_m_s3 = 10.0F;
    config.tier.hip_advance_s = config.tier.preview_s;
    config.tier.height_exp = .15F;
    config.tier.body_damping_s = .08F;
    config.tier.body_damp_lpf_hz = 4.5F;
    config.tier.knee_follow_hz = 1.0F;
    config.tier.knee_follow_rate_rad_s = .6F;
    config.tier.sus_adaptive = true;
    config.trim_acc = false;
    {
      // The firmware's recovery tuning (StandUp::tunedRecovery); --standup-profile fw keeps the StandUp::Config
      // defaults.
      if (options.standup_sim_profile) {
        config.standup = rb1::control::StandUp::tunedRecoveryRb(static_cast<float>(options.wheel_lag_s));
      }
      for (const auto& [key, v] : options.standup_overrides) {
        auto& su = config.standup;
        const auto f = static_cast<float>(v);
        if (key == "lift") su.catch_lift = v >= 0.5;
        else if (key == "lift_hip_min") su.lift_hip_rate_min_rad_s = f;
        else if (key == "lift_hip_travel") su.lift_hip_travel_rad = f;
        else if (key == "lift_hip_mintravel") su.lift_hip_min_travel_rad = f;
        else if (key == "lift_hip_accel") su.lift_hip_accel_rad_s2 = f;
        else if (key == "lift_hip_release") su.lift_hip_release_s = f;
        else if (key == "lift_cp") su.lift_cp_max_rad = f;
        else if (key == "lift_s") su.lift_max_s = f;
        else if (key == "lift_clearance") su.lift_target_clearance_m = f;
        else if (key == "knee_max") su.lift_knee_max_rad = f;
        else if (key == "knee_rate") su.lift_knee_rate_rad_s = f;
        else if (key == "knee_accel") su.lift_knee_accel_rad_s2 = f;
        else if (key == "knee_track") su.lift_knee_tracking_max_rad = f;
        else if (key == "knee_full") su.lift_knee_full = v >= 0.5;
        else if (key == "arm_hip") su.body_level_arm_hip_rad = f;
        else if (key == "arm_clearance") su.body_level_arm_clearance_m = f;
        else if (key == "accel_max") su.catch_accel_max = f;
        else if (key == "hip_accel") { su.catch_hip_accel_rad_s2 = f; su.lift_hip_accel_rad_s2 = f; }
        else if (key == "hip_rate_max") su.catch_hip_rate_max_rad_s = f;
        else if (key == "signed_posture") su.lift_signed_posture = v >= 0.5;
        else if (key == "body_gain") su.recovery_body_gain = f;
        else if (key == "body_damp") su.recovery_body_damping_s = f;
        else if (key == "body_rate_max") su.recovery_body_rate_max_rad_s = f;
        else if (key == "level") su.catch_body_leveling = v >= 0.5;
        else if (key == "wheel_assist") su.lift_wheel_assist = f;
        else if (key == "pitch_neutral") su.lift_pitch_neutral = f;
        else if (key == "knee_stage1") su.lift_knee_stage1_rad = f;
        else if (key == "knee_gate") su.lift_knee_lean_gate_rad = f;
        else if (key == "fold") su.fold_travel_rad = f;
        else if (key == "fold_rate") su.fold_rate_rad_s = f;
        else if (key == "slump") su.slump_travel_rad = f;
        else if (key == "cagain") su.catch_accel_gain = f;
        else if (key == "rise_rate") su.catch_rise_rate_rad_s = f;
        else if (key == "rise_accel") su.catch_rise_accel_rad_s2 = f;
        else if (key == "rise_dwell") su.catch_rise_start_dwell_s = f;
        else if (key == "rise_knee") su.catch_rise_knee_rad = f;
        else if (key == "riseposture") su.catch_rise_posture_rate_rad_s = f;
        else if (key == "wlead") su.catch_wheel_lead_s = f;
        else if (key == "sgain") su.catch_speed_gain = f;
        else if (key == "sint") su.catch_speed_integral_gain = f;
        else if (key == "hdead") su.catch_hip_lean_deadband_rad = f;
        else if (key == "blend") su.catch_blend_s = f;
        else if (key == "khold") su.kneel_hold_s = f;
        else if (key == "sintmax") su.catch_speed_integral_max_rad = f;
        else if (key == "pred") su.catch_wheel_prediction = v >= 0.5;
        else if (key == "capvel") su.catch_capture_velocity = v >= 0.5;
        else if (key == "wlagc") su.catch_wheel_lag_s = f;
        else if (key == "rise_hip_ff") su.rise_hip_ff = f;
        else if (key == "probe_rate") su.probe_rate_rad_s = f;
        else if (key == "probe_lift") su.probe_lift_rad = f;
        else if (key == "probe_lift2") su.probe_lift2_rad = f;
        else if (key == "probe_settle") su.probe_settle_s = f;
        else if (key == "probe_avg") su.probe_average_s = f;
        else if (key == "probe_relax") su.probe_relax_s = f;
        else if (key == "farm") su.floor_arm_clearance_m = f;
        else if (key == "farm_dwell") su.floor_arm_dwell_s = f;
        else if (key == "fclear") su.floor_clearance_m = f;
        else if (key == "somega_full") su.servo_omega_full_rad_s = f;
        else if (key == "somega_off") su.servo_omega_off_rad_s = f;
        else if (key == "eq_hip") su.eq_hip_travel_min_rad = f;
        else if (key == "eq_clear") su.eq_clearance_min_m = f;
        else if (key == "eq_speed") su.eq_speed_max_m_s = f;
        else if (key == "eq_rate") su.eq_rate_limit_rad_s = f;
        else if (key == "eq_limit") su.eq_limit_rad = f;
        else if (key == "eq_dwell") su.eq_dwell_s = f;
        else if (key == "assist") su.lift_assist_rate_rad_s = f;
        else if (key == "assist_travel") su.lift_assist_travel_rad = f;
        else if (key == "trackm") su.catch_tracking_margin_m_s = f;
        else if (key == "slmax") su.catch_speed_lean_max_rad = f;
        else if (key == "sfilter") su.catch_speed_filter_s = f;
        else if (key == "cadamp") su.catch_accel_damping = f;
        else if (key == "slump_rate") su.slump_rate_rad_s = f;
        else if (key == "head_rate") su.lift_head_rate_limit_rad_s = f;
        else if (key == "hip_delay") su.lift_hip_delay_s = f;
        else if (key == "knee_pitch_stop") su.lift_knee_pitch_stop_rad = f;
        else if (key == "hip_gain") su.catch_hip_gain = f;
        else if (key == "wheel_max") su.catch_wheel_max_rad_s = f;
        else if (key == "floor") su.floor_guard = v >= 0.5;
        else throw std::invalid_argument("unknown --standup-set key: " + key);
      }
      config.standup.enabled = true;
      config.standup.hold_at_kneel = false;
      config.standup.catch_wheel_lead_s = static_cast<float>(options.wheel_lag_s);
    }
    if (options.plain_balance) {
      config.tier.virtual_pitch = false;
      config.tier.trunk = false;
      config.tier.level = false;
      config.tier.suspension = false;
      config.tier.preview = false;
      config.lqr_lead_adaptive = false;
    }
    if (options.plan_vmax > 0.0) config.tier.plan_vmax_m_s = static_cast<float>(options.plan_vmax);
    // motor_reserve < 0 keeps the default; 0 still caps at the no-load speed; >= 1.5 turns the envelope off.
    if (options.ground_vmax > 0.0) config.lqr.max_ground_velocity_m_s = static_cast<float>(options.ground_vmax);
    if (options.track_margin >= 0.0) config.lqr.command_velocity_tracking_margin_m_s = static_cast<float>(options.track_margin);
    config.height.rate_m_s = .015F;
    config.height.accel_m_s2 = .05F;
    config.height.jerk_m_s3 = .20F;
    config.tier.joint_rate_gain_low = 1.0F;
    std::stringstream tuning(options.control_spec);
    std::string item;
    while (std::getline(tuning, item, ',')) {
      const auto eq = item.find('=');
      if (eq == std::string::npos) throw std::invalid_argument("--control expects key=value");
      const std::string key = item.substr(0, eq);
      const double v = number(item.substr(eq+1).c_str(), "--control");
      if (v < 0) throw std::invalid_argument("negative --control value");
      if (key == "trunk") config.tier.trunk = v >= .5;
      else if (key == "body_damping") config.tier.body_damping_s = v;
      else if (key == "body_lpf") config.tier.body_damp_lpf_hz = v;
      else if (key == "hip_lead") config.tier.lead_s = v;
      else if (key == "hip_lead_lpf") config.tier.lead_lpf_hz = v;
      else if (key == "knee_lpf") config.tier.knee_follow_hz = v;
      else if (key == "plan_accel") config.tier.plan_accel_m_s2 = v;
      else if (key == "plan_jerk") config.tier.plan_jerk_m_s3 = v;
      else if (key == "preview") config.tier.preview = v >= .5;
      else if (key == "carrier") config.tier.wheel_carrier_gain = v;
      else if (key == "wheel_lead") config.lqr_lead_s = v;
      else if (key == "trim_gain") config.lqr_trim_gain = v;
      else if (key == "lqr_dbc") config.lqr_deadband_comp_rad_s = v;
      else if (key == "dshare") config.tier.disturbance_share = v;
      else if (key == "hdb") config.tier.hip_deadband_rad = v;
      else if (key == "gate") config.tier.reflex_gate = v >= .5;
      else if (key == "hip_rate") config.tier.rate_rad_s = v;
      else if (key == "hip_max") config.tier.max_rad = v;
      else if (key == "lean_share") config.tier.lean_share = v;
      else if (key == "pred_delay" || key == "pred_lag") {}  // no predictor in this controller
      else if (key == "lead_adaptive") config.lqr_lead_adaptive = v >= .5;
      else throw std::invalid_argument("unknown --control key: " + key);
    }
    for (const auto& [key, v] : options.balance_overrides) {
      const auto f = static_cast<float>(v);
      auto& l = config.lqr;
      auto& ti = config.tier;
      if (key == "pos") l.position_cost_scale = l.station_position_cost_scale = f;
      else if (key == "pitch") l.pitch_cost_scale = l.station_pitch_cost_scale = f;
      else if (key == "vel") l.velocity_cost_scale = l.station_velocity_cost_scale = f;
      else if (key == "rate") l.pitch_rate_cost_scale = l.station_pitch_rate_cost_scale = f;
      else if (key == "input") l.input_cost_scale = l.station_input_cost_scale = f;
      else if (key == "spos") l.station_position_cost_scale = f;
      else if (key == "spitch") l.station_pitch_cost_scale = f;
      else if (key == "svel") l.station_velocity_cost_scale = f;
      else if (key == "srate") l.station_pitch_rate_cost_scale = f;
      else if (key == "sinput") l.station_input_cost_scale = f;
      else if (key == "spf") l.wheel_pitch_feedback_scale = f;
      else if (key == "plen") l.pendulum_length_per_com_height = f;
      else if (key == "minplen") l.min_pendulum_length_m = f;
      else if (key == "lead") config.lqr_lead_s = f;
      else if (key == "shold") l.station_hold_gain_hz = f;
      else if (key == "shmax") l.station_hold_max_v = f;
      else if (key == "sdamp") l.station_hold_velocity_damping_gain = f;
      else if (key == "scap") l.station_capture_gain = f;
      else if (key == "squiet") l.station_quiet_gain_scale = f;
      else if (key == "lean_share") ti.lean_share = f;
      else if (key == "tier_lead") ti.lead_s = f;
      else if (key == "tier_damp") ti.body_damping_s = f;
      else if (key == "level_tau") ti.level_tau_s = f;
      else if (key == "hexp") ti.height_exp = f;
      else if (key == "carrier") ti.wheel_carrier_gain = f;
      else if (key == "com_dx") ti.body_com_dx_m = f * 0.001F;   // controller's body CoM correction [mm]
      else if (key == "probe_apply") config.standup_probe_apply = v >= .5;
      else if (key == "imu_zero") config.imu_home_zero = v >= .5;
      else if (key == "probe_max") config.standup_probe_max_dx_m = f * 0.001F;   // [mm]
      else throw std::invalid_argument("unknown --balance-set key: " + key);
    }
    return config;
  }

  // RobotController measures the hip from the "hang" angle (LegHeight::hangHipCad). The MJCF joint is the CAD angle.
  static float hipCadOffset(std::size_t leg_index) {
    return leg_index % 2 == 0 ? rb1::control::LegHeight::hangHipCad() : 0.0F;
  }

  static int id(mjModel* model, mjtObj type, const char* name) {
    const int result = mj_name2id(model, type, name);
    if (result < 0) throw std::runtime_error(std::string("RB MJCF missing ") + name);
    return result;
  }

  void bindModel() {
    const std::array<const char*, 6> names = {
        "left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"};
    const std::array<const char*, 6> pos = {
        "enc_left_hip_pos", "enc_left_knee_pos", "enc_left_wheel_pos",
        "enc_right_hip_pos", "enc_right_knee_pos", "enc_right_wheel_pos"};
    const std::array<const char*, 6> vel = {
        "enc_left_hip_vel", "enc_left_knee_vel", "enc_left_wheel_vel",
        "enc_right_hip_vel", "enc_right_knee_vel", "enc_right_wheel_vel"};
    const std::array<const char*, 6> act = {
        "left_hip_torque", "left_knee_torque", "left_wheel_torque",
        "right_hip_torque", "right_knee_torque", "right_wheel_torque"};
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      auto& joint = joints_[i];
      joint.joint = id(model_, mjOBJ_JOINT, names[i]);
      joint.qpos = model_->jnt_qposadr[joint.joint];
      joint.qvel = model_->jnt_dofadr[joint.joint];
      joint.position_sensor = id(model_, mjOBJ_SENSOR, pos[i]);
      joint.velocity_sensor = id(model_, mjOBJ_SENSOR, vel[i]);
      joint.actuator = id(model_, mjOBJ_ACTUATOR, act[i]);
      // dynamic.xml clamps each wheel motor to its MJCF ctrlrange (+-0.13 N.m): the runner's 0.20 N.m
      // "peak" is only a command and never reached the plant. A what-if run widens the range to the cap.
      if ((i == 2 || i == 5) && options_.wheel_torque_whatif) {
        model_->actuator_ctrlrange[2 * joint.actuator] = -options_.wheel_torque;
        model_->actuator_ctrlrange[2 * joint.actuator + 1] = options_.wheel_torque;
      }
      for (const auto binding : {std::pair<int, int>{joint.position_sensor, mjSENS_JOINTPOS},
                                 {joint.velocity_sensor, mjSENS_JOINTVEL}}) {
        if (model_->sensor_type[binding.first] != binding.second ||
            model_->sensor_dim[binding.first] != 1 ||
            model_->sensor_objtype[binding.first] != mjOBJ_JOINT ||
            model_->sensor_objid[binding.first] != joint.joint)
          throw std::runtime_error("RB encoder must measure its joint, not world state");
      }
    }
    accel_sensor_ = id(model_, mjOBJ_SENSOR, "imu_accel");
    body_id_ = id(model_, mjOBJ_BODY, "body_link");
    left_wheel_body_ = id(model_, mjOBJ_BODY, "left_wheel");
    right_wheel_body_ = id(model_, mjOBJ_BODY, "right_wheel");
    gyro_sensor_ = id(model_, mjOBJ_SENSOR, "imu_gyro");
    const int imu_site = id(model_, mjOBJ_SITE, "imu");
    for (const auto binding : {std::pair<int, int>{accel_sensor_, mjSENS_ACCELEROMETER},
                               {gyro_sensor_, mjSENS_GYRO}}) {
      if (model_->sensor_type[binding.first] != binding.second ||
          model_->sensor_dim[binding.first] != 3 ||
          model_->sensor_objtype[binding.first] != mjOBJ_SITE ||
          model_->sensor_objid[binding.first] != imu_site)
        throw std::runtime_error("RB IMU must measure acceleration/gyro at its site");
    }
    if (model_->nq != 13 || model_->nv != 12 || model_->nu != 6 ||
        model_->nbody != 8 || (!options_.allow_model_mismatch &&
          std::abs(mj_getTotalmass(model_) - geometry::total_mass_kg) > 1.0e-4))
      throw std::runtime_error("RB MJCF joint or mass contract mismatch");
  }

  void configureImu() {
    rb1::core::ImuStateEstimator::Config config{};
    config.accel_correction_gain_hz = 5.0F;
    config.commanded_accel_blend = 0.0F;
    config.seed_window_samples = 50;
    estimator_.setConfig(rb1::control::balancingEstimatorConfig(config));
  }

  void setNonBlockingStdin() {
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  void openCommandSocket() {
    if (options_.command_port == 0) return;
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) throw std::runtime_error("command socket creation failed");
    int reuse = 1;
    setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options_.command_port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
      throw std::runtime_error("command socket bind failed");
    const int flags = fcntl(socket_, F_GETFL, 0);
    if (flags >= 0) fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
  }

  double sensor(int sensor_id, int component = 0) const {
    return data_->sensordata[model_->sensor_adr[sensor_id] + component];
  }

  void readCommands() {
    if (!stdin_closed_) {
      pollfd fd{STDIN_FILENO, POLLIN, 0};
      while (poll(&fd, 1, 0) > 0 && (fd.revents & POLLIN)) {
        char buffer[512];
        const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count == 0) { stdin_closed_ = true; break; }
        if (count < 0) break;
        input_buffer_.append(buffer, static_cast<std::size_t>(count));
        std::size_t end;
        while ((end = input_buffer_.find('\n')) != std::string::npos) {
          const auto line = input_buffer_.substr(0, end);
          input_buffer_.erase(0, end + 1);
          parseCommand(line);
        }
      }
    }
    if (socket_ < 0) return;
    for (;;) {
      char packet[512];
      const ssize_t count = recv(socket_, packet, sizeof(packet), 0);
      if (count <= 0) break;
      parseCommand(std::string(packet, static_cast<std::size_t>(count)));
    }
  }

  void parseCommand(const std::string& line) {
    std::istringstream stream(line);
    std::string op;
    stream >> op;
    if (op == "ENABLE") {
      // A robot that starts lying (--start-key other than spawn_extended) only becomes active through STANDUP:
      // the normal start-up would lock the legs to the extended pose and drag the lying robot there.
      if (options_.start_key != "spawn_extended" && !stood_up_) {
        std::cout << "ENABLE_REJECTED|reason=robot_starts_lying_use_STANDUP" << std::endl;
        return;
      }
      if (!estop_ && !enable_request_) start_pending_ = true;
      enable_request_ = true;
      disable_request_ = false;
    } else if (op == "DISABLE") {
      enable_request_ = false;
      disable_request_ = true;
      stood_up_ = false;                 // limp again: a lying start needs STANDUP once more
      command_.forward_m_s = command_.yaw_rad_s = 0.0F;
    } else if (op == "ESTOP") {
      estop_ = true;
      stood_up_ = false;
      enable_request_ = false;
      command_.forward_m_s = command_.yaw_rad_s = 0.0F;
    } else if (op == "CLEAR_ESTOP") estop_ = false;
    else if (op == "RESET") reset();
    else if (op == "STOP") command_.forward_m_s = command_.yaw_rad_s = 0.0F;
    else if (op == "GO") go_pending_ = true;
    else if (op == "STANDUP") {          // the rocking stand-up from the lying pose (start with --start-key home)
      if (!estop_ && !enable_request_) {
        standup_pending_ = true;
        // Height request after the hand-over, like App (SET standheight): full supported height unless
        // --stand-height MM (body/root height above the floor, as the GUI HEIGHT command) says otherwise.
        command_.height_m = options_.stand_height_mm > 0.0
            ? std::clamp(static_cast<float>(options_.stand_height_mm) * 0.001F - geometry::wheel_radius_m +
                             geometry::origins[0].z,
                         rb1::control::LegHeight::minHeight(), controller_.supportedMaxHeight())
            : controller_.supportedMaxHeight();
      }
      stood_up_ = !estop_;
      enable_request_ = true;
    }
    else if (op == "FLIP") standup_advance_pending_ = true;
    else if (op == "PUSH") {
      float force = 0.0F, duration = 0.0F;
      if (stream >> force >> duration && std::isfinite(force) && std::isfinite(duration) &&
          std::abs(force) <= 50.0F && duration > 0.0F && duration <= 2.0F) {
        push_force_n_ = force;
        push_until_s_ = sim_time_s_ + duration;
      } else std::cout << "ERROR|command=PUSH|reason=expected force_N (-50..50), duration_s (0..2)"
                       << std::endl;
    }
    else if (op == "DRIVE") {
      float forward = 0.0F, yaw = 0.0F;
      stream >> forward >> yaw;
      if ((forward != 0.0F || yaw != 0.0F) &&
          (!physics_started_ || output_.state != RobotController::State::Balancing || estop_)) {
        std::cout << "DRIVE_REJECTED|reason=balance_not_ready" << std::endl;
        return;
      }
      const float drive_max = static_cast<float>(options_.drive_max);
      command_.forward_m_s = std::clamp(std::isfinite(forward) ? forward : 0.0F, -drive_max, drive_max);
      command_.yaw_rad_s = std::clamp(std::isfinite(yaw) ? yaw : 0.0F, -1.5F, 1.5F);
    } else if (op == "HEIGHT") {
      float body_height = 0.0F;
      stream >> body_height;
      if (std::isfinite(body_height)) {
        const float axle_height = body_height - geometry::wheel_radius_m +
                                  geometry::origins[0].z;
        command_.height_m = std::clamp(axle_height,
            rb1::control::LegHeight::minHeight(), controller_.supportedMaxHeight());
        std::cout << "HEIGHT_ACCEPTED|target=" << command_.height_m + geometry::wheel_radius_m -
            geometry::origins[0].z << std::endl;
      }
    } else if (op == "POSE") {
      float lh = 0, lk = 0, rh = 0, rk = 0;
      if (stream >> lh >> lk >> rh >> rk) {
        const auto l = rb1::core::WholeBodyModel::legHeight(
            rb1::control::LegHeight::hangHipCad() + lh, lk);
        const auto r = rb1::core::WholeBodyModel::legHeight(
            rb1::control::LegHeight::hangHipCad() + rh, rk);
        command_.height_m = std::clamp(-0.5F * (l.height_m + r.height_m),
            rb1::control::LegHeight::minHeight(), rb1::control::LegHeight::maxHeight());
      }
    } else if (op == "QUIT") quit_ = true;
  }

  void reset() {
    controller_ = RobotController(simulationControllerConfig(options_));
    estimator_.reset();
    estimator_mode_set_ = false;
    mj_resetDataKeyframe(model_, data_, id(model_, mjOBJ_KEY, options_.start_key.c_str()));
    mj_forward(model_, data_);
    // The controller starts from the straight (fully extended) pose, as the firmware does at power-up.
    const float knee = rb1::control::LegHeight::kKneeMaxRad;
    const auto spawn_pose = rb1::control::LegHeight::forKnee(knee);
    const float hip = rb1::control::LegHeight::hangHipCad() + spawn_pose.hip_offset_rad;
    const auto spawn_com = rb1::core::WholeBodyModel{}.compute(hip, knee, hip, knee);
    const float hang_hip = rb1::control::LegHeight::hangHipCad();
    const auto hang_com = rb1::core::WholeBodyModel{}.compute(hang_hip, 0, hang_hip, 0);
    // V3's balance seed is in the virtual-pitch frame, not the raw IMU frame.
    const float virtual_offset = controller_.config().tier.virtual_pitch
        ? spawn_com.com_line_angle_rad - hang_com.com_line_angle_rad : 0.0F;
    controller_.setBalanceAngle(true,
        rb1::core::WholeBodyModel::equilibriumPitch(spawn_com) + virtual_offset);
    enable_request_ = disable_request_ = estop_ = false;
    // The extended spawn is held by the start-up fixture (physics frozen) until the controller is ready; a robot
    // that starts in another keyframe (the lying pose) is on the floor under gravity from the first tick.
    physics_started_ = options_.start_key != "spawn_extended";
    enable_seen_ = false;
    push_force_n_ = 0;
    push_until_s_ = 0;
    wheel_command_history_ = {};
    wheel_history_index_ = 0;
    wheel_enable_elapsed_s_ = 0;
    wheel_target_rad_s_.fill(0);
    wheel_speed_reference_rad_s_.fill(0);
    start_pending_ = go_pending_ = standup_pending_ = standup_advance_pending_ = stood_up_ = false;
    command_ = {};
    command_.height_m = spawn_pose.axle_below_hip_m;
    output_ = {};
    for (std::size_t i = 0; i < previous_leg_target_.size(); ++i) {
      previous_leg_target_[i] = i % 2 == 0 ? spawn_pose.hip_offset_rad : knee;
    }
    driver_target_ = previous_leg_target_;
    driver_vel_ff_.fill(0.0F);
    leg_target_rate_.fill(0.0F);
    leg_target_current_.fill(0.0F);
    leg_applied_current_.fill(0.0F);
    leg_measured_current_.fill(0.0F);
    leg_pos_integrator_nm_.fill(0.0F);
    for (std::size_t side = 0; side < 2; ++side)
      wheels_[side].onTorqueEnable(false);
    imu_seeded_ = false;
  }

  float sensorNoise(float sigma) {
    if (options_.sensor_noise == 0.0 || !physics_started_) return 0.0F;
    return static_cast<float>(options_.sensor_noise) * sigma * noise_normal_(noise_rng_);
  }

  void updateController() {
    const float dt = static_cast<float>(1.0 / options_.sensor_hz);
    const std::uint64_t now_us = static_cast<std::uint64_t>(sim_time_s_ * 1.0e6);
    std::array<float, 3> accel{}, gyro{};
    if (!physics_started_) {
      // The extended startup fixture holds the mechanism stationary until
      // balance commands and the wheel torque-enable settle window are ready.
      accel[2] = 9.80665F;
    } else {
      if (options_.imu_model) {
        std::array<float, 3> true_accel{}, true_gyro{};
        for (int i = 0; i < 3; ++i) {
          true_accel[static_cast<std::size_t>(i)] = static_cast<float>(sensor(accel_sensor_, i));
          true_gyro[static_cast<std::size_t>(i)] = static_cast<float>(sensor(gyro_sensor_, i));
        }
        const float wheel_speed = 0.5F * static_cast<float>(sensor(joints_[2].velocity_sensor) +
                                                            sensor(joints_[5].velocity_sensor));
        mpu_.step(true_gyro, true_accel, dt, wheel_speed, gyro, accel);
      } else {
        for (int i = 0; i < 3; ++i) {
          accel[static_cast<std::size_t>(i)] = static_cast<float>(sensor(accel_sensor_, i)) + sensorNoise(.03F);
          gyro[static_cast<std::size_t>(i)] = static_cast<float>(sensor(gyro_sensor_, i)) + sensorNoise(.005F);
        }
      }
    }
    if (options_.imu_model) {
      // The firmware's estimator switching (App): gyro-dominant config in Balancing, and in the stand-up when
      // StandUp::Config::gyro_dominant_imu; the base config (fast accelerometer correction, bias learning) otherwise.
      const bool gyro_dominant = output_.state == RobotController::State::Balancing ||
          (output_.state == RobotController::State::StandUp && controller_.config().standup.gyro_dominant_imu);
      if (gyro_dominant != estimator_gyro_dominant_ || !estimator_mode_set_) {
        estimator_.setConfig(gyro_dominant
            ? rb1::control::balancingEstimatorConfig(rb1::core::ImuStateEstimator::Config{})
            : rb1::core::ImuStateEstimator::Config{});
        estimator_gyro_dominant_ = gyro_dominant;
        estimator_mode_set_ = true;
      }
    }
    if (options_.imu_hw && !options_.imu_model && physics_started_) {
      // MPU6500 as the robot configures it: +-range per axis, then the ~44 Hz accelerometer low-pass.
      const float limit = static_cast<float>(options_.imu_range_g) * 9.80665F;
      const float a = static_cast<float>(1.0 - std::exp(-2.0 * 3.14159265 * 44.0 * static_cast<double>(dt)));
      for (std::size_t i = 0; i < 3; ++i) {
        const float clipped = std::clamp(accel[i], -limit, limit);
        imu_lp_[i] = imu_lp_seeded_ ? imu_lp_[i] + a * (clipped - imu_lp_[i]) : clipped;
        accel[i] = imu_lp_[i];
      }
      imu_lp_seeded_ = true;
    }
    const auto velocity = controller_.imuVelocity();
    const auto estimate = estimator_.update(accel, gyro, dt,
        static_cast<float>(sim_time_s_), 0.0F, output_.legs_moving, !enable_request_,
        {velocity.horizontal_m_s, velocity.vertical_m_s, physics_started_ && velocity.valid});
    imu_seeded_ = estimate.seeded || imu_seeded_;

    RobotController::Input in{};
    in.now_us = now_us;
    in.dt_s = dt;
    in.pitch_rad = estimate.pitch_rad;
    in.pitch_rate_rad_s = estimate.pitch_rate_rad_s;
    in.yaw_rate_rad_s = estimate.yaw_rate_rad_s;
    in.accel_x_m_s2 = accel[0];
    in.accel_z_m_s2 = accel[2];
    accel_norm_ = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    in.legs_measured = true;
    in.sensors_ok = imu_seeded_ && estimate.valid;
    in.legs_ready = true;
    for (std::size_t side = 0; side < 2; ++side) {
      const std::size_t hip = 3 * side;
      const std::size_t knee = hip + 1;
      const std::size_t wheel = hip + 2;
      in.hip_rad[side] = static_cast<float>(sensor(joints_[hip].position_sensor)) - hipCadOffset(0) + sensorNoise(.0005F);
      in.knee_rad[side] = static_cast<float>(sensor(joints_[knee].position_sensor)) + sensorNoise(.0005F);
      in.hip_rate_rad_s[side] = static_cast<float>(sensor(joints_[hip].velocity_sensor)) + sensorNoise(.01F);
      in.knee_rate_rad_s[side] = static_cast<float>(sensor(joints_[knee].velocity_sensor)) + sensorNoise(.01F);
      in.wheel_rad_s[side] = static_cast<float>(sensor(joints_[wheel].velocity_sensor)) + sensorNoise(.03F);
      if (wheel_quant_rad_s_ > 0.0F) {  // experiment: the BL4012 reports speed in ~7 rpm steps
        in.wheel_rad_s[side] = std::round(in.wheel_rad_s[side] / wheel_quant_rad_s_) * wheel_quant_rad_s_;
      }
      in.wheel_pos_rad[side] = static_cast<float>(sensor(joints_[wheel].position_sensor)) + sensorNoise(.0005F);
      in.leg_chain_rad += 0.5F * (in.hip_rad[side] + in.knee_rad[side]);
      // Hip joint torque as the drives would report it: the lagged leg current of the previous tick.
      in.hip_torque_nm[side] = kKtNmPerA * leg_measured_current_[2 * side] * hip_torque_scale_ +
          hip_torque_bias_nm_ + sensorNoise(hip_torque_noise_nm_);
    }
    in.hip_torque_valid = hip_torque_valid_;
    for (std::size_t i = 0; i < previous_leg_target_.size(); ++i) {
      const std::size_t joint = (i / 2) * 3 + (i % 2);
      const float q = static_cast<float>(sensor(joints_[joint].position_sensor));
      in.legs_ready = in.legs_ready && std::abs(previous_leg_target_[i] + hipCadOffset(i) - q) < 0.08F;
    }

    RobotController::Command cmd{};
    cmd.start = start_pending_ && in.sensors_ok;
    if (cmd.start) start_pending_ = false;
    cmd.standup = standup_pending_ && in.sensors_ok;
    if (cmd.standup) standup_pending_ = false;
    cmd.standup_advance = standup_advance_pending_;
    standup_advance_pending_ = false;
    cmd.go = go_pending_;
    go_pending_ = false;
    cmd.disable = disable_request_ || estop_;
    cmd.forward_m_s = enable_request_ && !cmd.disable ? command_.forward_m_s : 0.0F;
    cmd.yaw_rad_s = enable_request_ && !cmd.disable ? command_.yaw_rad_s : 0.0F;
    cmd.height_m = command_.height_m;
    output_ = controller_.update(in, cmd);

    // Like App: the leg path is the controller's hip/knee command, the same for both legs.
    const bool standing_up = output_.state == RobotController::State::StandUp;
    const float hip_cmd = standing_up ? output_.leg_hip_offset_rad : controller_.hipCommand();
    const float knee_cmd = standing_up ? output_.leg_knee_offset_rad : controller_.kneeCommand();
    const std::array<float, 4> commanded_target{hip_cmd, knee_cmd, hip_cmd, knee_cmd};
    const auto target = output_.state == RobotController::State::Fault
        ? previous_leg_target_ : commanded_target;
    {
      // Filtered rate of the common stand-up path (20 ms low-pass), the driver velocity feed-forward in StandUp only.
      const float alpha = static_cast<float>(dt / (0.02 + dt));
      const std::array<float, 2> common{hip_cmd, knee_cmd};
      for (std::size_t k = 0; k < 2; ++k) {
        if (standing_up) {
          const float raw = std::clamp((common[k] - standup_ff_prev_[k]) / static_cast<float>(dt), -6.0F, 6.0F);
          standup_ff_rate_[k] += alpha * (raw - standup_ff_rate_[k]);
        } else {
          standup_ff_rate_[k] = 0.0F;
        }
        standup_ff_prev_[k] = common[k];
      }
    }
    // Dead time between the controller and the driver position loop (the command reaches the driver late).
    std::array<float, 4> driver_target = target;
    if (leg_delay_ticks_ > 0U) {
      auto& slot = leg_delay_buf_[leg_delay_head_];
      const std::array<float, 4> late = slot;
      slot = target;
      for (std::size_t i = 0; i < 4; ++i) driver_target[i] = leg_delay_init_ ? late[i] : target[i];
      if (leg_delay_head_ + 1U == leg_delay_ticks_) leg_delay_init_ = true;
      leg_delay_head_ = (leg_delay_head_ + 1U) % leg_delay_ticks_;
    }
    for (std::size_t i = 0; i < target.size(); ++i) {
      leg_target_rate_[i] = (target[i] - previous_leg_target_[i]) / dt;
      previous_leg_target_[i] = target[i];
      const std::size_t side = i / 2;
      const std::size_t joint = side * 3 + (i % 2);
      const float q = static_cast<float>(sensor(joints_[joint].position_sensor));
      // The latest frame the driver holds until the next one: target and, like LegLock::velFfTurns, in stand-up
      // only, the path rate. The driver loop itself runs at the physics rate (driverPositionLoop): the real driver
      // closes it on its own clock, whatever the controller rate.
      driver_target_[i] = driver_target[i];
      driver_vel_ff_[i] = standing_up ? standup_ff_rate_[i % 2] : 0.0F;
      in.legs_ready = in.legs_ready && std::abs(target[i] + hipCadOffset(i) - q) < 0.08F;
    }
    for (std::size_t side = 0; side < 2; ++side) {
      auto& wheel = wheels_[side];
      wheel.onTorqueEnable(output_.wheels_enabled && !cmd.disable);
      float physical_speed = output_.wheel_cmd_rad_s[side];
      if (wheel_delay_ticks_ > 0) {
        physical_speed = wheel_command_history_[wheel_history_index_][side];
        wheel_command_history_[wheel_history_index_][side] = output_.wheel_cmd_rad_s[side];
      }
      if (wheel_deadband_rad_s_ > 0.0F) {  // experiment: the driver ignores the first rad/s of the command
        physical_speed = std::copysign(std::max(std::abs(physical_speed) - wheel_deadband_rad_s_, 0.0F), physical_speed);
      }
      wheel_target_rad_s_[side] = physical_speed;
    }
    if (wheel_delay_ticks_ > 0) wheel_history_index_ = (wheel_history_index_ + 1) % wheel_delay_ticks_;
    if (!output_.wheels_enabled) wheel_command_history_ = {};
    emitTelemetry(in, estimate);
  }

  // GDS68 position + passthrough: cascaded position -> velocity -> torque loop inside the driver, on the latest
  // frame (driver_target_, driver_vel_ff_). The driver has no gravity model, so the integrator removes the static
  // error. Gains are PROVISIONAL (the real driver pos_gain / vel_gain / vel_integrator_gain are set on the driver,
  // not in this repository): they reproduce the stiffness and damping of the delta-I path (kJointKp, kJointKd).
  void driverPositionLoop(float dt) {
    for (std::size_t i = 0; i < 4; ++i) {
      if (!options_.leg_pos) {
        leg_target_current_[i] = 0.0F;  // POS is the only leg drive of this controller
        continue;
      }
      float tau = 0.0F;
      if (output_.legs_locked) {
        const std::size_t joint = (i / 2) * 3 + (i % 2);
        const float q = static_cast<float>(sensor(joints_[joint].position_sensor));
        const float qd = static_cast<float>(sensor(joints_[joint].velocity_sensor));
        const float vel_cmd = std::clamp(static_cast<float>(options_.leg_kp_s) *
                                             (driver_target_[i] + hipCadOffset(i) - q) + driver_vel_ff_[i],
                                         -kDriverVelLimitRadS, kDriverVelLimitRadS);
        const float vel_err = vel_cmd - qd;
        const float unsat = static_cast<float>(options_.leg_kv) * vel_err + leg_pos_integrator_nm_[i];
        tau = std::clamp(unsat, -legJointLimitNm(i), legJointLimitNm(i));
        // Anti-windup: integrate only while the torque is not saturated against the error.
        if (tau == unsat) {
          leg_pos_integrator_nm_[i] = std::clamp(leg_pos_integrator_nm_[i] + kVelIntegratorGain * vel_err * dt,
                                                 -legJointLimitNm(i), legJointLimitNm(i));
        }
      } else {
        leg_pos_integrator_nm_[i] = 0.0F;
      }
      leg_target_current_[i] = tau / kKtNmPerA;
    }
  }

  void step() {
    while (next_scheduled_ < options_.schedule.size() &&
           options_.schedule[next_scheduled_].first <= sim_time_s_ + 1.0e-12) {
      parseCommand(options_.schedule[next_scheduled_++].second);
    }
    readCommands();
    const double dt = model_->opt.timestep;
    if (sim_time_s_ + 1.0e-12 >= next_control_s_) {
      next_control_s_ += 1.0 / options_.sensor_hz;
      updateController();
    }
    const bool active = enable_request_ && !estop_ &&
        output_.state != RobotController::State::Idle;
    enable_seen_ = enable_seen_ || enable_request_;
    wheel_enable_elapsed_s_ = output_.wheels_enabled ? wheel_enable_elapsed_s_ + dt : 0;
    const bool balance_ready = output_.state == RobotController::State::Balancing &&
        wheel_enable_elapsed_s_ >= options_.wheel_delay_s &&
        !wheels_[0].settling() && !wheels_[1].settling();
    physics_started_ = physics_started_ || balance_ready ||
        output_.state == RobotController::State::Fault ||
        (enable_seen_ && (!enable_request_ || estop_));
    driverPositionLoop(static_cast<float>(dt));
    // Leg joints: current lag, then the RB motor map. Joint order of i: left hip, left knee, right hip, right knee.
    std::array<float, 4> leg_request{};
    for (std::size_t i = 0; i < 4; ++i) {
      const double alpha = dt / (options_.leg_current_lag_s + dt);
      if (active && output_.legs_locked)
        leg_applied_current_[i] += alpha * (leg_target_current_[i] - leg_applied_current_[i]);
      else
        leg_applied_current_[i] = 0.0F;
      leg_measured_current_[i] = static_cast<float>(leg_applied_current_[i]);
      leg_request[i] = active && output_.legs_locked
          ? std::clamp(kKtNmPerA * leg_applied_current_[i], -legJointLimitNm(i), legJointLimitNm(i))
          : 0.0F;
    }
    for (std::size_t side = 0; side < 2; ++side) {
      const auto applied = legMotorMap(leg_request[2 * side], leg_request[2 * side + 1]);
      data_->ctrl[joints_[side * 3].actuator] = applied.first;
      data_->ctrl[joints_[side * 3 + 1].actuator] = applied.second;
    }
    // Each measured Iq channel follows its own simulated actuator state.
    for (std::size_t side = 0; side < 2; ++side) {
      auto& wheel = wheels_[side];
      const bool recovery_peak = output_.state == RobotController::State::StandUp &&
          controller_.standupState().wheel_peak_requested && !options_.wheel_torque_override;
      const float torque_cap = recovery_peak ? 0.20F : static_cast<float>(options_.wheel_torque);
      wheel.setMaxTorqueNm(torque_cap, options_.wheel_torque_whatif ? 2.0F : 0.20F);
      // Reported wheel lag is speed-response lag, not motor-current lag.
      const double alpha = options_.wheel_lag_s > 0 ? dt / (options_.wheel_lag_s + dt) : 1;
      if (active && output_.wheels_enabled) {
        wheel_speed_reference_rad_s_[side] += static_cast<float>(alpha) *
            (wheel_target_rad_s_[side] - wheel_speed_reference_rad_s_[side]);
      } else wheel_speed_reference_rad_s_[side] = 0;
      wheel.onMovingSpeed(wheel_speed_reference_rad_s_[side]);
      wheel.step(static_cast<float>(dt));
      const std::size_t index = 3 * side + 2;
      const float physical_velocity = static_cast<float>(data_->qvel[joints_[index].qvel]);
      data_->ctrl[joints_[index].actuator] = active && output_.wheels_enabled
          ? wheel.computeTorqueNm(physical_velocity) : 0.0;
    }
    // A controller stop cannot suspend gravity or contact dynamics.
    data_->xfrc_applied[6 * body_id_] = sim_time_s_ < push_until_s_ ? push_force_n_ : 0.0F;
    if (physics_started_) mj_step(model_, data_);
    sim_time_s_ += dt;
    if (sim_time_s_ >= next_telemetry_s_) {
      next_telemetry_s_ += 0.02;
      emitView();
    }
  }

  void emitTelemetry(const RobotController::Input& in,
                     const rb1::core::ImuStateEstimator::Output& estimate) {
    if (sim_time_s_ < next_state_s_) return;
    next_state_s_ += 1.0 / options_.state_hz;
    const char* mode = rb1::control::toString(output_.state);
    const auto wheel_l = static_cast<float>(sensor(joints_[2].velocity_sensor));
    const auto wheel_r = static_cast<float>(sensor(joints_[5].velocity_sensor));
    const auto com = rb1::core::WholeBodyModel{}.compute(
        in.hip_rad[0], in.knee_rad[0], in.hip_rad[1], in.knee_rad[1]);
    std::cout << "STATE|t=" << sim_time_s_ << "|mode=" << mode
              << "|pitch=" << in.pitch_rad << "|pitch_rate=" << in.pitch_rate_rad_s
              << "|pitch_rate_corrected=" << in.pitch_rate_rad_s
              << "|reflex_gate=" << controller_.reflexGate()
              << "|hip_trunk_request=" << controller_.trunkSwingRequest()
              << "|hip_trunk_command=" << controller_.trunkHip()
              << "|hip_level=" << controller_.levelHip()
              << "|trunk_authority=" << controller_.trunkAuthority()
              << "|trunk_workspace_limited=" << controller_.trunkWorkspaceLimited()
              << "|hip_target=" << controller_.hipCommand()
              << "|knee_target=" << controller_.kneeCommand()
              << "|hip_lead_gyro=" << controller_.config().tier.lead_gyro_only
              << "|accel_mag=" << estimate.accel_magnitude_m_s2
              << "|imu_seeded=" << imu_seeded_ << "|imu_tilt_valid=" << estimate.valid
              << "|wheel_velocity=" << 0.5F * geometry::wheel_radius_m * (wheel_l + wheel_r)
              << "|wheel_position=" << controller_.groundPosition()
              << "|wheel_left_position=" << geometry::wheel_radius_m *
                  (in.wheel_pos_rad[0] + in.pitch_rad + in.hip_rad[0] + in.knee_rad[0])
              << "|wheel_right_position=" << geometry::wheel_radius_m *
                  (in.wheel_pos_rad[1] + in.pitch_rad + in.hip_rad[1] + in.knee_rad[1])
              << "|joint_lh=" << in.hip_rad[0] << "|joint_lk=" << in.knee_rad[0]
              << "|joint_rh=" << in.hip_rad[1] << "|joint_rk=" << in.knee_rad[1]
              << "|enabled=" << enable_request_ << "|estop=" << estop_
              << "|startup_fixture=" << !physics_started_
              << "|drive_ready=" << (physics_started_ && output_.state == RobotController::State::Balancing &&
                                         output_.wheels_enabled && in.sensors_ok && !estop_)
              << "|fault_reason=" << (controller_.lastFault() && *controller_.lastFault()
                                       ? controller_.lastFault() : "none")
              << "|height_request=" << (command_.height_m > 0.0F)
              << "|height_target=" << controller_.heightTarget() + geometry::wheel_radius_m -
                  geometry::origins[0].z
              << "|max_body_height=" << controller_.supportedMaxHeight() + geometry::wheel_radius_m -
                  geometry::origins[0].z
              << "|height_motion=" << output_.legs_moving
              << "|height_settled=" << (!output_.legs_moving && output_.legs_locked)
              << "|height_rejected=0|flight=unknown|airborne=unknown|touchdown=unknown"
              << "|torque_valid=1|skill=none|dance_height_target=0|dance_yaw_rate=0"
              << "|lqr_pitch_state=" << controller_.virtualPitch()
              << "|balance_ref=" << controller_.balanceRef()
              << "|ground_velocity=" << controller_.groundVelocity()
              << "|wheel_cmd_l=" << output_.wheel_cmd_rad_s[0]
              << "|wheel_cmd_r=" << output_.wheel_cmd_rad_s[1]
              << "|station_capture_distance=unknown|wb_com_line_angle=" << com.com_line_angle_rad
              << "|fz_l=unknown|fz_r=unknown|loop_hz=" << options_.sensor_hz
              << "|ground_truth_to_controller=0"
              << "|leg_iq_lh=" << leg_measured_current_[0]
              << "|leg_iq_lk=" << leg_measured_current_[1]
              << "|wheel_tq_l=" << data_->ctrl[joints_[2].actuator]
              << "|wheel_tq_r=" << data_->ctrl[joints_[5].actuator]
              << "|wheel_force_l=" << data_->actuator_force[joints_[2].actuator]
              << "|sus_dz=" << controller_.suspensionOffset()
              << "|sus_db=" << controller_.suspensionDeadband()
              << "|su_phase=" << static_cast<int>(controller_.standupState().phase)
              << "|su_phase_name=" << rb1::control::StandUp::name(controller_.standupState().phase)
              << "|su_lean=" << controller_.standupLean()
              << "|su_hip=" << controller_.standupState().hip_target_rad
              << "|su_wheel=" << controller_.standupState().wheel_rad_s
              << "|su_lift=" << controller_.standupState().lift_state
              << "|su_knee_rate=" << controller_.standupState().knee_rate_rad_s
              << "|su_floor_armed=" << controller_.standupState().floor_armed
              << "|su_floor_infeasible=" << controller_.standupState().floor_infeasible
              << "|su_floor_pred=" << controller_.standupState().floor_predicted_clearance_m
              << "|su_floor_rate=" << controller_.standupState().floor_clearance_rate_m_s
              << "|su_floor_lo=" << controller_.standupState().floor_hip_rate_lower_rad_s
              << "|su_floor_hi=" << controller_.standupState().floor_hip_rate_upper_rad_s
              << "|su_ready=" << controller_.standupState().handoff_ready
              << "|su_settle=" << controller_.standupState().settle_s
              << "|su_hip_lim=" << controller_.standupState().hip_limited
              << "|su_wheel_lim=" << controller_.standupState().wheel_limited
              << "|su_floor_lim=" << controller_.standupState().floor_limited
              << "|su_wtrack=" << controller_.standupState().wheel_tracking_error_rad_s
              << "|su_clear=" << controller_.standupState().body_clearance_m
              << "|su_cp=" << controller_.standupState().capture_point_rad
              << "|su_caprate=" << controller_.standupState().capture_rate_rad_s
              << "|su_blend=" << controller_.standupState().catch_blend
              << "|su_predw=" << controller_.standupState().wheel_prediction_weight
              << "|su_accel=" << controller_.standupState().wheel_accel_m_s2
              << "|su_lead=" << controller_.standupState().wheel_lead_s
              << "|su_prepw=" << controller_.standupState().handoff_prepare_weight
              << "|su_transw=" << controller_.standupState().transfer_weight
              << "|su_eq=" << controller_.standupState().capture_equilibrium_rad
              << "|su_leanref=" << controller_.standupState().wheel_lean_ref_rad
              << "|su_servo=" << controller_.standupState().capture_servo_weight
              << "|su_probe=" << controller_.standupState().probe_stage
              << "|su_probe_tq=" << controller_.standupState().probe_torque_nm
              << "|su_probe_dx=" << controller_.probeComDx()
              << "|imu_zero=" << controller_.imuPitchOffset()
              << "|com_dx_used=" << controller_.config().tier.body_com_dx_m
              << "|su_omega=" << controller_.standupOmega()
              << "|cmd_hip=" << controller_.hipCommand() << "|cmd_knee=" << controller_.kneeCommand()
              << "|imu_accel_norm=" << accel_norm_
              << "|held_quiet=" << controller_.heldQuiet()
              << "|held_outer_off=" << controller_.heldOuterOff()
              << "|wheel_clearance=" << wheelClearance()
              << "|ang_mom_y=" << angularMomentumY() << "|body_wy=" << data_->qvel[4]
              << "|axle_x=" << data_->xpos[3 * left_wheel_body_] << "|axle_z=" << data_->xpos[3 * left_wheel_body_ + 2]
              << "|com_x=" << data_->subtree_com[3 * body_id_] << "|com_z=" << data_->subtree_com[3 * body_id_ + 2]
              << "|leg_iq_rh=" << leg_measured_current_[2]
              << "|leg_iq_rk=" << leg_measured_current_[3]
              << std::endl;
  }

  // Logging only: angular momentum of the whole robot about its CoM (about +Y).
  double angularMomentumY() {
    mj_subtreeVel(model_, data_);
    return data_->subtree_angmom[3 * body_id_ + 1];
  }

  // Logging only: lowest wheel-centre height above the floor minus the radius.
  double wheelClearance() const {
    if (left_wheel_body_ < 0 || right_wheel_body_ < 0) return 0.0;
    return std::min(data_->xpos[3 * left_wheel_body_ + 2], data_->xpos[3 * right_wheel_body_ + 2]) -
           geometry::wheel_radius_m;
  }

  void emitView() {
    std::cout << "VIEW|t=" << sim_time_s_ << "|x=" << data_->qpos[0]
              << "|physics_t=" << data_->time
              << "|y=" << data_->qpos[1] << "|z=" << data_->qpos[2]
              << "|qw=" << data_->qpos[3] << "|qx=" << data_->qpos[4]
              << "|qy=" << data_->qpos[5] << "|qz=" << data_->qpos[6]
              << "|body_height=" << data_->qpos[2]
              << "|lh=" << data_->qpos[joints_[0].qpos]
              << "|lk=" << data_->qpos[joints_[1].qpos]
              << "|lw=" << data_->qpos[joints_[2].qpos]
              << "|rh=" << data_->qpos[joints_[3].qpos]
              << "|rk=" << data_->qpos[joints_[4].qpos]
              << "|rw=" << data_->qpos[joints_[5].qpos] << std::endl;
  }

  Options options_;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  RobotController controller_;
  std::mt19937 noise_rng_;
  rb1::sim::Mpu6500Model mpu_;
  bool estimator_gyro_dominant_ = false, estimator_mode_set_ = false;
  std::normal_distribution<float> noise_normal_{0.0F, 1.0F};
  RobotController::Command command_{};
  RobotController::Output output_{};
  rb1::core::ImuStateEstimator estimator_;
  std::array<rb1::sim::SimBl4012Wheel, 2> wheels_{{
      rb1::sim::SimBl4012Wheel({1}), rb1::sim::SimBl4012Wheel({2})}};
  std::array<std::array<float, 2>, 400> wheel_command_history_{};
  std::size_t wheel_history_index_ = 0, wheel_delay_ticks_ = 0;
  // Hip torque as reported by the leg drives (CoM probe). RB1_SIM_HIP_TQ=0 hides it; the others model a drive
  // current error: scale (Kt error), bias [N*m] and white noise [N*m].
  static float envFloat(const char* name, float fallback) {
    const char* v = std::getenv(name);
    return v ? static_cast<float>(std::atof(v)) : fallback;
  }
  bool hip_torque_valid_ = envFloat("RB1_SIM_HIP_TQ", 1.0F) >= 0.5F;
  float hip_torque_scale_ = envFloat("RB1_SIM_HIP_TQ_SCALE", 1.0F);
  float hip_torque_bias_nm_ = envFloat("RB1_SIM_HIP_TQ_BIAS", 0.0F);
  float hip_torque_noise_nm_ = envFloat("RB1_SIM_HIP_TQ_NOISE", 0.0F);
  float wheel_quant_rad_s_ = std::getenv("RB1_SIM_WHEEL_QUANT") ? static_cast<float>(std::atof(std::getenv("RB1_SIM_WHEEL_QUANT"))) : 0.0F;
  float wheel_deadband_rad_s_ = std::getenv("RB1_SIM_WHEEL_DEADBAND") ? static_cast<float>(std::atof(std::getenv("RB1_SIM_WHEEL_DEADBAND"))) : 0.0F;
  double wheel_enable_elapsed_s_ = 0;
  std::array<float, 2> wheel_target_rad_s_{}, wheel_speed_reference_rad_s_{};
  std::array<Joint, 6> joints_{};
  std::array<float, 4> previous_leg_target_{};
  std::array<float, 4> leg_target_rate_{};
  std::array<float, 4> leg_target_current_{};
  std::array<float, 4> leg_applied_current_{};
  std::array<float, 4> leg_measured_current_{};
  std::array<float, 4> leg_pos_integrator_nm_{};
  std::array<float, 4> driver_target_{}, driver_vel_ff_{};  // the latest leg frame the driver holds
  std::array<float, 2> standup_ff_rate_{}, standup_ff_prev_{};  // [hip, knee] common path rate (POS stand-up feed-forward)
  int accel_sensor_ = -1, gyro_sensor_ = -1, body_id_ = -1;
  int left_wheel_body_ = -1, right_wheel_body_ = -1;
  std::uint64_t socket_port_ = 0;
  int socket_ = -1;
  double sim_time_s_ = 0.0, next_control_s_ = 0.0, next_telemetry_s_ = 0.0,
         next_state_s_ = 0.0;
  bool enable_request_ = false, disable_request_ = false, estop_ = false;
  std::size_t next_scheduled_ = 0;
  int last_jump_phase_ = 0;
  bool start_pending_ = false, go_pending_ = false, jump_pending_ = false;
  float accel_norm_ = 0.0F;  // |IMU specific force| of the last control tick (telemetry)
  std::array<float, 3> imu_lp_{};
  bool imu_lp_seeded_ = false;
  std::array<std::array<float, 4>, 64> leg_delay_buf_{};
  std::size_t leg_delay_ticks_ = 0U, leg_delay_head_ = 0U;
  bool leg_delay_init_ = false;
  bool standup_pending_ = false, standup_advance_pending_ = false, imu_seeded_ = false;
  bool stood_up_ = false;   // a lying start: STANDUP was given (ENABLE is refused until then)
  float jump_forward_m_ = std::numeric_limits<float>::quiet_NaN();
  bool physics_started_ = false;
  bool enable_seen_ = false;
  float push_force_n_ = 0.0F;
  double push_until_s_ = 0.0;
  bool quit_ = false, stdin_closed_ = false;
  std::string input_buffer_;

  static constexpr float kKtNmPerA = 0.47F;
  static constexpr float kCurrentLimitA = 8.0F;
  static constexpr float kCurrentCompGain = 0.2F;
  static constexpr float kCurrentLagS = 0.020F;
  static constexpr float kJointKp = 12.0F;
  static constexpr float kJointKd = 0.8F;
  // RB v1.3 leg motors (model/actuators.yaml). The crank motor (knee, 5 N.m peak at the
  // crank) drives hip + knee together through the parallelogram; the hip motor (BL4012 x 7.5 belt) adds only to the
  // hip. Joint torque (hip, knee) = (tau_hip_motor + tau_knee_motor, tau_knee_motor).
  static constexpr float kKneeMotorLimitNm = 5.0F;
  static constexpr float kHipMotorLimitNm = 1.5F;
  static constexpr float kTorqueLimitNm = kKneeMotorLimitNm + kHipMotorLimitNm;  // hip joint, both motors
  static float legJointLimitNm(std::size_t leg_index) { return leg_index % 2 == 0 ? kTorqueLimitNm : kKneeMotorLimitNm; }
  // Requested joint torques of one leg -> torques that the two motors can really deliver.
  static std::pair<float, float> legMotorMap(float hip_request, float knee_request) {
    const float knee_motor = std::clamp(knee_request, -kKneeMotorLimitNm, kKneeMotorLimitNm);
    const float hip_motor = std::clamp(hip_request - knee_motor, -kHipMotorLimitNm, kHipMotorLimitNm);
    return {hip_motor + knee_motor, knee_motor};
  }
  // Provisional GDS68 POS-loop model (see updateController): kPosGainPerS * kVelGainNmSPerRad == kJointKp and
  // kVelGainNmSPerRad == kJointKd. Driver Set_Limits: 20 rotor turns/s over the 8:1 gear.
  static constexpr float kVelGainNmSPerRad = kJointKd;
  static constexpr float kPosGainPerS = kJointKp / kJointKd;
  static constexpr float kVelIntegratorGain = 5.0F;
  // RB hip: BL4012 400 rpm rated through the 1:7.5 belt = 5.6 rad/s (no-load 13.3). Provisional until the driver exists.
  static constexpr float kDriverVelLimitRadS = 5.0F;
};
}  // namespace

int main(int argc, char** argv) {
  try {
    RbV3Runner runner(parseOptions(argc, argv));
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "ERROR|" << error.what() << std::endl;
    return 2;
  }
}
