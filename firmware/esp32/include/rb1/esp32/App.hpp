#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>

#include <Arduino.h>

#include "rb1/core/ImuStateEstimator.hpp"
#include "rb1/core/Types.hpp"
#include "rb1/control/board/BoardConfig.hpp"
#include "rb1/control/comm/BusTransport.hpp"
#include "rb1/control/comm/MotorFeedback.hpp"
#include "rb1/control/motion/MotionApi.hpp"
#include "rb1/control/legs/LegLock.hpp"
#include "rb1/control/RobotController.hpp"
#include "rb1/esp32/Esp32Bus.hpp"
#include "rb1/esp32/Mpu6500Spi.hpp"
#include "rb1/esp32/OutSink.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

namespace rb1::esp32 {

// Firmware v3: two-wheel inverted pendulum. The legs are arranged before the motor supply is switched on and locked at
// that power-up pose (encoder 0 of each GDS68, driver position mode); only the wheels balance.
class App {
 public:
  App() noexcept;
  bool begin() noexcept;
  void loop() noexcept;

 private:
  // FreeRTOS split: a control task (core 1, above the Arduino loop) runs
  // every 2 ms - commands, BLE, buses, IMU, controller, telemetry snapshots -
  // and the Arduino loop only moves serial bytes (input lines to a queue,
  // deferred numeric formatting and output from OutSink to the UART).
  // All controller state lives in the
  // control task; the loop touches none of it. If the task cannot be made,
  // the old single loop runs everything (rtos_ false).
  static void controlTaskEntry(void* arg) noexcept;
  // Serial in/out on core 0: on core 1 under the control task it starved whenever a tick ran past 2 ms (LQR during a
  // height ramp: ~2.8 ms), the GUI heartbeat stopped arriving and the 0.5 s session lease disabled the robot right
  // after engage.
  static void commsTaskEntry(void* arg) noexcept;
  TaskHandle_t comms_task_ = nullptr;
  // Wi-Fi access point + one TCP client (RB1_WIFI): the same console as the UART, on core 0.
  static void netTaskEntry(void* arg) noexcept;
  TaskHandle_t net_task_ = nullptr;
  StreamBufferHandle_t net_sb_ = nullptr;
  std::atomic<std::uint32_t> net_gap_max_ms_{0U}, net_eagain_{0U};  // Wi-Fi task: longest loop gap, full-send count
  std::atomic<int> net_last_send_errno_{0};
  std::atomic<std::uint32_t> net_send_closes_{0U}, net_stall_closes_{0U};
  std::atomic<std::uint32_t> net_rx_ms_{0U};  // millis() of the last byte from the Wi-Fi client
  void netTask() noexcept;
  void step(bool task) noexcept;
  void controlPart(std::uint64_t now_us) noexcept;
  struct LineItem { char text[96]; };
  OutSink out_{};
  TaskHandle_t control_task_ = nullptr;
  QueueHandle_t line_q_ = nullptr;
  bool rtos_ = false;
  std::uint32_t lines_dropped_ = 0U;
  void pollSerial(std::uint64_t now_us) noexcept;
  void handleLine(const char* line, std::uint64_t now_us) noexcept;
  void drainFeedback(std::uint64_t now_us) noexcept;
  void updateImu(std::uint64_t now_us) noexcept;
  void controlTick(std::uint64_t now_us) noexcept;
  void driveWheels(bool enabled, const std::array<float, 2>& cmd_rad_s,
                   std::uint64_t now_us) noexcept;
  void report(std::uint64_t now_us) noexcept;
  void printTunables() noexcept;
  bool tunables_pending_ = false;

  [[nodiscard]] bool imuFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] bool legsFresh(std::uint64_t now_us) const noexcept;
  [[nodiscard]] float legJointRad(std::size_t leg) const noexcept;
  // Driver velocity estimate (0x009) of a LegLock joint [rad/s], driver frame.
  [[nodiscard]] float legJointRate(std::size_t leg) const noexcept;
  // joint angle (driver frame, rad) the lock is holding this leg at.
  [[nodiscard]] float legTargetRad(std::size_t leg) const noexcept;
  [[nodiscard]] float legChainRad() const noexcept;
  // Encoder-zero poses in controller coordinates (hip measured from LegHeight::hangHipCad), generated with the
  // geometry by tools/codegen/generate_geometry.py. Measured joints get this offset added and every leg
  // target has it subtracted.
  // Lying start (the RB default): the motor supply is switched on with the legs in the homing pose, knee folded to
  // its stop and hip at its positive mechanical stop (+135 deg), so the driver zero is that pose. STANDUP works from here.
  static constexpr float kHomeHipRad = rb1::core::geometry::home_hip_rad;
  static constexpr float kHomeKneeRad = rb1::core::geometry::home_knee_rad;
  // Straight-leg start (SET lying 0): body on the stand, legs extended, supply switched on in that pose.
  // RB: knee 110 deg soft limit, axle 184.8 mm straight below the hip. UNVERIFIED on the robot.
  static constexpr float kStraightHipRad = rb1::core::geometry::straight_hip_rad;
  static constexpr float kStraightKneeRad = rb1::core::geometry::straight_knee_rad;
  bool lying_start_ = true;   // RB homes folded (see above); SET lying 0 selects the straight-leg start
  bool standup_pending_ = false;  // STANDUP request (SET lying 1, SET standup 1)
  std::uint64_t rec_period_us_ = 4'000U;
  // Stand-up POS velocity feed-forward: filtered rate of the commanded hip/knee path [core rad/s].
  float ff_hip_prev_ = 0.0F, ff_knee_prev_ = 0.0F, ff_hip_rate_ = 0.0F, ff_knee_rate_ = 0.0F;
  std::uint64_t ff_prev_us_ = 0U;
  [[nodiscard]] float homeHip() const noexcept { return lying_start_ ? kHomeHipRad : kStraightHipRad; }
  [[nodiscard]] float homeKnee() const noexcept { return lying_start_ ? kHomeKneeRad : kStraightKneeRad; }
  [[nodiscard]] const char* legAxisProblem(std::uint64_t now_us, bool need_closed_loop) const noexcept;

  Mpu6500Spi mpu_{};
  rb1::core::ImuStateEstimator estimator_{};
  Esp32TwaiPort can_{};
  Esp32UartPort uart1_{2, rb1::control::board::kWheelLeftGpio,
                       rb1::control::board::kWheelLeftUnusedRxGpio, 2U, "left"};
  Esp32UartPort uart2_{1, rb1::control::board::kWheelRightGpio,
                       rb1::control::board::kWheelRightUnusedRxGpio, 1U, "right"};
  rb1::control::CallbackBusTransport transport_;
  rb1::control::MotorFeedback feedback_{};
  rb1::control::LegLock legs_{};
  rb1::control::RobotController ctrl_;

  Mpu6500Spi::Sample chip_{};
  Mpu6500Spi::Sample body_{};
  rb1::core::ImuStateEstimator::Output imu_{};
  rb1::core::RobotState state_{};

  std::uint64_t next_control_us_ = 0U;
  // Loop timing (SENS reports and resets them): real tick period fed to the
  // controller, worst tick execution time, ticks started >1 period late.
  std::uint64_t last_tick_us_ = 0U;
  float tick_dt_s_ = 0.002F;
  // Finite moves (MOVE / TURN): calibration of the odometry and the gyro against the floor, and the response lag
  // behind the speed plan that the braking point allows for. SET modist / moyaw / molag / moylag.
  float motion_dist_scale_ = 1.0F;
  float motion_yaw_scale_ = 1.0F;
  float motion_lag_s_ = 0.12F;
  float motion_yaw_lag_s_ = 0.07F;
  std::uint32_t tick_exec_max_us_ = 0U;
  std::uint32_t tick_imu_max_us_ = 0U;
  std::uint32_t tick_ctrl_max_us_ = 0U;
  std::uint32_t tick_period_max_us_ = 0U, tick_overrun_ = 0U;
  std::uint32_t tick_report_max_us_ = 0U;
  // Per-section maxima of the control tick [us] (SENS prof=[update legs
  // wheels trace]) and the mean controller time (upd_avg).
  std::array<std::uint32_t, 4> prof_max_{};
  std::uint32_t prof_upd_sum_ = 0U, prof_upd_n_ = 0U;
  void prof(int i, std::uint64_t us) noexcept {
    prof_max_[static_cast<std::size_t>(i)] =
        std::max(prof_max_[static_cast<std::size_t>(i)], static_cast<std::uint32_t>(us));
  }
  std::uint32_t pre_serial_max_us_ = 0U;  // per loop() pass, not only control ticks
  std::uint32_t pre_ble_max_us_ = 0U;
  std::uint32_t pre_bus_max_us_ = 0U;
  std::uint32_t tick_late_ = 0U;
  float hip_vel_sq_ = 0.0F;  // hip velocity estimate noise, per SENS report
  std::uint32_t hip_vel_n_ = 0U;
  std::uint64_t last_imu_us_ = 0U;
  std::uint64_t last_report_us_ = 0U;
  std::uint64_t next_trace_us_ = 0U;
  std::uint64_t last_rx_us_ = 0U;
  std::uint64_t state_since_us_ = 0U;
  std::uint64_t engage_us_ = 0U;
  std::uint64_t wheels_on_since_us_ = 0U;
  bool wheels_on_ = false;
  bool start_pending_ = false;
  bool disable_pending_ = false;
  bool go_pending_ = false;
  bool go_after_ready_ = false;
  float height_request_m_ = 0.0F;  // SET height; 0 = no request
  float stand_height_m_ = 0.0F;    // SET standheight: hip-axle height after a stand-up [m]; 0 = full supported height
  // HMOVE hold-to-run: +1 raise / -1 lower while the GUI keeps repeating it;
  // released (0) or silent for kHeightHoldUs -> freeze at the current height.
  int hmove_dir_ = 0;
  std::uint64_t hmove_last_us_ = 0U;
  void freezeHeight() noexcept;
  void setHeightMove(int dir, std::uint64_t now_us) noexcept;

  // BLE (Dabble GamePad): joystick drives, Start = START, Select = GO,
  // Triangle / Cross held = raise / lower the height. USB serial stays on.
  void pollBle(std::uint64_t now_us) noexcept;
  std::uint64_t next_ble_poll_us_ = 0U;
  std::uint64_t last_phone_rx_us_ = 0U;
  std::uint64_t ble_advertise_at_us_ = 0U;
  bool ble_link_up_ = false;
  bool ble_inputs_armed_ = false;
  bool ble_start_prev_ = false;
  bool ble_select_prev_ = false;
  int ble_height_dir_ = 0;
  bool ble_driving_ = false;  // GO from Idle/Fault: start, then engage at Ready
  bool estop_ = false;
  // TILTTEST: wheels follow pitch only (no velocity loop), for checking the
  // tilt->wheel direction with the robot lifted.
  bool tilt_test_ = false;
  float tilt_ref_rad_ = 0.0F;
  // Joint tests and recordings, one RAM recorder (250 Hz, <= 8 s) dumped
  // afterwards as "RC" lines (tools/fit_leg_servo.py):
  //  - HIPTEST / KNEETEST [amp]: robot on the stand, legs locked, wheels off;
  //    steps + 1/2/4/8 Hz sines on both hips / knees (GDS68 lag, no load);
  //  - KNEETEST FLOOR [amp]: balancing on the floor, knee sines at 0.5-3 Hz
  //    through the height pose (knee under the body's weight);
  //  - REC [s]: record only (pushes, bumps, turns).
  enum class RecKind : std::uint8_t { None, HipStand, KneeStand, KneeFloor, Free };
  RecKind rec_kind_ = RecKind::None;
  // Start a free 8 s / 250 Hz REC at every stand-up handoff into Balancing, to identify the wheel
  // command-to-speed response (wL, wR, wcmd) in the real operating regime. SET recbal 0/1.
  bool rec_balance_auto_ = true;
  bool rec_active_ = false;
  bool rec_dumping_ = false;
  // Blackbox: from STANDUP the last ~12 s (8 ms samples) are kept in g_rec as a ring, frozen and dumped as RC lines
  // when the robot leaves StandUp/Balancing (fault, DISABLE, lease). It survives a Wi-Fi blackout: DUMP (or a
  // client joining) sends it again.
  bool rec_ring_ = false;
  std::size_t rec_ring_head_ = 0U;
  std::atomic<bool> redump_request_{false};
  void startBlackbox(std::uint64_t now_us) noexcept;
  void startDump(bool aborted) noexcept;
  bool rec_aborted_ = false;
  std::uint64_t rec_start_us_ = 0U;
  std::uint64_t rec_next_us_ = 0U;
  float rec_duration_s_ = 0.0F;
  float rec_amp_ = 0.0F;
  float rec_centre_ = 0.0F;
  float rec_cmd_ = 0.0F;  // current excitation (rad)
  std::uint32_t rec_resend_us_ = 4'000U;
  std::size_t rec_n_ = 0U;
  std::size_t rec_dump_i_ = 0U;
  [[nodiscard]] bool recStand() const noexcept {
    return rec_kind_ == RecKind::HipStand || rec_kind_ == RecKind::KneeStand;
  }
  void startRecording(RecKind kind, float amp, float duration_s, std::uint64_t now_us) noexcept;
  void stopRecording(bool aborted) noexcept;
  [[nodiscard]] float recExcitation(std::uint64_t now_us) const noexcept;
  void recordSample(std::uint64_t now_us, float hip_cmd, float knee_cmd,
                    float wheel_cmd) noexcept;
  void dumpRecording() noexcept;
  // 0.5 s average of pitch and of its spread, for measuring the balance angle
  // (operator holds the robot at its tipping point and sends BALANCE).
  float pitch_avg_ = 0.0F;
  float pitch_spread_ = 0.0F;
  bool can_ready_ = false;
  bool uart_ready_ = false;
  bool imu_ready_ = false;
  // Finite moves (rb1::control::MotionApi): MOVE <m> [v], TURN <rad> [w], MSTAT on the console.
  rb1::control::MotionApi motion_{};
  void reportMotion(const rb1::control::MotionApi::Event& e) noexcept;
  void pumpMotionEvents() noexcept;
  std::array<float, rb1::control::LegLock::kLegs> leg_zero_turns_{};  // ZERO: driver turns taken as the encoder zero
  std::uint64_t drive_serial_us_ = 0U;   // last serial DRIVE; one that is not refreshed for 1 s is dropped
  bool drive_serial_active_ = false;
  float drive_forward_ = 0.0F;
  float drive_yaw_ = 0.0F;
  int last_state_ = -1;
  int last_ready_phase_ = -1;
  char line_[96]{};
  std::size_t line_length_ = 0U;
  std::array<float, 2> last_wheel_cmd_{};
};

}  // namespace rb1::esp32
