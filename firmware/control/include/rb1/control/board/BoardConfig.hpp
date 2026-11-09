#pragma once

#include <array>
#include <cstdint>

#include "rb1/core/GeneratedGeometry.hpp"
#include "rb1/hal/MotorMapping.hpp"
#include "rb1/control/board/ImuMount.hpp"

namespace rb1::control::board {

inline constexpr int kButtonGpio = 6;
inline constexpr int kLed1Gpio = 39;
inline constexpr int kLed2Gpio = 40;
inline constexpr int kLed3Gpio = 41;
inline constexpr int kSpareGpio = 42;
inline constexpr int kUsbDmGpio = 19;
inline constexpr int kUsbDpGpio = 20;
inline constexpr int kDebugUartTxGpio = 43;
inline constexpr int kDebugUartRxGpio = 44;
inline constexpr int kBootGpio = 0;
// EN is the ESP32-S3 reset input, not a normal GPIO number.
inline constexpr int kI2cSdaGpio = 8;
inline constexpr int kI2cSclGpio = 9;
inline constexpr int kCanTxGpio = 4;
inline constexpr int kCanRxGpio = 5;
// Each wheel has its own half-duplex single-wire bus (TX and RX on one pad).
inline constexpr int kWheelLeftGpio = 15;    // "UART2" connector, UART_NUM_2, motor id 2
inline constexpr int kWheelRightGpio = 17;   // "UART1" connector, UART_NUM_1, motor id 1
// These PCB RX pads are tied to the corresponding DATA nets. The working
// EC4012 bridge leaves them input-floating and receives on GPIO15/17 instead.
inline constexpr int kWheelLeftUnusedRxGpio = 16;
inline constexpr int kWheelRightUnusedRxGpio = 18;
inline constexpr int kImuCsGpio = 10;
inline constexpr int kImuMosiGpio = 11;
inline constexpr int kImuSclkGpio = 12;
inline constexpr int kImuMisoGpio = 13;
inline constexpr int kImuIntGpio = 14;
inline constexpr std::uint32_t kCanBitrate = 500'000U;
inline constexpr std::uint32_t kMotorUartBaud = 1'000'000U;
// Control loop period: 200 Hz. The controller works in time (in.dt_s, LQR sampling_period_s); in the RB MuJoCo model
// the stand-up passes the same scenarios at 200 Hz as at 500 Hz. The leg drivers close their own position loops, so
// only the target / vel_ff frame rate drops.
inline constexpr std::uint32_t kControlPeriodUs = 5'000U;
inline constexpr std::uint32_t kFeedbackMaxAgeUs = 20'000U;
// Wheel reply age the controller tolerates before it declares the sensors lost. The wheel bus is single-wire
// half-duplex: when the robot is carried or touched its gaps reach 9-27 ms and the 20 ms limit faulted 7 of 23 starts
// at Ready. Ready / Locking only hold the wheels still, so they tolerate a long outage; Balancing acts on the wheel
// speed, so it tolerates less.
inline constexpr std::uint32_t kWheelHoldMaxAgeUs = 100'000U;
inline constexpr std::uint32_t kWheelBalanceMaxAgeUs = 40'000U;
inline constexpr std::uint32_t kCanProbePeriodUs = 10'000U;
inline constexpr std::uint32_t kWheelFeedbackPeriodUs = 2'000U;
inline constexpr bool kWheelDirectionsVerified =
    rb1::hal::motor_mapping::kWheelDirectionsVerified;
inline constexpr bool kWheelSpeedScaleVerified =
    rb1::hal::motor_mapping::kWheelSpeedScaleVerified;
// False until the IMU axis map is checked on the RB hardware. A wrong map closes a positive-feedback loop and the
// first balance runs away.
inline constexpr bool kImuFrameVerified = false;
// MPU6500 axes as mounted on the PCB, mapped to the body frame before the estimator (see ImuMount.hpp).
// RB v1.3 (picked on the CAD, not yet verified on the robot): chip X -> +X, Y -> +Y, Z -> +Z, i.e. no remap.
inline constexpr imu_mount::AxisMap kImuAxisMap = imu_mount::kIdentity;
static_assert(imu_mount::isRotation(kImuAxisMap), "IMU axis map must be a rotation");
// Hardware finding: a task-space Kp ~40 N.m/rad on the real 8:1 GDS68 joints chattered
// ("buzzing", accelerometer saturating at 2 g); the bench jog ran smoothly at Kp 8 / Kd 0.6.
// Cap on the joint Kp (N.m/rad); the hip reached ~43 near the straight (hanging) leg and the
// legs buzzed while folding from there.
inline constexpr float kLegMaxKpNmPerRad = 12.0F;
// GDS68 current limit sent with Set_Limits (0x00F) on every enable edge. 8 A ~ 3.8 N.m at the output, below the 10.5 A
// rating. The same day the GDS68 ignored an MIT torque feed-forward of 2.6 N.m (it did not lift a 0.25 N.m shank) while
// the Kp spring worked.
inline constexpr float kLegCurrentLimitA = 8.0F;
// Rotor turns/s (8:1): 20 = 15.7 rad/s at the joint, far above leg motion.
inline constexpr float kLegVelLimitTurnsS = 20.0F;
// Wheel/leg kinematic acceleration compensation in the IMU pitch estimator. Off: it assumes the wheels roll the body
// along. With the body held (or the wheels slipping) wheel spin became a fake body acceleration and a fake +-0.3 rad
// pitch while accelerometer and gyro both read upright, and the wheels chased it into the runaway guard.
inline constexpr bool kImuWheelKinematicCompensation = false;
// Knee angle at the extension stop. CAD straight leg = top of the native
// knee envelope; the physical stop has not been measured against it.
inline constexpr float kKneeExtensionStopRad = rb1::core::geometry::native_knee_max_rad;
// Joints must be this still to home, and the zero is dropped when a leg's
// MIT feedback is silent for kLegZeroLossUs (a driver reboot restarts its
// position from zero).
inline constexpr float kHomingMaxJointSpeedRadS = 0.10F;
inline constexpr std::uint32_t kLegZeroLossUs = 300'000U;
// A leg silent longer than kLegZeroLossUs keeps its zero unless, when it
// talks again, it reports ~0 after a clearly non-zero position (the driver
// rebooted and restarted counting), or it stays silent past kLegZeroForgetUs.
// A CAN bus-off recovery (~1 s, drivers still running) no longer forces a
// re-home.
inline constexpr std::uint32_t kLegZeroForgetUs = 10'000'000U;
inline constexpr float kLegRebootZeroRad = 0.01F;
inline constexpr float kLegRebootMinTravelRad = 0.05F;
// GDS68 heartbeats arrive every 100 ms. After an enable edge the drivers must
// report CLOSED_LOOP within kClosedLoopGraceUs, or the controller disables:
// MIT replies keep flowing from a driver that refused closed loop.
inline constexpr std::uint32_t kHeartbeatMaxAgeUs = 350'000U;
inline constexpr std::uint32_t kClosedLoopGraceUs = 500'000U;
// Bench option: open TWAI listen-only so leg feedback can be read while the
// CAN TX path is being debugged. Nothing is transmitted or
// acknowledged on CAN, and enable stays refused while this is true.
inline constexpr bool kCanListenOnly = false;
// Bench option: TWAI in normal mode, but every firmware CAN transmission is
// suppressed. Instead one single-shot frame with an unassigned ID (node 62,
// cmd 0, no GDS68 addresses it) is sent per second, so a TX failure can be
// separated from the firmware's traffic pattern. Enable stays refused.
inline constexpr bool kCanTxSingleFrameTest = false;
inline constexpr std::uint16_t kCanTxTestId = 0x7C0U;
inline constexpr std::uint32_t kCanTxTestPeriodUs = 1'000'000U;
// After a bus-off, wait this long before starting TWAI recovery so a broken
// TX path does not keep disturbing the bus. Short: the enabled legs ride
// through an outage up to kLegRideThroughUs (the GDS68 hold their last MIT
// set-point meanwhile), and a 1 s delay turned every bus-off into a drop.
inline constexpr std::uint32_t kCanBusOffRecoveryDelayUs = 20'000U;
inline constexpr std::uint32_t kLegRideThroughUs = 250'000U;
// Diagnostics: one GDS68 DC-bus voltage request (0x017) per this period,
// round robin; SENS prints the lowest voltage seen per report.
inline constexpr std::uint32_t kBusVoltagePollUs = 50'000U;

struct MotorIds {
  std::array<rb1::hal::motor_mapping::LegCanBinding, 4> leg_can =
      rb1::hal::motor_mapping::kVerifiedLegCanBindings;
  std::array<rb1::hal::motor_mapping::WheelUartBinding, 2> wheel_uart =
      rb1::hal::motor_mapping::kWheelUartBindings;
};

}  // namespace rb1::control::board
