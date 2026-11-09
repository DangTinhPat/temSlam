// PCB IMU mounting. RB v1.3: no remap (chip X -> +X, Y -> +Y, Z -> +Z), picked on the CAD of the body PCB
// and confirmed by the user, not yet verified on the robot. A second mounting (chip Y forward, chip X right) stays covered below.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "rb1/control/board/BoardConfig.hpp"

int main() {
  using namespace rb1::control;
  static_assert(imu_mount::isRotation(board::kImuAxisMap));
  static_assert(imu_mount::isRotation(imu_mount::kChipYForwardXRight));
  // A left-handed reading of the chip (X left and Y forward, Z up) is rejected.
  constexpr imu_mount::AxisMap mirror{{1U, 0U, 2U}, {1.0F, 1.0F, 1.0F}};
  static_assert(!imu_mount::isRotation(mirror));

  const float g = 9.80665F;
  const float th = 0.3F;
  // Nose-down by 0.3 rad about body +Y: body accel = (-g sin, 0, g cos).

  // Current RB map: the chip frame is the body frame.
  const auto& map = board::kImuAxisMap;
  assert(map.chip_axis[0] == 0U && map.chip_axis[1] == 1U && map.chip_axis[2] == 2U);
  assert(map.sign[0] == 1.0F && map.sign[1] == 1.0F && map.sign[2] == 1.0F);
  const auto body = imu_mount::toBody(map, {-g * std::sin(th), 0.0F, g * std::cos(th)});
  const float pitch = std::atan2(-body[0], body[2]);
  assert(std::abs(pitch - th) < 1.0e-5F);
  assert(std::abs(body[1]) < 1.0e-6F);
  // Nose-down rotation rate about body +Y is chip +Y; yaw about chip Z; roll about chip X.
  const auto rate = imu_mount::toBody(map, {0.0F, 0.5F, 0.0F});
  assert(rate[1] == 0.5F && rate[0] == 0.0F && rate[2] == 0.0F);
  assert(imu_mount::toBody(map, {0.0F, 0.0F, 0.7F})[2] == 0.7F);
  assert(imu_mount::toBody(map, {0.2F, 0.0F, 0.0F})[0] == 0.2F);

  // Alternative mounting (chip Y forward, chip X right, chip Z up).
  const auto& old_map = imu_mount::kChipYForwardXRight;
  const auto old_body = imu_mount::toBody(old_map, {0.0F, -g * std::sin(th), g * std::cos(th)});
  assert(std::abs(std::atan2(-old_body[0], old_body[2]) - th) < 1.0e-5F);
  const auto old_rate = imu_mount::toBody(old_map, {-0.5F, 0.0F, 0.0F});
  assert(old_rate[1] == 0.5F && old_rate[0] == 0.0F && old_rate[2] == 0.0F);
  std::puts("imu_mount_tests: all checks passed");
  return 0;
}
