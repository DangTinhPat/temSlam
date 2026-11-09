#pragma once

#include "rb1/core/LqrBalancer.hpp"

namespace rb1::core {

// Bộ hệ số LQR đã tinh chỉnh cho robot RB. RobotController dùng làm điểm xuất phát rồi ghi đè các tham số riêng.
[[nodiscard]] LqrBalancer::Config tunedLqrConfig();

}  // namespace rb1::core
