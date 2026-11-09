#pragma once
#include "rb1/control/RobotController.hpp"

namespace rb1::control {
inline void applyAddWifiBalanceProfile(RobotController::Config& cfg, bool enabled) noexcept {
  cfg.tier.lean_share = enabled ? 1.0F : 0.85F;
  cfg.tier.disturbance_share = enabled ? 0.60F : 0.45F;
}
inline bool isAddWifiBalanceProfile(const RobotController::Config& cfg) noexcept {
  return cfg.tier.lean_share == 1.0F && cfg.tier.disturbance_share == 0.60F;
}
}  // namespace rb1::control
