#pragma once
// Values carry their source in that file (datasheet PS-MPU-6500A-01, logged sensor data, firmware register setup).
// Motor vibration is NOT measured: the vibration terms are an assumption, white noise proportional to the mean wheel
// speed, for stress tests only.
#include <algorithm>
#include <array>
#include <cmath>
#include <random>

namespace rb1::sim {

class Mpu6500Model {
 public:
  struct Params {
    // gyro (firmware: +-500 dps, 65.5 LSB/dps, DLPF CONFIG=3)
    float gyro_noise_dps = 0.061F;            // measured per sample after the filter
    float gyro_noise_max_dps = 0.08F;         // a unit draws its noise uniformly in [value, max]
    float gyro_turn_on_sigma_dps = 2.5F;      // datasheet +-5 dps as 2 sigma
    float gyro_turn_on_clip_dps = 5.0F;
    float gyro_session_sigma_dps = 0.08F;     // measured power-up spread
    float gyro_walk_min = 4.0e-5F;            // bias random walk [rad/s/sqrt(s)], measured range
    float gyro_walk_max = 1.07e-4F;
    float gyro_dlpf_hz = 41.0F;
    float gyro_full_scale_dps = 500.0F;
    float gyro_lsb_per_dps = 65.5F;
    // accelerometer (firmware: +-2 g, 16384 LSB/g, ACCEL_CONFIG2=3)
    std::array<float, 3> accel_noise_mg{3.0F, 2.2F, 3.8F};   // measured per sample after the filter
    float accel_turn_on_sigma_mg = 30.0F;     // datasheet +-60 mg zero-g tolerance
    float accel_turn_on_clip_mg = 60.0F;
    float accel_dlpf_hz = 44.8F;
    float accel_full_scale_g = 2.0F;
    float accel_lsb_per_g = 16384.0F;
    // both
    float scale_sigma = 0.015F, scale_clip = 0.03F;           // datasheet +-3 %
    float cross_sigma = 0.007F, cross_clip = 0.02F;           // datasheet +-2 %
    // stress knobs (not measured)
    float noise_scale = 1.0F;                 // multiplies the white noise and the bias walk
    float vib_accel_mg_per_rad_s = 0.0F;      // motor vibration: white accel noise per mean |wheel speed|
    float vib_gyro_dps_per_rad_s = 0.0F;
  };

  Mpu6500Model() = default;
  Mpu6500Model(Params params, unsigned unit_seed, unsigned session_seed) { configure(params, unit_seed, session_seed); }

  void configure(Params params, unsigned unit_seed, unsigned session_seed) {
    p_ = params;
    std::mt19937 unit(unit_seed);
    std::normal_distribution<float> n(0.0F, 1.0F);
    std::uniform_real_distribution<float> u(0.0F, 1.0F);
    const auto clipped = [&](float sigma, float clip) { return std::clamp(sigma * n(unit), -clip, clip); };
    for (auto* m : {&gyro_m_, &accel_m_}) {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          (*m)[r][c] = r == c ? 1.0F + clipped(p_.scale_sigma, p_.scale_clip) : clipped(p_.cross_sigma, p_.cross_clip);
    }
    for (int i = 0; i < 3; ++i) {
      gyro_bias_[i] = deg(clipped(p_.gyro_turn_on_sigma_dps, p_.gyro_turn_on_clip_dps));
      accel_bias_[i] = mg(clipped(p_.accel_turn_on_sigma_mg, p_.accel_turn_on_clip_mg));
    }
    gyro_noise_dps_ = p_.gyro_noise_dps + u(unit) * std::max(p_.gyro_noise_max_dps - p_.gyro_noise_dps, 0.0F);
    gyro_walk_ = p_.gyro_walk_min + u(unit) * std::max(p_.gyro_walk_max - p_.gyro_walk_min, 0.0F);
    rng_.seed(session_seed);
    for (int i = 0; i < 3; ++i) gyro_bias_[i] += deg(p_.gyro_session_sigma_dps * normal_(rng_));
    seeded_ = false;
  }

  // true_gyro [rad/s], true_specific_force [m/s^2] in the IMU frame; mean_wheel_speed [rad/s] drives the vibration.
  void step(const std::array<float, 3>& true_gyro, const std::array<float, 3>& true_accel, float dt,
            float mean_wheel_speed, std::array<float, 3>& gyro_out, std::array<float, 3>& accel_out) {
    const float ag = alpha(p_.gyro_dlpf_hz, dt), aa = alpha(p_.accel_dlpf_hz, dt);
    // White noise is added before the filter, scaled so that its filtered spread is the measured per-sample value.
    const float kg = std::sqrt((2.0F - ag) / ag), ka = std::sqrt((2.0F - aa) / aa);
    const float speed = std::abs(mean_wheel_speed);
    const float walk = gyro_walk_ * p_.noise_scale * std::sqrt(std::max(dt, 0.0F));
    for (int i = 0; i < 3; ++i) {
      gyro_bias_[i] += walk * normal_(rng_);
      float g = 0.0F, a = 0.0F;
      for (int c = 0; c < 3; ++c) {
        g += gyro_m_[i][c] * true_gyro[c];
        a += accel_m_[i][c] * true_accel[c];
      }
      const float g_sigma = deg(gyro_noise_dps_ * p_.noise_scale + p_.vib_gyro_dps_per_rad_s * speed);
      const float a_sigma = mg(p_.accel_noise_mg[i] * p_.noise_scale + p_.vib_accel_mg_per_rad_s * speed);
      g += gyro_bias_[i] + kg * g_sigma * normal_(rng_);
      a += accel_bias_[i] + ka * a_sigma * normal_(rng_);
      gyro_lp_[i] = seeded_ ? gyro_lp_[i] + ag * (g - gyro_lp_[i]) : g;
      accel_lp_[i] = seeded_ ? accel_lp_[i] + aa * (a - accel_lp_[i]) : a;
      gyro_out[i] = digitise(gyro_lp_[i], deg(p_.gyro_full_scale_dps), deg(1.0F / p_.gyro_lsb_per_dps));
      accel_out[i] = digitise(accel_lp_[i], p_.accel_full_scale_g * kG, kG / p_.accel_lsb_per_g);
    }
    seeded_ = true;
  }

  [[nodiscard]] std::array<float, 3> gyroBias() const noexcept { return gyro_bias_; }

 private:
  static constexpr float kG = 9.80665F;
  static float deg(float v) { return v * 0.017453292F; }
  static float mg(float v) { return v * 1.0e-3F * kG; }
  static float alpha(float hz, float dt) { return 1.0F - std::exp(-2.0F * 3.14159265F * hz * dt); }
  static float digitise(float x, float full_scale, float lsb) {
    return std::round(std::clamp(x, -full_scale, full_scale) / lsb) * lsb;
  }

  Params p_{};
  std::array<std::array<float, 3>, 3> gyro_m_{}, accel_m_{};
  std::array<float, 3> gyro_bias_{}, accel_bias_{}, gyro_lp_{}, accel_lp_{};
  float gyro_noise_dps_ = 0.061F, gyro_walk_ = 4.0e-5F;
  bool seeded_ = false;
  std::mt19937 rng_{1U};
  std::normal_distribution<float> normal_{0.0F, 1.0F};
};

}  // namespace rb1::sim
