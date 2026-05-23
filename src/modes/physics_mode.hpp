#ifndef TELEOP_MODES_PHYSICS_MODE_HPP
#define TELEOP_MODES_PHYSICS_MODE_HPP

#include "core/drive_mode.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace autoware::manual_control {

class PhysicsDriveMode : public DriveMode {
public:
  void onEnter(const VehicleState &current_state) override {
    current_vel_ = std::abs(current_state.velocity);
    current_steer_ = current_state.steer_angle;
    current_accel_ = 0.0f;
    current_accel_rate_ = 0.0f;
    last_gear_ = current_state.gear;
  }

  ControlCommand update(float dt, const InputState &input,
                        const VehicleState &vehicle_state) override {

    // Debounced gear-change reset; raw transitions flicker.
    if (vehicle_state.gear != last_gear_) {
      gear_debounce_count_++;
      if (gear_debounce_count_ >= GEAR_DEBOUNCE_TICKS) {
        current_vel_ = 0.0f;
        current_accel_ = 0.0f;
        current_accel_rate_ = 0.0f;
        last_gear_ = vehicle_state.gear;
        gear_debounce_count_ = 0;
      }
    } else {
      gear_debounce_count_ = 0;
    }

    if (input.steer_dir != 0) {
      current_steer_ += input.steer_dir * params_.steer_attack * dt;
    } else {
      if (std::abs(current_steer_) > params_.steer_deadzone) {
        float decay = params_.steer_decay * dt;
        if (current_steer_ > 0) {
          current_steer_ = std::max(0.0f, current_steer_ - decay);
        } else {
          current_steer_ = std::min(0.0f, current_steer_ + decay);
        }
      } else {
        current_steer_ = 0;
      }
    }
    current_steer_ =
        std::clamp(current_steer_, -params_.max_steer, params_.max_steer);

    float real_speed_mag = std::abs(vehicle_state.velocity);

    // Anchor target to actual speed during coast so target leads downward;
    // min() prevents target from re-inflating after inertia overshoot.
    if (input.throttle == 0.0f && input.brake == 0.0f) {
      float anchor = std::min(current_vel_, real_speed_mag);
      float decayed = anchor - params_.coast_decel * dt;
      current_vel_ = std::max(0.0f, decayed);
    }

    if (input.throttle > 0) {
      current_accel_rate_ = input.throttle * 4.0f;
      current_vel_ += current_accel_rate_ * dt;
    } else if (input.brake > 0) {
      current_vel_ -= input.brake * params_.brake_rate * dt;
      if (current_vel_ < 0) current_vel_ = 0;
    }

    // Anti-windup against delayed vehicle_state (real_speed=0 stale → runaway).
    current_vel_ = std::min(current_vel_, real_speed_mag + params_.max_vel_offset);
    current_vel_ = std::clamp(current_vel_, 0.0f, params_.max_speed);

    ControlCommand cmd;
    // Autoware 1.5+: velocity must be positive magnitude; sign comes from GearCommand.
    cmd.velocity = current_vel_;

    float accel_cmd_mag = 0.0f;
    const float vel_error = current_vel_ - real_speed_mag;
    if (input.throttle > 0) {
      float base_accel = input.throttle * params_.accel_max;
      float ratio = std::clamp(vel_error / params_.accel_taper_range, 0.0f, 1.0f);
      accel_cmd_mag = base_accel * ratio;
      // Minimum accel so the vehicle starts moving from standstill.
      if (real_speed_mag < 0.5f && input.throttle > 0.5f)
        accel_cmd_mag = std::max(accel_cmd_mag, params_.accel_min_start);
    } else if (input.brake > 0) {
      accel_cmd_mag = -input.brake * params_.brake_accel;
    } else {
      accel_cmd_mag = (real_speed_mag > 0.1f) ? -params_.coast_decel * 1.5f : 0.0f;
    }

    // Overshoot brake: real exceeds target → corrective decel regardless of input.
    const float overshoot = real_speed_mag - current_vel_;
    if (overshoot > 0.3f) {  // deadzone to avoid oscillation
      float correction = -overshoot * params_.overshoot_p_gain;
      accel_cmd_mag = std::min(accel_cmd_mag, correction);
    }

    cmd.acceleration = accel_cmd_mag;
    cmd.steer_angle = current_steer_;

    current_accel_ = accel_cmd_mag;

    return cmd;
  }

  std::string getName() const override { return "PHYSICS"; }

  std::string getStatusString() const override {
    char buf[64];
    snprintf(buf, sizeof(buf), "Acc: %.2f m/s2", current_accel_);
    return std::string(buf);
  }

private:
  // Magnitude only; sign comes from gear.
  float current_vel_ = 0.0f;
  float current_steer_ = 0.0f;
  float current_accel_ = 0.0f;
  float current_accel_rate_ = 0.0f;
  Gear last_gear_ = Gear::PARK;
  int gear_debounce_count_ = 0;
  static constexpr int GEAR_DEBOUNCE_TICKS = 6;  // ~100ms at 60Hz

  struct Params {
    float max_speed = 27.78f;
    float max_steer = 0.6f;
    float steer_attack = 0.8f;
    float steer_decay = 1.0f;
    float steer_deadzone = 0.01f;
    float brake_rate = 10.0f;
    float brake_accel = 5.0f;     // m/s^2 brake command magnitude
    float coast_decel = 2.0f;     // m/s^2 engine braking when coasting
    float max_vel_offset = 3.0f;  // m/s — anti-windup clamp
    float accel_max = 3.5f;       // m/s^2 max acceleration command
    float accel_taper_range = 3.0f; // m/s — P-control range: full accel at this error, 0 at error=0
    float accel_min_start = 1.0f; // m/s^2 minimum accel from standstill
    float overshoot_p_gain = 3.0f; // P-gain for active overshoot correction
  } params_;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODES_PHYSICS_MODE_HPP
