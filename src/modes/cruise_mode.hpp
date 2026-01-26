#ifndef TELEOP_MODES_CRUISE_MODE_HPP
#define TELEOP_MODES_CRUISE_MODE_HPP

#include "core/drive_mode.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace autoware::manual_control {

// ==========================================
// Cruise Mode: Setpoint based with Steering Lock
// ==========================================
class CruiseDriveMode : public DriveMode {
public:
  void onEnter(const VehicleState &current_state) override {
    target_speed_ = std::abs(current_state.velocity);

    // Safety clamp on entry
    if (target_speed_ > MAX_SPEED)
      target_speed_ = 0.0f;

    // Reset if entering in Reverse (Safety)
    if (current_state.gear == Gear::REVERSE) {
      target_speed_ = 0.0f;
    }

    current_steer_ = current_state.steer_angle;

    // Reset toggle states
    last_throttle_press_ = false;
    last_brake_press_ = false;

    // Initialize Gear Tracker
    last_gear_ = current_state.gear;
  }

  ControlCommand update(float dt, const InputState &input,
                        const VehicleState &vehicle_state) override {

    // Safety: Reset Setpoint on ANY Gear Change
    if (vehicle_state.gear != last_gear_) {
      target_speed_ = 0.0f;
      last_gear_ = vehicle_state.gear;
    }

    // 1. Steering (Lock Logic - No Auto-Centering)
    if (input.steer_dir != 0) {
      current_steer_ += input.steer_dir * STEER_RATE * dt;
    }
    current_steer_ = std::clamp(current_steer_, -STEER_LIMIT, STEER_LIMIT);

    // 2. Velocity (Setpoint) with Tap/Hold Logic
    if (vehicle_state.gear == Gear::PARK) {
      target_speed_ = 0.0f;
    } else {
      // Throttle Logic
      bool throttle_active = (input.throttle > 0);
      if (throttle_active) {
        if (input.throttle_hold) {
          // Holding: Continuous increase
          target_speed_ += VEL_INC_HOLD * dt;
        } else {
          // Tapping (Rising Edge): Snap to next integer km/h
          if (!last_throttle_press_) {
            float current_kph = target_speed_ * 3.6f;
            float next_kph = std::floor(current_kph) + 1.0f;
            target_speed_ = next_kph / 3.6f;
          }
        }
      }
      last_throttle_press_ = throttle_active;

      // Brake Logic (Decrement)
      bool brake_active = (input.brake > 0);
      if (brake_active) {
        if (input.brake_hold) {
          // Holding: Continuous decrease
          target_speed_ -= VEL_DEC_HOLD * dt;
        } else {
          // Tapping (Rising Edge): Snap to prev integer km/h
          if (!last_brake_press_) {
            float current_kph = target_speed_ * 3.6f;
            float next_kph = std::ceil(current_kph) - 1.0f;
            target_speed_ = next_kph / 3.6f;
          }
        }
      }
      last_brake_press_ = brake_active;
    }

    target_speed_ = std::clamp(target_speed_, 0.0f, MAX_SPEED);

    // P-Control for Accel
    float real_speed = std::abs(vehicle_state.velocity);
    float error = target_speed_ - real_speed;
    float accel_cmd = error * ACCEL_P_GAIN;
    accel_cmd = std::clamp(accel_cmd, MIN_ACCEL, MAX_ACCEL);

    ControlCommand cmd;
    cmd.velocity = target_speed_;
    cmd.acceleration = accel_cmd;
    cmd.steer_angle = current_steer_;
    return cmd;
  }

  std::string getName() const override { return "CRUISE"; }
  std::string getStatusString() const override {
    return ""; // Standard UI shows Set speed
  }

private:
  // Constants
  static constexpr float MAX_SPEED = 27.78f;          // 100 km/h
  static constexpr float STEER_RATE = 0.3f;           // rad/s
  static constexpr float STEER_LIMIT = 0.6f;          // rad (~34 deg)
  static constexpr float VEL_INC_HOLD = 5.0f / 3.6f;  // +5 km/h per sec
  static constexpr float VEL_DEC_HOLD = 10.0f / 3.6f; // -10 km/h per sec
  static constexpr float ACCEL_P_GAIN = 3.0f;
  static constexpr float MAX_ACCEL = 5.0f;
  static constexpr float MIN_ACCEL = -10.0f;

  float target_speed_ = 0.0f;
  float current_steer_ = 0.0f;
  Gear last_gear_ = Gear::PARK;

  // Edge detection for Tap
  bool last_throttle_press_ = false;
  bool last_brake_press_ = false;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODES_CRUISE_MODE_HPP
