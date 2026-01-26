#ifndef TELEOP_MODES_PHYSICS_MODE_HPP
#define TELEOP_MODES_PHYSICS_MODE_HPP

#include "core/drive_mode.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace autoware::manual_control {

// ==========================================
// Physics Mode: Inertia-based control
// ==========================================
class PhysicsDriveMode : public DriveMode {
public:
  void onEnter(const VehicleState &current_state) override {
    // Always start with absolute speed concept
    current_vel_ = std::abs(current_state.velocity); // seamless handover
    current_steer_ = current_state.steer_angle;
    current_accel_ = 0.0f;
    current_accel_rate_ = 0.0f;
    last_gear_ = current_state.gear;
  }

  ControlCommand update(float dt, const InputState &input,
                        const VehicleState &vehicle_state) override {

    // Safety: Reset State on Gear Change
    if (vehicle_state.gear != last_gear_) {
      current_vel_ = 0.0f;
      current_accel_ = 0.0f;
      current_accel_rate_ = 0.0f;
      last_gear_ = vehicle_state.gear;
    }

    // 1. Steering Physics (Attack/Decay)
    if (input.steer_dir != 0) {
      current_steer_ += input.steer_dir * params_.steer_attack * dt;
    } else {
      // Auto-center
      if (current_steer_ > params_.steer_deadzone) {
        current_steer_ -= params_.steer_decay * dt;
        if (current_steer_ < 0)
          current_steer_ = 0;
      } else if (current_steer_ < -params_.steer_deadzone) {
        current_steer_ += params_.steer_decay * dt;
        if (current_steer_ > 0)
          current_steer_ = 0;
      } else {
        current_steer_ = 0;
      }
    }
    current_steer_ =
        std::clamp(current_steer_, -params_.max_steer, params_.max_steer);

    // 2. Velocity Physics (Magnitude Based)
    float real_speed_mag = std::abs(vehicle_state.velocity);

    // Acceleration Ramp
    if (input.throttle > 0) {
      current_accel_rate_ = std::min(current_accel_rate_ + 1.0f, 9.0f);
    } else {
      current_accel_rate_ = 0.0f;
      // Anti-ghosting: Check if Target (Magnitude) >> Real (Magnitude)
      // If we are commanding 5.0, but real is 0.0, and throttle released ->
      // snap
      if (current_vel_ > real_speed_mag + 2.0f) {
        current_vel_ = real_speed_mag + 0.5f;
      }
    }

    // Apply Acceleration (Increase Magnitude)
    current_vel_ += (input.throttle > 0 ? current_accel_rate_ : 0.0f) * dt;

    // Apply Braking (Decrease Magnitude)
    if (input.brake > 0) {
      // Linear decrease
      if (current_vel_ > 0) {
        current_vel_ -= params_.brake_rate * dt;
        if (current_vel_ < 0)
          current_vel_ = 0;
      }
    }

    // Apply Friction (Decrease Magnitude)
    float friction = params_.friction_rate;
    if (vehicle_state.gear == Gear::PARK)
      friction = 10.0f;

    if (current_vel_ > 0) {
      current_vel_ -= friction * dt;
      if (current_vel_ < 0)
        current_vel_ = 0;
    }

    current_vel_ = std::clamp(current_vel_, 0.0f, params_.max_speed);

    // Calculate Acceleration Command (Derivative)
    // Calculated based on speed magnitude to prevent sign flip instability
    // during direction changes
    float accel_cmd = 0.0f;
    if (dt > 1e-4) {
      accel_cmd = (current_vel_ - real_speed_mag) / dt;
    }

    ControlCommand cmd;
    cmd.velocity = current_vel_;
    cmd.acceleration = accel_cmd;
    cmd.steer_angle = current_steer_;

    return cmd;
  }

  std::string getName() const override { return "PHYSICS"; }

  std::string getStatusString() const override {
    char buf[64];
    snprintf(buf, sizeof(buf), "Acc: %.2f m/s2", current_accel_);
    return std::string(buf);
  }

private:
  // We track Magnitude (Speed) here, assumed positive
  float current_vel_ = 0.0f;
  float current_steer_ = 0.0f;
  float current_accel_ = 0.0f;
  float current_accel_rate_ = 0.0f;
  Gear last_gear_ = Gear::PARK;

  struct Params {
    float max_speed = 27.78f;
    float max_steer = 0.6f;
    float steer_attack = 0.8f;
    float steer_decay = 1.0f;
    float steer_deadzone = 0.01f;
    float brake_rate = 10.0f;
    float friction_rate = 3.0f;
  } params_;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODES_PHYSICS_MODE_HPP
