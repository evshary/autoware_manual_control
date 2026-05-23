#ifndef TELEOP_MODES_PHYSICS_MODE_HPP
#define TELEOP_MODES_PHYSICS_MODE_HPP

#include "core/drive_mode.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace autoware::manual_control {

// Direct mapping: input -> desired acceleration -> integrate -> velocity
// setpoint. The vehicle controller downstream owns velocity tracking; we do
// not stack a second loop on top of it.
class PhysicsDriveMode : public DriveMode {
public:
  struct Params {
    float max_speed = 27.78f;      // m/s (100 km/h)
    float max_steer = 0.6f;        // rad
    float steer_rate = 0.8f;       // rad/s while a steer key is held
    float steer_decay = 1.0f;      // rad/s auto-center on release
    float steer_deadzone = 0.01f;  // rad
    float accel_max = 3.5f;        // m/s^2 at full throttle
    float brake_max = 5.0f;        // m/s^2 at full brake
    float coast_decel = 2.0f;      // m/s^2 with no input (engine braking)
  };

  explicit PhysicsDriveMode(const Params &params) : params_(params) {}

  void onEnter(const VehicleState &state) override {
    desired_vel_ = std::abs(state.velocity);
    current_steer_ = state.steer_angle;
    last_gear_ = state.gear;
    status_accel_ = 0.0f;
  }

  ControlCommand update(float dt, const InputState &input,
                        const VehicleState &vehicle_state) override {
    // Gear change wipes the velocity setpoint to avoid carrying speed across
    // a direction reverse.
    if (vehicle_state.gear != last_gear_) {
      desired_vel_ = 0.0f;
      last_gear_ = vehicle_state.gear;
    }

    // Steering: integrate rate to angle; auto-center toward zero on release.
    if (input.steer_dir != 0) {
      current_steer_ += input.steer_dir * params_.steer_rate * dt;
    } else if (std::abs(current_steer_) > params_.steer_deadzone) {
      float step = params_.steer_decay * dt;
      if (current_steer_ > 0) {
        current_steer_ = std::max(0.0f, current_steer_ - step);
      } else {
        current_steer_ = std::min(0.0f, current_steer_ + step);
      }
    } else {
      current_steer_ = 0.0f;
    }
    current_steer_ =
        std::clamp(current_steer_, -params_.max_steer, params_.max_steer);

    // Input -> acceleration intent (linear, immediate).
    float desired_accel;
    if (input.throttle > 0.0f) {
      desired_accel = input.throttle * params_.accel_max;
    } else if (input.brake > 0.0f) {
      desired_accel = -input.brake * params_.brake_max;
    } else {
      desired_accel = -params_.coast_decel;
    }

    // Integrate to velocity setpoint, clamped to [0, max_speed].
    desired_vel_ =
        std::clamp(desired_vel_ + desired_accel * dt, 0.0f, params_.max_speed);

    // Pass the operator's acceleration intent through, except cap positive
    // intent at the upper bound (otherwise the downstream controller pushes
    // past max_speed). At the lower bound we keep the negative intent so the
    // controller keeps braking until the real speed reaches 0 — the gear
    // direction prevents the vehicle from going past zero on its own.
    float realized_accel = desired_accel;
    if (desired_vel_ >= params_.max_speed && realized_accel > 0.0f) {
      realized_accel = 0.0f;
    }

    ControlCommand cmd;
    cmd.velocity = desired_vel_;
    cmd.acceleration = realized_accel;
    cmd.steer_angle = current_steer_;
    status_accel_ = realized_accel;
    return cmd;
  }

  std::string getName() const override { return "PHYSICS"; }

  std::string getStatusString() const override {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Acc: %.2f m/s2", status_accel_);
    return std::string(buf);
  }

private:
  Params params_;
  float desired_vel_ = 0.0f;
  float current_steer_ = 0.0f;
  float status_accel_ = 0.0f;
  Gear last_gear_ = Gear::PARK;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODES_PHYSICS_MODE_HPP
