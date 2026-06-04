#ifndef TELEOP_MODES_PHYSICS_MODE_HPP
#define TELEOP_MODES_PHYSICS_MODE_HPP

#include "core/drive_mode.hpp"
#include "core/param_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace autoware::manual_control {

class PhysicsDriveMode : public DriveMode {
public:
  struct Params {
    float max_speed = 27.78f;       // m/s (100 km/h)
    float max_steer = 0.6f;         // rad
    float steer_rate = 0.8f;        // rad/s while a steer key is held
    float steer_decay = 1.0f;       // rad/s auto-center on release
    float steer_deadzone = 0.01f;   // rad
    float accel_max = 3.5f;         // m/s^2 at full throttle
    float brake_max = 5.0f;         // m/s^2 at full brake
    float coast_decel = 2.0f;       // m/s^2 with no input (engine braking)
    float max_vel_offset = 3.0f;    // m/s — setpoint may lead real by this much
  };

  explicit PhysicsDriveMode(const Params &params) : params_(params) {}

  static constexpr const char *kName = "physics";
  static Params loadParams(rclcpp::Node &node) {
    Params p;
    p.max_speed = load_float(node, "physics.max_speed", p.max_speed);
    p.max_steer = load_float(node, "physics.max_steer", p.max_steer);
    p.steer_rate = load_float(node, "physics.steer_rate", p.steer_rate);
    p.steer_decay = load_float(node, "physics.steer_decay", p.steer_decay);
    p.steer_deadzone = load_float(node, "physics.steer_deadzone", p.steer_deadzone);
    p.accel_max = load_float(node, "physics.accel_max", p.accel_max);
    p.brake_max = load_float(node, "physics.brake_max", p.brake_max);
    p.coast_decel = load_float(node, "physics.coast_decel", p.coast_decel);
    p.max_vel_offset = load_float(node, "physics.max_vel_offset", p.max_vel_offset);
    return p;
  }

  void onEnter(const VehicleState &state) override {
    desired_vel_ = std::abs(state.velocity);
    current_steer_ = state.steer_angle;
    last_gear_ = state.gear;
    status_accel_ = 0.0f;
  }

  ControlCommand update(float dt, const InputState &input,
                        const VehicleState &vehicle_state) override {
    // Wipe the setpoint on gear change so speed isn't carried across a reverse.
    if (vehicle_state.gear != last_gear_) {
      desired_vel_ = 0.0f;
      last_gear_ = vehicle_state.gear;
    }

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

    float desired_accel;
    if (input.throttle > 0.0f) {
      desired_accel = input.throttle * params_.accel_max;
    } else if (input.brake > 0.0f) {
      desired_accel = -input.brake * params_.brake_max;
    } else {
      desired_accel = -params_.coast_decel;
    }

    // Bound the setpoint to real speed so it can't wind up arbitrarily far off.
    desired_vel_ += desired_accel * dt;
    desired_vel_ = std::min(desired_vel_,
                            std::abs(vehicle_state.velocity) + params_.max_vel_offset);
    desired_vel_ = std::clamp(desired_vel_, 0.0f, params_.max_speed);

    // Drop only positive intent at max_speed; keep negative so braking holds.
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
