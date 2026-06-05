#ifndef TELEOP_MODES_CRUISE_MODE_HPP
#define TELEOP_MODES_CRUISE_MODE_HPP

#include "core/drive_mode.hpp"
#include "core/param_utils.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::manual_control {

class CruiseDriveMode : public DriveMode {
public:
  struct Params {
    float max_speed = 27.78f;          // m/s (100 km/h)
    float steer_rate = 0.3f;           // rad/s
    float steer_limit = 0.6f;          // rad
    float vel_inc_hold_kph_s = 5.0f;   // km/h per second when holding throttle
    float vel_dec_hold_kph_s = 10.0f;  // km/h per second when holding brake
    float accel_p_gain = 3.0f;
    float max_accel = 5.0f;
    float min_accel = -10.0f;
  };

  explicit CruiseDriveMode(const Params &params) : params_(params) {}

  static constexpr const char *kName = "cruise";
  static Params loadParams(rclcpp::Node &node) {
    Params p;
    p.max_speed = load_float(node, "cruise.max_speed", p.max_speed);
    p.steer_rate = load_float(node, "cruise.steer_rate", p.steer_rate);
    p.steer_limit = load_float(node, "cruise.steer_limit", p.steer_limit);
    p.vel_inc_hold_kph_s = load_float(node, "cruise.vel_inc_hold_kph_s", p.vel_inc_hold_kph_s);
    p.vel_dec_hold_kph_s = load_float(node, "cruise.vel_dec_hold_kph_s", p.vel_dec_hold_kph_s);
    p.accel_p_gain = load_float(node, "cruise.accel_p_gain", p.accel_p_gain);
    p.max_accel = load_float(node, "cruise.max_accel", p.max_accel);
    p.min_accel = load_float(node, "cruise.min_accel", p.min_accel);
    return p;
  }

  void onEnter(const VehicleState &state) override {
    target_speed_ = std::abs(state.velocity);
    if (target_speed_ > params_.max_speed) {
      target_speed_ = 0.0f;
    }
    if (state.gear == Gear::REVERSE) {
      target_speed_ = 0.0f;
    }
    current_steer_ = state.steer_angle;
    last_throttle_press_ = false;
    last_brake_press_ = false;
    last_gear_ = state.gear;
  }

  ControlCommand update(float dt, const Intent &intent,
                        const VehicleState &vehicle_state) override {
    if (vehicle_state.gear != last_gear_) {
      target_speed_ = 0.0f;
      last_gear_ = vehicle_state.gear;
    }

    // Steering (no auto-centering: held angle persists).
    if (intent.steer_dir != 0) {
      current_steer_ += intent.steer_dir * params_.steer_rate * dt;
    }
    current_steer_ =
        std::clamp(current_steer_, -params_.steer_limit, params_.steer_limit);

    if (vehicle_state.gear == Gear::PARK) {
      target_speed_ = 0.0f;
    } else {
      const float inc_per_s = params_.vel_inc_hold_kph_s / 3.6f;
      const float dec_per_s = params_.vel_dec_hold_kph_s / 3.6f;

      bool throttle_active = (intent.throttle > 0.0f);
      if (throttle_active) {
        if (intent.throttle_hold) {
          target_speed_ += inc_per_s * dt;
        } else if (!last_throttle_press_) {
          float current_kph = target_speed_ * 3.6f;
          target_speed_ = (std::floor(current_kph) + 1.0f) / 3.6f;
        }
      }
      last_throttle_press_ = throttle_active;

      bool brake_active = (intent.brake > 0.0f);
      if (brake_active) {
        if (intent.brake_hold) {
          target_speed_ -= dec_per_s * dt;
        } else if (!last_brake_press_) {
          float current_kph = target_speed_ * 3.6f;
          target_speed_ = (std::ceil(current_kph) - 1.0f) / 3.6f;
        }
      }
      last_brake_press_ = brake_active;
    }

    target_speed_ = std::clamp(target_speed_, 0.0f, params_.max_speed);

    float real_speed = std::abs(vehicle_state.velocity);
    float accel_cmd = (target_speed_ - real_speed) * params_.accel_p_gain;
    accel_cmd = std::clamp(accel_cmd, params_.min_accel, params_.max_accel);

    ControlCommand cmd;
    cmd.velocity = target_speed_;
    cmd.acceleration = accel_cmd;
    cmd.steer_angle = current_steer_;
    return cmd;
  }

  std::string getName() const override { return "CRUISE"; }
  std::string getStatusString() const override { return ""; }

private:
  Params params_;
  float target_speed_ = 0.0f;
  float current_steer_ = 0.0f;
  Gear last_gear_ = Gear::PARK;
  bool last_throttle_press_ = false;
  bool last_brake_press_ = false;
};

} // namespace autoware::manual_control

#endif // TELEOP_MODES_CRUISE_MODE_HPP
