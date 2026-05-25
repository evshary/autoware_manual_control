#ifndef TELEOP_PARAMS_LOADER_HPP
#define TELEOP_PARAMS_LOADER_HPP

// Loads runtime + per-mode parameters from a teleop_config.yaml-style ROS
// parameter set and registers the default drive modes with those params.

#include "core/control_runtime.hpp"
#include "core/drive_mode_factory.hpp"
#include "modes/cruise_mode.hpp"
#include "modes/physics_mode.hpp"
#include "modes/stop_mode.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace autoware::manual_control {

template <typename T>
inline T load_param(rclcpp::Node &node, const std::string &name, const T &dflt) {
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, dflt);
  }
  return node.get_parameter(name).get_value<T>();
}

// ROS params are doubles on the wire; the mode Params are floats. This helper
// loads a numeric param (accepting either int or double) and returns a float.
inline float load_float(rclcpp::Node &node, const std::string &name, float dflt) {
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, static_cast<double>(dflt));
  }
  auto p = node.get_parameter(name);
  return (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
             ? static_cast<float>(p.as_int())
             : static_cast<float>(p.as_double());
}

inline RuntimeConfig load_runtime_config(rclcpp::Node &node) {
  RuntimeConfig cfg;
  cfg.rate_hz = load_param<double>(node, "control_rate_hz", cfg.rate_hz);
  cfg.shift_stop_tolerance =
      load_float(node, "shift_stop_tolerance", cfg.shift_stop_tolerance);
  cfg.shift_brake_accel =
      load_float(node, "shift_brake_accel", cfg.shift_brake_accel);
  return cfg;
}

inline PhysicsDriveMode::Params load_physics_params(rclcpp::Node &node) {
  PhysicsDriveMode::Params p;
  p.max_speed = load_float(node, "physics.max_speed", p.max_speed);
  p.max_steer = load_float(node, "physics.max_steer", p.max_steer);
  p.steer_rate = load_float(node, "physics.steer_rate", p.steer_rate);
  p.steer_decay = load_float(node, "physics.steer_decay", p.steer_decay);
  p.steer_deadzone = load_float(node, "physics.steer_deadzone", p.steer_deadzone);
  p.accel_max = load_float(node, "physics.accel_max", p.accel_max);
  p.brake_max = load_float(node, "physics.brake_max", p.brake_max);
  p.coast_decel = load_float(node, "physics.coast_decel", p.coast_decel);
  p.max_vel_offset =
      load_float(node, "physics.max_vel_offset", p.max_vel_offset);
  return p;
}

inline CruiseDriveMode::Params load_cruise_params(rclcpp::Node &node) {
  CruiseDriveMode::Params p;
  p.max_speed = load_float(node, "cruise.max_speed", p.max_speed);
  p.steer_rate = load_float(node, "cruise.steer_rate", p.steer_rate);
  p.steer_limit = load_float(node, "cruise.steer_limit", p.steer_limit);
  p.vel_inc_hold_kph_s =
      load_float(node, "cruise.vel_inc_hold_kph_s", p.vel_inc_hold_kph_s);
  p.vel_dec_hold_kph_s =
      load_float(node, "cruise.vel_dec_hold_kph_s", p.vel_dec_hold_kph_s);
  p.accel_p_gain = load_float(node, "cruise.accel_p_gain", p.accel_p_gain);
  p.max_accel = load_float(node, "cruise.max_accel", p.max_accel);
  p.min_accel = load_float(node, "cruise.min_accel", p.min_accel);
  return p;
}

inline StopDriveMode::Params load_stop_params(rclcpp::Node &node) {
  StopDriveMode::Params p;
  p.brake_accel = load_float(node, "stop.brake_accel", p.brake_accel);
  return p;
}

// Read each mode's params from the node and register a creator that captures
// them. Both teleop entries call this so the wiring is single-sourced.
inline void register_default_modes(DriveModeFactory &factory, rclcpp::Node &node) {
  auto phys = load_physics_params(node);
  auto cruise = load_cruise_params(node);
  auto stop = load_stop_params(node);
  factory.registerMode(ModeType::PHYSICS,
                       [phys] { return std::make_unique<PhysicsDriveMode>(phys); });
  factory.registerMode(ModeType::CRUISE,
                       [cruise] { return std::make_unique<CruiseDriveMode>(cruise); });
  factory.registerMode(ModeType::STOP,
                       [stop] { return std::make_unique<StopDriveMode>(stop); });
}

} // namespace autoware::manual_control

#endif // TELEOP_PARAMS_LOADER_HPP
