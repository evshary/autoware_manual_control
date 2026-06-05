#ifndef TELEOP_PARAM_UTILS_HPP
#define TELEOP_PARAM_UTILS_HPP

// Generic ROS-param read helpers (no mode / runtime-config knowledge).

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

// Accepts a YAML int or double so a whole-number param doesn't throw.
inline float load_float(rclcpp::Node &node, const std::string &name, float dflt) {
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, static_cast<double>(dflt));
  }
  auto p = node.get_parameter(name);
  return (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
             ? static_cast<float>(p.as_int())
             : static_cast<float>(p.as_double());
}

inline double load_double(rclcpp::Node &node, const std::string &name, double dflt) {
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, dflt);
  }
  auto p = node.get_parameter(name);
  return (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
             ? static_cast<double>(p.as_int())
             : p.as_double();
}

} // namespace autoware::manual_control

#endif // TELEOP_PARAM_UTILS_HPP
