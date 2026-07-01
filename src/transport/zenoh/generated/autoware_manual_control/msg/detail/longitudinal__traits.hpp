// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:msg/Longitudinal.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/msg/detail/longitudinal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
// Member 'control_time'
#include "autoware_manual_control/msg/detail/time__traits.hpp"

namespace autoware_manual_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Longitudinal & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: control_time
  {
    out << "control_time: ";
    to_flow_style_yaml(msg.control_time, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: jerk
  {
    out << "jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.jerk, out);
    out << ", ";
  }

  // member: is_defined_acceleration
  {
    out << "is_defined_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.is_defined_acceleration, out);
    out << ", ";
  }

  // member: is_defined_jerk
  {
    out << "is_defined_jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.is_defined_jerk, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Longitudinal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: control_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_time:\n";
    to_block_style_yaml(msg.control_time, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << "\n";
  }

  // member: jerk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.jerk, out);
    out << "\n";
  }

  // member: is_defined_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_defined_acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.is_defined_acceleration, out);
    out << "\n";
  }

  // member: is_defined_jerk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_defined_jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.is_defined_jerk, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Longitudinal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace autoware_manual_control

namespace rosidl_generator_traits
{

[[deprecated("use autoware_manual_control::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_manual_control::msg::Longitudinal & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::msg::Longitudinal & msg)
{
  return autoware_manual_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::msg::Longitudinal>()
{
  return "autoware_manual_control::msg::Longitudinal";
}

template<>
inline const char * name<autoware_manual_control::msg::Longitudinal>()
{
  return "autoware_manual_control/msg/Longitudinal";
}

template<>
struct has_fixed_size<autoware_manual_control::msg::Longitudinal>
  : std::integral_constant<bool, has_fixed_size<autoware_manual_control::msg::Time>::value> {};

template<>
struct has_bounded_size<autoware_manual_control::msg::Longitudinal>
  : std::integral_constant<bool, has_bounded_size<autoware_manual_control::msg::Time>::value> {};

template<>
struct is_message<autoware_manual_control::msg::Longitudinal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__TRAITS_HPP_
