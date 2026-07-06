// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:msg/VelocityReport.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/msg/detail/velocity_report__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "autoware_manual_control/msg/detail/header__traits.hpp"

namespace autoware_manual_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const VelocityReport & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: longitudinal_velocity
  {
    out << "longitudinal_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.longitudinal_velocity, out);
    out << ", ";
  }

  // member: lateral_velocity
  {
    out << "lateral_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.lateral_velocity, out);
    out << ", ";
  }

  // member: heading_rate
  {
    out << "heading_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VelocityReport & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: longitudinal_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitudinal_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.longitudinal_velocity, out);
    out << "\n";
  }

  // member: lateral_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lateral_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.lateral_velocity, out);
    out << "\n";
  }

  // member: heading_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.heading_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VelocityReport & msg, bool use_flow_style = false)
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
  const autoware_manual_control::msg::VelocityReport & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::msg::VelocityReport & msg)
{
  return autoware_manual_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::msg::VelocityReport>()
{
  return "autoware_manual_control::msg::VelocityReport";
}

template<>
inline const char * name<autoware_manual_control::msg::VelocityReport>()
{
  return "autoware_manual_control/msg/VelocityReport";
}

template<>
struct has_fixed_size<autoware_manual_control::msg::VelocityReport>
  : std::integral_constant<bool, has_fixed_size<autoware_manual_control::msg::Header>::value> {};

template<>
struct has_bounded_size<autoware_manual_control::msg::VelocityReport>
  : std::integral_constant<bool, has_bounded_size<autoware_manual_control::msg::Header>::value> {};

template<>
struct is_message<autoware_manual_control::msg::VelocityReport>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__TRAITS_HPP_
