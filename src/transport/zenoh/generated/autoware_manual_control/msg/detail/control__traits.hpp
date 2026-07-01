// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONTROL__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
// Member 'control_time'
#include "autoware_manual_control/msg/detail/time__traits.hpp"
// Member 'lateral'
#include "autoware_manual_control/msg/detail/lateral__traits.hpp"
// Member 'longitudinal'
#include "autoware_manual_control/msg/detail/longitudinal__traits.hpp"

namespace autoware_manual_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Control & msg,
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

  // member: lateral
  {
    out << "lateral: ";
    to_flow_style_yaml(msg.lateral, out);
    out << ", ";
  }

  // member: longitudinal
  {
    out << "longitudinal: ";
    to_flow_style_yaml(msg.longitudinal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control & msg,
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

  // member: lateral
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lateral:\n";
    to_block_style_yaml(msg.lateral, out, indentation + 2);
  }

  // member: longitudinal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitudinal:\n";
    to_block_style_yaml(msg.longitudinal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control & msg, bool use_flow_style = false)
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
  const autoware_manual_control::msg::Control & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::msg::Control & msg)
{
  return autoware_manual_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::msg::Control>()
{
  return "autoware_manual_control::msg::Control";
}

template<>
inline const char * name<autoware_manual_control::msg::Control>()
{
  return "autoware_manual_control/msg/Control";
}

template<>
struct has_fixed_size<autoware_manual_control::msg::Control>
  : std::integral_constant<bool, has_fixed_size<autoware_manual_control::msg::Lateral>::value && has_fixed_size<autoware_manual_control::msg::Longitudinal>::value && has_fixed_size<autoware_manual_control::msg::Time>::value> {};

template<>
struct has_bounded_size<autoware_manual_control::msg::Control>
  : std::integral_constant<bool, has_bounded_size<autoware_manual_control::msg::Lateral>::value && has_bounded_size<autoware_manual_control::msg::Longitudinal>::value && has_bounded_size<autoware_manual_control::msg::Time>::value> {};

template<>
struct is_message<autoware_manual_control::msg::Control>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONTROL__TRAITS_HPP_
