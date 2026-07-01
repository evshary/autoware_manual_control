// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:msg/OperationModeState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/msg/detail/operation_mode_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "autoware_manual_control/msg/detail/time__traits.hpp"

namespace autoware_manual_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const OperationModeState & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: is_autoware_control_enabled
  {
    out << "is_autoware_control_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autoware_control_enabled, out);
    out << ", ";
  }

  // member: is_in_transition
  {
    out << "is_in_transition: ";
    rosidl_generator_traits::value_to_yaml(msg.is_in_transition, out);
    out << ", ";
  }

  // member: is_stop_mode_available
  {
    out << "is_stop_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_stop_mode_available, out);
    out << ", ";
  }

  // member: is_autonomous_mode_available
  {
    out << "is_autonomous_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autonomous_mode_available, out);
    out << ", ";
  }

  // member: is_local_mode_available
  {
    out << "is_local_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_local_mode_available, out);
    out << ", ";
  }

  // member: is_remote_mode_available
  {
    out << "is_remote_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_remote_mode_available, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperationModeState & msg,
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

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: is_autoware_control_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_autoware_control_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autoware_control_enabled, out);
    out << "\n";
  }

  // member: is_in_transition
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_in_transition: ";
    rosidl_generator_traits::value_to_yaml(msg.is_in_transition, out);
    out << "\n";
  }

  // member: is_stop_mode_available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_stop_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_stop_mode_available, out);
    out << "\n";
  }

  // member: is_autonomous_mode_available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_autonomous_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_autonomous_mode_available, out);
    out << "\n";
  }

  // member: is_local_mode_available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_local_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_local_mode_available, out);
    out << "\n";
  }

  // member: is_remote_mode_available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_remote_mode_available: ";
    rosidl_generator_traits::value_to_yaml(msg.is_remote_mode_available, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperationModeState & msg, bool use_flow_style = false)
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
  const autoware_manual_control::msg::OperationModeState & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::msg::OperationModeState & msg)
{
  return autoware_manual_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::msg::OperationModeState>()
{
  return "autoware_manual_control::msg::OperationModeState";
}

template<>
inline const char * name<autoware_manual_control::msg::OperationModeState>()
{
  return "autoware_manual_control/msg/OperationModeState";
}

template<>
struct has_fixed_size<autoware_manual_control::msg::OperationModeState>
  : std::integral_constant<bool, has_fixed_size<autoware_manual_control::msg::Time>::value> {};

template<>
struct has_bounded_size<autoware_manual_control::msg::OperationModeState>
  : std::integral_constant<bool, has_bounded_size<autoware_manual_control::msg::Time>::value> {};

template<>
struct is_message<autoware_manual_control::msg::OperationModeState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__TRAITS_HPP_
