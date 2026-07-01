// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:msg/Header.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__HEADER__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__HEADER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/msg/detail/header__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "autoware_manual_control/msg/detail/time__traits.hpp"

namespace autoware_manual_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Header & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: frame_id
  {
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Header & msg,
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

  // member: frame_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Header & msg, bool use_flow_style = false)
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
  const autoware_manual_control::msg::Header & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::msg::Header & msg)
{
  return autoware_manual_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::msg::Header>()
{
  return "autoware_manual_control::msg::Header";
}

template<>
inline const char * name<autoware_manual_control::msg::Header>()
{
  return "autoware_manual_control/msg/Header";
}

template<>
struct has_fixed_size<autoware_manual_control::msg::Header>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autoware_manual_control::msg::Header>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autoware_manual_control::msg::Header>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__HEADER__TRAITS_HPP_
