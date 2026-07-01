// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:msg/Nested.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__NESTED__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__NESTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/msg/detail/nested__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'basic_types_value'
#include "autoware_manual_control/msg/detail/basic_types__traits.hpp"

namespace autoware_manual_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const Nested & msg,
  std::ostream & out)
{
  out << "{";
  // member: basic_types_value
  {
    out << "basic_types_value: ";
    to_flow_style_yaml(msg.basic_types_value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Nested & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: basic_types_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "basic_types_value:\n";
    to_block_style_yaml(msg.basic_types_value, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Nested & msg, bool use_flow_style = false)
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
  const autoware_manual_control::msg::Nested & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::msg::Nested & msg)
{
  return autoware_manual_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::msg::Nested>()
{
  return "autoware_manual_control::msg::Nested";
}

template<>
inline const char * name<autoware_manual_control::msg::Nested>()
{
  return "autoware_manual_control/msg/Nested";
}

template<>
struct has_fixed_size<autoware_manual_control::msg::Nested>
  : std::integral_constant<bool, has_fixed_size<autoware_manual_control::msg::BasicTypes>::value> {};

template<>
struct has_bounded_size<autoware_manual_control::msg::Nested>
  : std::integral_constant<bool, has_bounded_size<autoware_manual_control::msg::BasicTypes>::value> {};

template<>
struct is_message<autoware_manual_control::msg::Nested>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__NESTED__TRAITS_HPP_
