// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:msg/PoseWithCovariance.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/msg/detail/pose_with_covariance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "autoware_manual_control/msg/detail/pose__traits.hpp"

namespace autoware_manual_control
{

namespace msg
{

inline void to_flow_style_yaml(
  const PoseWithCovariance & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: covariance
  {
    if (msg.covariance.size() == 0) {
      out << "covariance: []";
    } else {
      out << "covariance: [";
      size_t pending_items = msg.covariance.size();
      for (auto item : msg.covariance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PoseWithCovariance & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.covariance.size() == 0) {
      out << "covariance: []\n";
    } else {
      out << "covariance:\n";
      for (auto item : msg.covariance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PoseWithCovariance & msg, bool use_flow_style = false)
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
  const autoware_manual_control::msg::PoseWithCovariance & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::msg::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::msg::PoseWithCovariance & msg)
{
  return autoware_manual_control::msg::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::msg::PoseWithCovariance>()
{
  return "autoware_manual_control::msg::PoseWithCovariance";
}

template<>
inline const char * name<autoware_manual_control::msg::PoseWithCovariance>()
{
  return "autoware_manual_control/msg/PoseWithCovariance";
}

template<>
struct has_fixed_size<autoware_manual_control::msg::PoseWithCovariance>
  : std::integral_constant<bool, has_fixed_size<autoware_manual_control::msg::Pose>::value> {};

template<>
struct has_bounded_size<autoware_manual_control::msg::PoseWithCovariance>
  : std::integral_constant<bool, has_bounded_size<autoware_manual_control::msg::Pose>::value> {};

template<>
struct is_message<autoware_manual_control::msg::PoseWithCovariance>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__TRAITS_HPP_
