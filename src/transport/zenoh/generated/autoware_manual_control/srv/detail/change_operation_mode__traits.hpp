// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_manual_control:srv/ChangeOperationMode.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__TRAITS_HPP_
#define AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "autoware_manual_control/srv/detail/change_operation_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace autoware_manual_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChangeOperationMode_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangeOperationMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangeOperationMode_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autoware_manual_control

namespace rosidl_generator_traits
{

[[deprecated("use autoware_manual_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_manual_control::srv::ChangeOperationMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::srv::ChangeOperationMode_Request & msg)
{
  return autoware_manual_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::srv::ChangeOperationMode_Request>()
{
  return "autoware_manual_control::srv::ChangeOperationMode_Request";
}

template<>
inline const char * name<autoware_manual_control::srv::ChangeOperationMode_Request>()
{
  return "autoware_manual_control/srv/ChangeOperationMode_Request";
}

template<>
struct has_fixed_size<autoware_manual_control::srv::ChangeOperationMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<autoware_manual_control::srv::ChangeOperationMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<autoware_manual_control::srv::ChangeOperationMode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'status'
#include "autoware_manual_control/msg/detail/response_status__traits.hpp"

namespace autoware_manual_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChangeOperationMode_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    to_flow_style_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ChangeOperationMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status:\n";
    to_block_style_yaml(msg.status, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ChangeOperationMode_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace autoware_manual_control

namespace rosidl_generator_traits
{

[[deprecated("use autoware_manual_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const autoware_manual_control::srv::ChangeOperationMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  autoware_manual_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use autoware_manual_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const autoware_manual_control::srv::ChangeOperationMode_Response & msg)
{
  return autoware_manual_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<autoware_manual_control::srv::ChangeOperationMode_Response>()
{
  return "autoware_manual_control::srv::ChangeOperationMode_Response";
}

template<>
inline const char * name<autoware_manual_control::srv::ChangeOperationMode_Response>()
{
  return "autoware_manual_control/srv/ChangeOperationMode_Response";
}

template<>
struct has_fixed_size<autoware_manual_control::srv::ChangeOperationMode_Response>
  : std::integral_constant<bool, has_fixed_size<autoware_manual_control::msg::ResponseStatus>::value> {};

template<>
struct has_bounded_size<autoware_manual_control::srv::ChangeOperationMode_Response>
  : std::integral_constant<bool, has_bounded_size<autoware_manual_control::msg::ResponseStatus>::value> {};

template<>
struct is_message<autoware_manual_control::srv::ChangeOperationMode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<autoware_manual_control::srv::ChangeOperationMode>()
{
  return "autoware_manual_control::srv::ChangeOperationMode";
}

template<>
inline const char * name<autoware_manual_control::srv::ChangeOperationMode>()
{
  return "autoware_manual_control/srv/ChangeOperationMode";
}

template<>
struct has_fixed_size<autoware_manual_control::srv::ChangeOperationMode>
  : std::integral_constant<
    bool,
    has_fixed_size<autoware_manual_control::srv::ChangeOperationMode_Request>::value &&
    has_fixed_size<autoware_manual_control::srv::ChangeOperationMode_Response>::value
  >
{
};

template<>
struct has_bounded_size<autoware_manual_control::srv::ChangeOperationMode>
  : std::integral_constant<
    bool,
    has_bounded_size<autoware_manual_control::srv::ChangeOperationMode_Request>::value &&
    has_bounded_size<autoware_manual_control::srv::ChangeOperationMode_Response>::value
  >
{
};

template<>
struct is_service<autoware_manual_control::srv::ChangeOperationMode>
  : std::true_type
{
};

template<>
struct is_service_request<autoware_manual_control::srv::ChangeOperationMode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<autoware_manual_control::srv::ChangeOperationMode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__TRAITS_HPP_
