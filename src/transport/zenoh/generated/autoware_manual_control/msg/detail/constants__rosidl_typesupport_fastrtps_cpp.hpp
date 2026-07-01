// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from autoware_manual_control:msg/Constants.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONSTANTS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONSTANTS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "autoware_manual_control/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "autoware_manual_control/msg/detail/constants__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace autoware_manual_control
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
cdr_serialize(
  const autoware_manual_control::msg::Constants & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  autoware_manual_control::msg::Constants & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
get_serialized_size(
  const autoware_manual_control::msg::Constants & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
max_serialized_size_Constants(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace autoware_manual_control

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autoware_manual_control, msg, Constants)();

#ifdef __cplusplus
}
#endif

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONSTANTS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
