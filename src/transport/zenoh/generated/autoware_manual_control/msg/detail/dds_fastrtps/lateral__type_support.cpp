// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from autoware_manual_control:msg/Lateral.idl
// generated code does not contain a copyright notice
#include "autoware_manual_control/msg/detail/lateral__rosidl_typesupport_fastrtps_cpp.hpp"
#include "autoware_manual_control/msg/detail/lateral__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace autoware_manual_control
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const autoware_manual_control::msg::Time &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  autoware_manual_control::msg::Time &);
size_t get_serialized_size(
  const autoware_manual_control::msg::Time &,
  size_t current_alignment);
size_t
max_serialized_size_Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace autoware_manual_control

// functions for autoware_manual_control::msg::Time already declared above


namespace autoware_manual_control
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
cdr_serialize(
  const autoware_manual_control::msg::Lateral & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: stamp
  autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.stamp,
    cdr);
  // Member: control_time
  autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.control_time,
    cdr);
  // Member: steering_tire_angle
  cdr << ros_message.steering_tire_angle;
  // Member: steering_tire_rotation_rate
  cdr << ros_message.steering_tire_rotation_rate;
  // Member: is_defined_steering_tire_rotation_rate
  cdr << (ros_message.is_defined_steering_tire_rotation_rate ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  autoware_manual_control::msg::Lateral & ros_message)
{
  // Member: stamp
  autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.stamp);

  // Member: control_time
  autoware_manual_control::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.control_time);

  // Member: steering_tire_angle
  cdr >> ros_message.steering_tire_angle;

  // Member: steering_tire_rotation_rate
  cdr >> ros_message.steering_tire_rotation_rate;

  // Member: is_defined_steering_tire_rotation_rate
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_defined_steering_tire_rotation_rate = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
get_serialized_size(
  const autoware_manual_control::msg::Lateral & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: stamp

  current_alignment +=
    autoware_manual_control::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.stamp, current_alignment);
  // Member: control_time

  current_alignment +=
    autoware_manual_control::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.control_time, current_alignment);
  // Member: steering_tire_angle
  {
    size_t item_size = sizeof(ros_message.steering_tire_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: steering_tire_rotation_rate
  {
    size_t item_size = sizeof(ros_message.steering_tire_rotation_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_defined_steering_tire_rotation_rate
  {
    size_t item_size = sizeof(ros_message.is_defined_steering_tire_rotation_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_autoware_manual_control
max_serialized_size_Lateral(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: stamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        autoware_manual_control::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: control_time
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        autoware_manual_control::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: steering_tire_angle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: steering_tire_rotation_rate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: is_defined_steering_tire_rotation_rate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = autoware_manual_control::msg::Lateral;
    is_plain =
      (
      offsetof(DataType, is_defined_steering_tire_rotation_rate) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Lateral__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const autoware_manual_control::msg::Lateral *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Lateral__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<autoware_manual_control::msg::Lateral *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Lateral__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const autoware_manual_control::msg::Lateral *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Lateral__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Lateral(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Lateral__callbacks = {
  "autoware_manual_control::msg",
  "Lateral",
  _Lateral__cdr_serialize,
  _Lateral__cdr_deserialize,
  _Lateral__get_serialized_size,
  _Lateral__max_serialized_size
};

static rosidl_message_type_support_t _Lateral__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Lateral__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace autoware_manual_control

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_autoware_manual_control
const rosidl_message_type_support_t *
get_message_type_support_handle<autoware_manual_control::msg::Lateral>()
{
  return &autoware_manual_control::msg::typesupport_fastrtps_cpp::_Lateral__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, autoware_manual_control, msg, Lateral)() {
  return &autoware_manual_control::msg::typesupport_fastrtps_cpp::_Lateral__handle;
}

#ifdef __cplusplus
}
#endif
