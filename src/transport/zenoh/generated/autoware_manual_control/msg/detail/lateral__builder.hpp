// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/Lateral.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LATERAL__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LATERAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/lateral__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_Lateral_is_defined_steering_tire_rotation_rate
{
public:
  explicit Init_Lateral_is_defined_steering_tire_rotation_rate(::autoware_manual_control::msg::Lateral & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::Lateral is_defined_steering_tire_rotation_rate(::autoware_manual_control::msg::Lateral::_is_defined_steering_tire_rotation_rate_type arg)
  {
    msg_.is_defined_steering_tire_rotation_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::Lateral msg_;
};

class Init_Lateral_steering_tire_rotation_rate
{
public:
  explicit Init_Lateral_steering_tire_rotation_rate(::autoware_manual_control::msg::Lateral & msg)
  : msg_(msg)
  {}
  Init_Lateral_is_defined_steering_tire_rotation_rate steering_tire_rotation_rate(::autoware_manual_control::msg::Lateral::_steering_tire_rotation_rate_type arg)
  {
    msg_.steering_tire_rotation_rate = std::move(arg);
    return Init_Lateral_is_defined_steering_tire_rotation_rate(msg_);
  }

private:
  ::autoware_manual_control::msg::Lateral msg_;
};

class Init_Lateral_steering_tire_angle
{
public:
  explicit Init_Lateral_steering_tire_angle(::autoware_manual_control::msg::Lateral & msg)
  : msg_(msg)
  {}
  Init_Lateral_steering_tire_rotation_rate steering_tire_angle(::autoware_manual_control::msg::Lateral::_steering_tire_angle_type arg)
  {
    msg_.steering_tire_angle = std::move(arg);
    return Init_Lateral_steering_tire_rotation_rate(msg_);
  }

private:
  ::autoware_manual_control::msg::Lateral msg_;
};

class Init_Lateral_control_time
{
public:
  explicit Init_Lateral_control_time(::autoware_manual_control::msg::Lateral & msg)
  : msg_(msg)
  {}
  Init_Lateral_steering_tire_angle control_time(::autoware_manual_control::msg::Lateral::_control_time_type arg)
  {
    msg_.control_time = std::move(arg);
    return Init_Lateral_steering_tire_angle(msg_);
  }

private:
  ::autoware_manual_control::msg::Lateral msg_;
};

class Init_Lateral_stamp
{
public:
  Init_Lateral_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Lateral_control_time stamp(::autoware_manual_control::msg::Lateral::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Lateral_control_time(msg_);
  }

private:
  ::autoware_manual_control::msg::Lateral msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::Lateral>()
{
  return autoware_manual_control::msg::builder::Init_Lateral_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LATERAL__BUILDER_HPP_
