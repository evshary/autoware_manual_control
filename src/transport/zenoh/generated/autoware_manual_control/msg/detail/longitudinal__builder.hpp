// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/Longitudinal.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/longitudinal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_Longitudinal_is_defined_jerk
{
public:
  explicit Init_Longitudinal_is_defined_jerk(::autoware_manual_control::msg::Longitudinal & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::Longitudinal is_defined_jerk(::autoware_manual_control::msg::Longitudinal::_is_defined_jerk_type arg)
  {
    msg_.is_defined_jerk = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::Longitudinal msg_;
};

class Init_Longitudinal_is_defined_acceleration
{
public:
  explicit Init_Longitudinal_is_defined_acceleration(::autoware_manual_control::msg::Longitudinal & msg)
  : msg_(msg)
  {}
  Init_Longitudinal_is_defined_jerk is_defined_acceleration(::autoware_manual_control::msg::Longitudinal::_is_defined_acceleration_type arg)
  {
    msg_.is_defined_acceleration = std::move(arg);
    return Init_Longitudinal_is_defined_jerk(msg_);
  }

private:
  ::autoware_manual_control::msg::Longitudinal msg_;
};

class Init_Longitudinal_jerk
{
public:
  explicit Init_Longitudinal_jerk(::autoware_manual_control::msg::Longitudinal & msg)
  : msg_(msg)
  {}
  Init_Longitudinal_is_defined_acceleration jerk(::autoware_manual_control::msg::Longitudinal::_jerk_type arg)
  {
    msg_.jerk = std::move(arg);
    return Init_Longitudinal_is_defined_acceleration(msg_);
  }

private:
  ::autoware_manual_control::msg::Longitudinal msg_;
};

class Init_Longitudinal_acceleration
{
public:
  explicit Init_Longitudinal_acceleration(::autoware_manual_control::msg::Longitudinal & msg)
  : msg_(msg)
  {}
  Init_Longitudinal_jerk acceleration(::autoware_manual_control::msg::Longitudinal::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_Longitudinal_jerk(msg_);
  }

private:
  ::autoware_manual_control::msg::Longitudinal msg_;
};

class Init_Longitudinal_velocity
{
public:
  explicit Init_Longitudinal_velocity(::autoware_manual_control::msg::Longitudinal & msg)
  : msg_(msg)
  {}
  Init_Longitudinal_acceleration velocity(::autoware_manual_control::msg::Longitudinal::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Longitudinal_acceleration(msg_);
  }

private:
  ::autoware_manual_control::msg::Longitudinal msg_;
};

class Init_Longitudinal_control_time
{
public:
  explicit Init_Longitudinal_control_time(::autoware_manual_control::msg::Longitudinal & msg)
  : msg_(msg)
  {}
  Init_Longitudinal_velocity control_time(::autoware_manual_control::msg::Longitudinal::_control_time_type arg)
  {
    msg_.control_time = std::move(arg);
    return Init_Longitudinal_velocity(msg_);
  }

private:
  ::autoware_manual_control::msg::Longitudinal msg_;
};

class Init_Longitudinal_stamp
{
public:
  Init_Longitudinal_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Longitudinal_control_time stamp(::autoware_manual_control::msg::Longitudinal::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Longitudinal_control_time(msg_);
  }

private:
  ::autoware_manual_control::msg::Longitudinal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::Longitudinal>()
{
  return autoware_manual_control::msg::builder::Init_Longitudinal_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__BUILDER_HPP_
