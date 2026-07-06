// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONTROL__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_Control_longitudinal
{
public:
  explicit Init_Control_longitudinal(::autoware_manual_control::msg::Control & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::Control longitudinal(::autoware_manual_control::msg::Control::_longitudinal_type arg)
  {
    msg_.longitudinal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::Control msg_;
};

class Init_Control_lateral
{
public:
  explicit Init_Control_lateral(::autoware_manual_control::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_longitudinal lateral(::autoware_manual_control::msg::Control::_lateral_type arg)
  {
    msg_.lateral = std::move(arg);
    return Init_Control_longitudinal(msg_);
  }

private:
  ::autoware_manual_control::msg::Control msg_;
};

class Init_Control_control_time
{
public:
  explicit Init_Control_control_time(::autoware_manual_control::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_lateral control_time(::autoware_manual_control::msg::Control::_control_time_type arg)
  {
    msg_.control_time = std::move(arg);
    return Init_Control_lateral(msg_);
  }

private:
  ::autoware_manual_control::msg::Control msg_;
};

class Init_Control_stamp
{
public:
  Init_Control_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_control_time stamp(::autoware_manual_control::msg::Control::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Control_control_time(msg_);
  }

private:
  ::autoware_manual_control::msg::Control msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::Control>()
{
  return autoware_manual_control::msg::builder::Init_Control_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__CONTROL__BUILDER_HPP_
