// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/Time.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__TIME__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__TIME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/time__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_Time_nanosec
{
public:
  explicit Init_Time_nanosec(::autoware_manual_control::msg::Time & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::Time nanosec(::autoware_manual_control::msg::Time::_nanosec_type arg)
  {
    msg_.nanosec = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::Time msg_;
};

class Init_Time_sec
{
public:
  Init_Time_sec()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Time_nanosec sec(::autoware_manual_control::msg::Time::_sec_type arg)
  {
    msg_.sec = std::move(arg);
    return Init_Time_nanosec(msg_);
  }

private:
  ::autoware_manual_control::msg::Time msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::Time>()
{
  return autoware_manual_control::msg::builder::Init_Time_sec();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__TIME__BUILDER_HPP_
