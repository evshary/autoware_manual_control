// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/VelocityReport.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/velocity_report__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_VelocityReport_heading_rate
{
public:
  explicit Init_VelocityReport_heading_rate(::autoware_manual_control::msg::VelocityReport & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::VelocityReport heading_rate(::autoware_manual_control::msg::VelocityReport::_heading_rate_type arg)
  {
    msg_.heading_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::VelocityReport msg_;
};

class Init_VelocityReport_lateral_velocity
{
public:
  explicit Init_VelocityReport_lateral_velocity(::autoware_manual_control::msg::VelocityReport & msg)
  : msg_(msg)
  {}
  Init_VelocityReport_heading_rate lateral_velocity(::autoware_manual_control::msg::VelocityReport::_lateral_velocity_type arg)
  {
    msg_.lateral_velocity = std::move(arg);
    return Init_VelocityReport_heading_rate(msg_);
  }

private:
  ::autoware_manual_control::msg::VelocityReport msg_;
};

class Init_VelocityReport_longitudinal_velocity
{
public:
  explicit Init_VelocityReport_longitudinal_velocity(::autoware_manual_control::msg::VelocityReport & msg)
  : msg_(msg)
  {}
  Init_VelocityReport_lateral_velocity longitudinal_velocity(::autoware_manual_control::msg::VelocityReport::_longitudinal_velocity_type arg)
  {
    msg_.longitudinal_velocity = std::move(arg);
    return Init_VelocityReport_lateral_velocity(msg_);
  }

private:
  ::autoware_manual_control::msg::VelocityReport msg_;
};

class Init_VelocityReport_header
{
public:
  Init_VelocityReport_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelocityReport_longitudinal_velocity header(::autoware_manual_control::msg::VelocityReport::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VelocityReport_longitudinal_velocity(msg_);
  }

private:
  ::autoware_manual_control::msg::VelocityReport msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::VelocityReport>()
{
  return autoware_manual_control::msg::builder::Init_VelocityReport_header();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__BUILDER_HPP_
