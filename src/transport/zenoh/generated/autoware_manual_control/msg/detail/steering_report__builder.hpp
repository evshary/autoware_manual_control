// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/SteeringReport.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__STEERING_REPORT__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__STEERING_REPORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/steering_report__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_SteeringReport_steering_tire_angle
{
public:
  explicit Init_SteeringReport_steering_tire_angle(::autoware_manual_control::msg::SteeringReport & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::SteeringReport steering_tire_angle(::autoware_manual_control::msg::SteeringReport::_steering_tire_angle_type arg)
  {
    msg_.steering_tire_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::SteeringReport msg_;
};

class Init_SteeringReport_stamp
{
public:
  Init_SteeringReport_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SteeringReport_steering_tire_angle stamp(::autoware_manual_control::msg::SteeringReport::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_SteeringReport_steering_tire_angle(msg_);
  }

private:
  ::autoware_manual_control::msg::SteeringReport msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::SteeringReport>()
{
  return autoware_manual_control::msg::builder::Init_SteeringReport_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__STEERING_REPORT__BUILDER_HPP_
