// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/GearReport.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_REPORT__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_REPORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/gear_report__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_GearReport_report
{
public:
  explicit Init_GearReport_report(::autoware_manual_control::msg::GearReport & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::GearReport report(::autoware_manual_control::msg::GearReport::_report_type arg)
  {
    msg_.report = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::GearReport msg_;
};

class Init_GearReport_stamp
{
public:
  Init_GearReport_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GearReport_report stamp(::autoware_manual_control::msg::GearReport::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_GearReport_report(msg_);
  }

private:
  ::autoware_manual_control::msg::GearReport msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::GearReport>()
{
  return autoware_manual_control::msg::builder::Init_GearReport_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_REPORT__BUILDER_HPP_
