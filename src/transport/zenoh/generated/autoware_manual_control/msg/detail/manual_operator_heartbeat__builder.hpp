// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/ManualOperatorHeartbeat.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/manual_operator_heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_ManualOperatorHeartbeat_ready
{
public:
  explicit Init_ManualOperatorHeartbeat_ready(::autoware_manual_control::msg::ManualOperatorHeartbeat & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::ManualOperatorHeartbeat ready(::autoware_manual_control::msg::ManualOperatorHeartbeat::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::ManualOperatorHeartbeat msg_;
};

class Init_ManualOperatorHeartbeat_stamp
{
public:
  Init_ManualOperatorHeartbeat_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManualOperatorHeartbeat_ready stamp(::autoware_manual_control::msg::ManualOperatorHeartbeat::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ManualOperatorHeartbeat_ready(msg_);
  }

private:
  ::autoware_manual_control::msg::ManualOperatorHeartbeat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::ManualOperatorHeartbeat>()
{
  return autoware_manual_control::msg::builder::Init_ManualOperatorHeartbeat_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__BUILDER_HPP_
