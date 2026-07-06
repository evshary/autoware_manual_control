// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/ResponseStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__RESPONSE_STATUS__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__RESPONSE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/response_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_ResponseStatus_message
{
public:
  explicit Init_ResponseStatus_message(::autoware_manual_control::msg::ResponseStatus & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::ResponseStatus message(::autoware_manual_control::msg::ResponseStatus::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::ResponseStatus msg_;
};

class Init_ResponseStatus_code
{
public:
  explicit Init_ResponseStatus_code(::autoware_manual_control::msg::ResponseStatus & msg)
  : msg_(msg)
  {}
  Init_ResponseStatus_message code(::autoware_manual_control::msg::ResponseStatus::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_ResponseStatus_message(msg_);
  }

private:
  ::autoware_manual_control::msg::ResponseStatus msg_;
};

class Init_ResponseStatus_success
{
public:
  Init_ResponseStatus_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ResponseStatus_code success(::autoware_manual_control::msg::ResponseStatus::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ResponseStatus_code(msg_);
  }

private:
  ::autoware_manual_control::msg::ResponseStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::ResponseStatus>()
{
  return autoware_manual_control::msg::builder::Init_ResponseStatus_success();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__RESPONSE_STATUS__BUILDER_HPP_
