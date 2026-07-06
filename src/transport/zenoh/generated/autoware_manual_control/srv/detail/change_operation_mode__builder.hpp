// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:srv/ChangeOperationMode.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/srv/detail/change_operation_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::srv::ChangeOperationMode_Request>()
{
  return ::autoware_manual_control::srv::ChangeOperationMode_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace autoware_manual_control


namespace autoware_manual_control
{

namespace srv
{

namespace builder
{

class Init_ChangeOperationMode_Response_status
{
public:
  Init_ChangeOperationMode_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_manual_control::srv::ChangeOperationMode_Response status(::autoware_manual_control::srv::ChangeOperationMode_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::srv::ChangeOperationMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::srv::ChangeOperationMode_Response>()
{
  return autoware_manual_control::srv::builder::Init_ChangeOperationMode_Response_status();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__BUILDER_HPP_
