// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/LocalizationInitializationState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LOCALIZATION_INITIALIZATION_STATE__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LOCALIZATION_INITIALIZATION_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/localization_initialization_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_LocalizationInitializationState_state
{
public:
  explicit Init_LocalizationInitializationState_state(::autoware_manual_control::msg::LocalizationInitializationState & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::LocalizationInitializationState state(::autoware_manual_control::msg::LocalizationInitializationState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::LocalizationInitializationState msg_;
};

class Init_LocalizationInitializationState_stamp
{
public:
  Init_LocalizationInitializationState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalizationInitializationState_state stamp(::autoware_manual_control::msg::LocalizationInitializationState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_LocalizationInitializationState_state(msg_);
  }

private:
  ::autoware_manual_control::msg::LocalizationInitializationState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::LocalizationInitializationState>()
{
  return autoware_manual_control::msg::builder::Init_LocalizationInitializationState_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LOCALIZATION_INITIALIZATION_STATE__BUILDER_HPP_
