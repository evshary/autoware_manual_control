// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/OperationModeState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/operation_mode_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_OperationModeState_is_remote_mode_available
{
public:
  explicit Init_OperationModeState_is_remote_mode_available(::autoware_manual_control::msg::OperationModeState & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::OperationModeState is_remote_mode_available(::autoware_manual_control::msg::OperationModeState::_is_remote_mode_available_type arg)
  {
    msg_.is_remote_mode_available = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

class Init_OperationModeState_is_local_mode_available
{
public:
  explicit Init_OperationModeState_is_local_mode_available(::autoware_manual_control::msg::OperationModeState & msg)
  : msg_(msg)
  {}
  Init_OperationModeState_is_remote_mode_available is_local_mode_available(::autoware_manual_control::msg::OperationModeState::_is_local_mode_available_type arg)
  {
    msg_.is_local_mode_available = std::move(arg);
    return Init_OperationModeState_is_remote_mode_available(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

class Init_OperationModeState_is_autonomous_mode_available
{
public:
  explicit Init_OperationModeState_is_autonomous_mode_available(::autoware_manual_control::msg::OperationModeState & msg)
  : msg_(msg)
  {}
  Init_OperationModeState_is_local_mode_available is_autonomous_mode_available(::autoware_manual_control::msg::OperationModeState::_is_autonomous_mode_available_type arg)
  {
    msg_.is_autonomous_mode_available = std::move(arg);
    return Init_OperationModeState_is_local_mode_available(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

class Init_OperationModeState_is_stop_mode_available
{
public:
  explicit Init_OperationModeState_is_stop_mode_available(::autoware_manual_control::msg::OperationModeState & msg)
  : msg_(msg)
  {}
  Init_OperationModeState_is_autonomous_mode_available is_stop_mode_available(::autoware_manual_control::msg::OperationModeState::_is_stop_mode_available_type arg)
  {
    msg_.is_stop_mode_available = std::move(arg);
    return Init_OperationModeState_is_autonomous_mode_available(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

class Init_OperationModeState_is_in_transition
{
public:
  explicit Init_OperationModeState_is_in_transition(::autoware_manual_control::msg::OperationModeState & msg)
  : msg_(msg)
  {}
  Init_OperationModeState_is_stop_mode_available is_in_transition(::autoware_manual_control::msg::OperationModeState::_is_in_transition_type arg)
  {
    msg_.is_in_transition = std::move(arg);
    return Init_OperationModeState_is_stop_mode_available(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

class Init_OperationModeState_is_autoware_control_enabled
{
public:
  explicit Init_OperationModeState_is_autoware_control_enabled(::autoware_manual_control::msg::OperationModeState & msg)
  : msg_(msg)
  {}
  Init_OperationModeState_is_in_transition is_autoware_control_enabled(::autoware_manual_control::msg::OperationModeState::_is_autoware_control_enabled_type arg)
  {
    msg_.is_autoware_control_enabled = std::move(arg);
    return Init_OperationModeState_is_in_transition(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

class Init_OperationModeState_mode
{
public:
  explicit Init_OperationModeState_mode(::autoware_manual_control::msg::OperationModeState & msg)
  : msg_(msg)
  {}
  Init_OperationModeState_is_autoware_control_enabled mode(::autoware_manual_control::msg::OperationModeState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_OperationModeState_is_autoware_control_enabled(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

class Init_OperationModeState_stamp
{
public:
  Init_OperationModeState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OperationModeState_mode stamp(::autoware_manual_control::msg::OperationModeState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_OperationModeState_mode(msg_);
  }

private:
  ::autoware_manual_control::msg::OperationModeState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::OperationModeState>()
{
  return autoware_manual_control::msg::builder::Init_OperationModeState_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__BUILDER_HPP_
