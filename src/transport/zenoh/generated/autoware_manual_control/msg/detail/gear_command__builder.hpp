// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/GearCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_COMMAND__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/gear_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_GearCommand_command
{
public:
  explicit Init_GearCommand_command(::autoware_manual_control::msg::GearCommand & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::GearCommand command(::autoware_manual_control::msg::GearCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::GearCommand msg_;
};

class Init_GearCommand_stamp
{
public:
  Init_GearCommand_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GearCommand_command stamp(::autoware_manual_control::msg::GearCommand::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_GearCommand_command(msg_);
  }

private:
  ::autoware_manual_control::msg::GearCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::GearCommand>()
{
  return autoware_manual_control::msg::builder::Init_GearCommand_stamp();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_COMMAND__BUILDER_HPP_
