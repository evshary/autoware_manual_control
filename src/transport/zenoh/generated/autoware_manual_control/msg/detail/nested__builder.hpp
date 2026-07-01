// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/Nested.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__NESTED__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__NESTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/nested__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_Nested_basic_types_value
{
public:
  Init_Nested_basic_types_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::autoware_manual_control::msg::Nested basic_types_value(::autoware_manual_control::msg::Nested::_basic_types_value_type arg)
  {
    msg_.basic_types_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::Nested msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::Nested>()
{
  return autoware_manual_control::msg::builder::Init_Nested_basic_types_value();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__NESTED__BUILDER_HPP_
