// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POINT__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_Point_z
{
public:
  explicit Init_Point_z(::autoware_manual_control::msg::Point & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::Point z(::autoware_manual_control::msg::Point::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::Point msg_;
};

class Init_Point_y
{
public:
  explicit Init_Point_y(::autoware_manual_control::msg::Point & msg)
  : msg_(msg)
  {}
  Init_Point_z y(::autoware_manual_control::msg::Point::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Point_z(msg_);
  }

private:
  ::autoware_manual_control::msg::Point msg_;
};

class Init_Point_x
{
public:
  Init_Point_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point_y x(::autoware_manual_control::msg::Point::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Point_y(msg_);
  }

private:
  ::autoware_manual_control::msg::Point msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::Point>()
{
  return autoware_manual_control::msg::builder::Init_Point_x();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POINT__BUILDER_HPP_
