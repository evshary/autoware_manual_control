// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/Quaternion.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__QUATERNION__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__QUATERNION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/quaternion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_Quaternion_w
{
public:
  explicit Init_Quaternion_w(::autoware_manual_control::msg::Quaternion & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::Quaternion w(::autoware_manual_control::msg::Quaternion::_w_type arg)
  {
    msg_.w = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::Quaternion msg_;
};

class Init_Quaternion_z
{
public:
  explicit Init_Quaternion_z(::autoware_manual_control::msg::Quaternion & msg)
  : msg_(msg)
  {}
  Init_Quaternion_w z(::autoware_manual_control::msg::Quaternion::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Quaternion_w(msg_);
  }

private:
  ::autoware_manual_control::msg::Quaternion msg_;
};

class Init_Quaternion_y
{
public:
  explicit Init_Quaternion_y(::autoware_manual_control::msg::Quaternion & msg)
  : msg_(msg)
  {}
  Init_Quaternion_z y(::autoware_manual_control::msg::Quaternion::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Quaternion_z(msg_);
  }

private:
  ::autoware_manual_control::msg::Quaternion msg_;
};

class Init_Quaternion_x
{
public:
  Init_Quaternion_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Quaternion_y x(::autoware_manual_control::msg::Quaternion::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Quaternion_y(msg_);
  }

private:
  ::autoware_manual_control::msg::Quaternion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::Quaternion>()
{
  return autoware_manual_control::msg::builder::Init_Quaternion_x();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__QUATERNION__BUILDER_HPP_
