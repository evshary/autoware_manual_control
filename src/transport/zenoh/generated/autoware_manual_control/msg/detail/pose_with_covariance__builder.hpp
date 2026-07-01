// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/PoseWithCovariance.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/pose_with_covariance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_PoseWithCovariance_covariance
{
public:
  explicit Init_PoseWithCovariance_covariance(::autoware_manual_control::msg::PoseWithCovariance & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::PoseWithCovariance covariance(::autoware_manual_control::msg::PoseWithCovariance::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::PoseWithCovariance msg_;
};

class Init_PoseWithCovariance_pose
{
public:
  Init_PoseWithCovariance_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseWithCovariance_covariance pose(::autoware_manual_control::msg::PoseWithCovariance::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_PoseWithCovariance_covariance(msg_);
  }

private:
  ::autoware_manual_control::msg::PoseWithCovariance msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::PoseWithCovariance>()
{
  return autoware_manual_control::msg::builder::Init_PoseWithCovariance_pose();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__BUILDER_HPP_
