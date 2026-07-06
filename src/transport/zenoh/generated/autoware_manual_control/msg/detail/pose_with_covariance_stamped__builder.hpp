// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/PoseWithCovarianceStamped.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE_STAMPED__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/pose_with_covariance_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_PoseWithCovarianceStamped_pose
{
public:
  explicit Init_PoseWithCovarianceStamped_pose(::autoware_manual_control::msg::PoseWithCovarianceStamped & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::PoseWithCovarianceStamped pose(::autoware_manual_control::msg::PoseWithCovarianceStamped::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::PoseWithCovarianceStamped msg_;
};

class Init_PoseWithCovarianceStamped_header
{
public:
  Init_PoseWithCovarianceStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseWithCovarianceStamped_pose header(::autoware_manual_control::msg::PoseWithCovarianceStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PoseWithCovarianceStamped_pose(msg_);
  }

private:
  ::autoware_manual_control::msg::PoseWithCovarianceStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::PoseWithCovarianceStamped>()
{
  return autoware_manual_control::msg::builder::Init_PoseWithCovarianceStamped_header();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE_STAMPED__BUILDER_HPP_
