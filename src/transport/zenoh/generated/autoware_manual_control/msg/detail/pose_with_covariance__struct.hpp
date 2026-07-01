// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/PoseWithCovariance.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "autoware_manual_control/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_manual_control__msg__PoseWithCovariance __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__PoseWithCovariance __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseWithCovariance_
{
  using Type = PoseWithCovariance_<ContainerAllocator>;

  explicit PoseWithCovariance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  explicit PoseWithCovariance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  // field types and members
  using _pose_type =
    autoware_manual_control::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _covariance_type =
    std::array<double, 36>;
  _covariance_type covariance;

  // setters for named parameter idiom
  Type & set__pose(
    const autoware_manual_control::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 36> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__PoseWithCovariance
    std::shared_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__PoseWithCovariance
    std::shared_ptr<autoware_manual_control::msg::PoseWithCovariance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseWithCovariance_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseWithCovariance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseWithCovariance_

// alias to use template instance with default allocator
using PoseWithCovariance =
  autoware_manual_control::msg::PoseWithCovariance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE_WITH_COVARIANCE__STRUCT_HPP_
