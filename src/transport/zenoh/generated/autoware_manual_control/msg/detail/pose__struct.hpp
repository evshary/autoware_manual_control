// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "autoware_manual_control/msg/detail/point__struct.hpp"
// Member 'orientation'
#include "autoware_manual_control/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_manual_control__msg__Pose __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__Pose __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose_
{
  using Type = Pose_<ContainerAllocator>;

  explicit Pose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    orientation(_init)
  {
    (void)_init;
  }

  explicit Pose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init),
    orientation(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _position_type =
    autoware_manual_control::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _orientation_type =
    autoware_manual_control::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;

  // setters for named parameter idiom
  Type & set__position(
    const autoware_manual_control::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__orientation(
    const autoware_manual_control::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::Pose_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::Pose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::Pose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::Pose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__Pose
    std::shared_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__Pose
    std::shared_ptr<autoware_manual_control::msg::Pose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose_

// alias to use template instance with default allocator
using Pose =
  autoware_manual_control::msg::Pose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__POSE__STRUCT_HPP_
