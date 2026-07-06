// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/Longitudinal.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
// Member 'control_time'
#include "autoware_manual_control/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_manual_control__msg__Longitudinal __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__Longitudinal __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Longitudinal_
{
  using Type = Longitudinal_<ContainerAllocator>;

  explicit Longitudinal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    control_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->acceleration = 0.0f;
      this->jerk = 0.0f;
      this->is_defined_acceleration = false;
      this->is_defined_jerk = false;
    }
  }

  explicit Longitudinal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    control_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->acceleration = 0.0f;
      this->jerk = 0.0f;
      this->is_defined_acceleration = false;
      this->is_defined_jerk = false;
    }
  }

  // field types and members
  using _stamp_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _control_time_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _control_time_type control_time;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _acceleration_type =
    float;
  _acceleration_type acceleration;
  using _jerk_type =
    float;
  _jerk_type jerk;
  using _is_defined_acceleration_type =
    bool;
  _is_defined_acceleration_type is_defined_acceleration;
  using _is_defined_jerk_type =
    bool;
  _is_defined_jerk_type is_defined_jerk;

  // setters for named parameter idiom
  Type & set__stamp(
    const autoware_manual_control::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__control_time(
    const autoware_manual_control::msg::Time_<ContainerAllocator> & _arg)
  {
    this->control_time = _arg;
    return *this;
  }
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const float & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__jerk(
    const float & _arg)
  {
    this->jerk = _arg;
    return *this;
  }
  Type & set__is_defined_acceleration(
    const bool & _arg)
  {
    this->is_defined_acceleration = _arg;
    return *this;
  }
  Type & set__is_defined_jerk(
    const bool & _arg)
  {
    this->is_defined_jerk = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::Longitudinal_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::Longitudinal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::Longitudinal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::Longitudinal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__Longitudinal
    std::shared_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__Longitudinal
    std::shared_ptr<autoware_manual_control::msg::Longitudinal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Longitudinal_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->control_time != other.control_time) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->jerk != other.jerk) {
      return false;
    }
    if (this->is_defined_acceleration != other.is_defined_acceleration) {
      return false;
    }
    if (this->is_defined_jerk != other.is_defined_jerk) {
      return false;
    }
    return true;
  }
  bool operator!=(const Longitudinal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Longitudinal_

// alias to use template instance with default allocator
using Longitudinal =
  autoware_manual_control::msg::Longitudinal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LONGITUDINAL__STRUCT_HPP_
