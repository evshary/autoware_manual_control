// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/Lateral.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LATERAL__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LATERAL__STRUCT_HPP_

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
# define DEPRECATED__autoware_manual_control__msg__Lateral __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__Lateral __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Lateral_
{
  using Type = Lateral_<ContainerAllocator>;

  explicit Lateral_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    control_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering_tire_angle = 0.0f;
      this->steering_tire_rotation_rate = 0.0f;
      this->is_defined_steering_tire_rotation_rate = false;
    }
  }

  explicit Lateral_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    control_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering_tire_angle = 0.0f;
      this->steering_tire_rotation_rate = 0.0f;
      this->is_defined_steering_tire_rotation_rate = false;
    }
  }

  // field types and members
  using _stamp_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _control_time_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _control_time_type control_time;
  using _steering_tire_angle_type =
    float;
  _steering_tire_angle_type steering_tire_angle;
  using _steering_tire_rotation_rate_type =
    float;
  _steering_tire_rotation_rate_type steering_tire_rotation_rate;
  using _is_defined_steering_tire_rotation_rate_type =
    bool;
  _is_defined_steering_tire_rotation_rate_type is_defined_steering_tire_rotation_rate;

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
  Type & set__steering_tire_angle(
    const float & _arg)
  {
    this->steering_tire_angle = _arg;
    return *this;
  }
  Type & set__steering_tire_rotation_rate(
    const float & _arg)
  {
    this->steering_tire_rotation_rate = _arg;
    return *this;
  }
  Type & set__is_defined_steering_tire_rotation_rate(
    const bool & _arg)
  {
    this->is_defined_steering_tire_rotation_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::Lateral_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::Lateral_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::Lateral_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::Lateral_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__Lateral
    std::shared_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__Lateral
    std::shared_ptr<autoware_manual_control::msg::Lateral_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Lateral_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->control_time != other.control_time) {
      return false;
    }
    if (this->steering_tire_angle != other.steering_tire_angle) {
      return false;
    }
    if (this->steering_tire_rotation_rate != other.steering_tire_rotation_rate) {
      return false;
    }
    if (this->is_defined_steering_tire_rotation_rate != other.is_defined_steering_tire_rotation_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const Lateral_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Lateral_

// alias to use template instance with default allocator
using Lateral =
  autoware_manual_control::msg::Lateral_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LATERAL__STRUCT_HPP_
