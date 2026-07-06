// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/VelocityReport.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "autoware_manual_control/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_manual_control__msg__VelocityReport __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__VelocityReport __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelocityReport_
{
  using Type = VelocityReport_<ContainerAllocator>;

  explicit VelocityReport_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitudinal_velocity = 0.0f;
      this->lateral_velocity = 0.0f;
      this->heading_rate = 0.0f;
    }
  }

  explicit VelocityReport_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitudinal_velocity = 0.0f;
      this->lateral_velocity = 0.0f;
      this->heading_rate = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    autoware_manual_control::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _longitudinal_velocity_type =
    float;
  _longitudinal_velocity_type longitudinal_velocity;
  using _lateral_velocity_type =
    float;
  _lateral_velocity_type lateral_velocity;
  using _heading_rate_type =
    float;
  _heading_rate_type heading_rate;

  // setters for named parameter idiom
  Type & set__header(
    const autoware_manual_control::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__longitudinal_velocity(
    const float & _arg)
  {
    this->longitudinal_velocity = _arg;
    return *this;
  }
  Type & set__lateral_velocity(
    const float & _arg)
  {
    this->lateral_velocity = _arg;
    return *this;
  }
  Type & set__heading_rate(
    const float & _arg)
  {
    this->heading_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::VelocityReport_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::VelocityReport_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::VelocityReport_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::VelocityReport_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__VelocityReport
    std::shared_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__VelocityReport
    std::shared_ptr<autoware_manual_control::msg::VelocityReport_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelocityReport_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->longitudinal_velocity != other.longitudinal_velocity) {
      return false;
    }
    if (this->lateral_velocity != other.lateral_velocity) {
      return false;
    }
    if (this->heading_rate != other.heading_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelocityReport_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelocityReport_

// alias to use template instance with default allocator
using VelocityReport =
  autoware_manual_control::msg::VelocityReport_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__VELOCITY_REPORT__STRUCT_HPP_
