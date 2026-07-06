// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/SteeringReport.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__STEERING_REPORT__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__STEERING_REPORT__STRUCT_HPP_

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
#include "autoware_manual_control/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_manual_control__msg__SteeringReport __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__SteeringReport __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SteeringReport_
{
  using Type = SteeringReport_<ContainerAllocator>;

  explicit SteeringReport_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering_tire_angle = 0.0f;
    }
  }

  explicit SteeringReport_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering_tire_angle = 0.0f;
    }
  }

  // field types and members
  using _stamp_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _steering_tire_angle_type =
    float;
  _steering_tire_angle_type steering_tire_angle;

  // setters for named parameter idiom
  Type & set__stamp(
    const autoware_manual_control::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__steering_tire_angle(
    const float & _arg)
  {
    this->steering_tire_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::SteeringReport_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::SteeringReport_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::SteeringReport_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::SteeringReport_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__SteeringReport
    std::shared_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__SteeringReport
    std::shared_ptr<autoware_manual_control::msg::SteeringReport_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SteeringReport_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->steering_tire_angle != other.steering_tire_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const SteeringReport_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SteeringReport_

// alias to use template instance with default allocator
using SteeringReport =
  autoware_manual_control::msg::SteeringReport_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__STEERING_REPORT__STRUCT_HPP_
