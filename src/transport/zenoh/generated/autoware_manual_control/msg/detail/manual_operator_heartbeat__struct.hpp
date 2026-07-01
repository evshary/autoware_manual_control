// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/ManualOperatorHeartbeat.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__STRUCT_HPP_

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
# define DEPRECATED__autoware_manual_control__msg__ManualOperatorHeartbeat __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__ManualOperatorHeartbeat __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ManualOperatorHeartbeat_
{
  using Type = ManualOperatorHeartbeat_<ContainerAllocator>;

  explicit ManualOperatorHeartbeat_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ready = false;
    }
  }

  explicit ManualOperatorHeartbeat_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ready = false;
    }
  }

  // field types and members
  using _stamp_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _ready_type =
    bool;
  _ready_type ready;

  // setters for named parameter idiom
  Type & set__stamp(
    const autoware_manual_control::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__ready(
    const bool & _arg)
  {
    this->ready = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__ManualOperatorHeartbeat
    std::shared_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__ManualOperatorHeartbeat
    std::shared_ptr<autoware_manual_control::msg::ManualOperatorHeartbeat_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManualOperatorHeartbeat_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->ready != other.ready) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManualOperatorHeartbeat_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManualOperatorHeartbeat_

// alias to use template instance with default allocator
using ManualOperatorHeartbeat =
  autoware_manual_control::msg::ManualOperatorHeartbeat_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MANUAL_OPERATOR_HEARTBEAT__STRUCT_HPP_
