// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/OperationModeState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__STRUCT_HPP_

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
# define DEPRECATED__autoware_manual_control__msg__OperationModeState __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__OperationModeState __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OperationModeState_
{
  using Type = OperationModeState_<ContainerAllocator>;

  explicit OperationModeState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->is_autoware_control_enabled = false;
      this->is_in_transition = false;
      this->is_stop_mode_available = false;
      this->is_autonomous_mode_available = false;
      this->is_local_mode_available = false;
      this->is_remote_mode_available = false;
    }
  }

  explicit OperationModeState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->is_autoware_control_enabled = false;
      this->is_in_transition = false;
      this->is_stop_mode_available = false;
      this->is_autonomous_mode_available = false;
      this->is_local_mode_available = false;
      this->is_remote_mode_available = false;
    }
  }

  // field types and members
  using _stamp_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _is_autoware_control_enabled_type =
    bool;
  _is_autoware_control_enabled_type is_autoware_control_enabled;
  using _is_in_transition_type =
    bool;
  _is_in_transition_type is_in_transition;
  using _is_stop_mode_available_type =
    bool;
  _is_stop_mode_available_type is_stop_mode_available;
  using _is_autonomous_mode_available_type =
    bool;
  _is_autonomous_mode_available_type is_autonomous_mode_available;
  using _is_local_mode_available_type =
    bool;
  _is_local_mode_available_type is_local_mode_available;
  using _is_remote_mode_available_type =
    bool;
  _is_remote_mode_available_type is_remote_mode_available;

  // setters for named parameter idiom
  Type & set__stamp(
    const autoware_manual_control::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__is_autoware_control_enabled(
    const bool & _arg)
  {
    this->is_autoware_control_enabled = _arg;
    return *this;
  }
  Type & set__is_in_transition(
    const bool & _arg)
  {
    this->is_in_transition = _arg;
    return *this;
  }
  Type & set__is_stop_mode_available(
    const bool & _arg)
  {
    this->is_stop_mode_available = _arg;
    return *this;
  }
  Type & set__is_autonomous_mode_available(
    const bool & _arg)
  {
    this->is_autonomous_mode_available = _arg;
    return *this;
  }
  Type & set__is_local_mode_available(
    const bool & _arg)
  {
    this->is_local_mode_available = _arg;
    return *this;
  }
  Type & set__is_remote_mode_available(
    const bool & _arg)
  {
    this->is_remote_mode_available = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t UNKNOWN =
    0u;
  static constexpr uint8_t STOP =
    1u;
  static constexpr uint8_t AUTONOMOUS =
    2u;
  static constexpr uint8_t LOCAL =
    3u;
  static constexpr uint8_t REMOTE =
    4u;

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::OperationModeState_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::OperationModeState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::OperationModeState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::OperationModeState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__OperationModeState
    std::shared_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__OperationModeState
    std::shared_ptr<autoware_manual_control::msg::OperationModeState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OperationModeState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->is_autoware_control_enabled != other.is_autoware_control_enabled) {
      return false;
    }
    if (this->is_in_transition != other.is_in_transition) {
      return false;
    }
    if (this->is_stop_mode_available != other.is_stop_mode_available) {
      return false;
    }
    if (this->is_autonomous_mode_available != other.is_autonomous_mode_available) {
      return false;
    }
    if (this->is_local_mode_available != other.is_local_mode_available) {
      return false;
    }
    if (this->is_remote_mode_available != other.is_remote_mode_available) {
      return false;
    }
    return true;
  }
  bool operator!=(const OperationModeState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OperationModeState_

// alias to use template instance with default allocator
using OperationModeState =
  autoware_manual_control::msg::OperationModeState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperationModeState_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperationModeState_<ContainerAllocator>::STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperationModeState_<ContainerAllocator>::AUTONOMOUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperationModeState_<ContainerAllocator>::LOCAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t OperationModeState_<ContainerAllocator>::REMOTE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__OPERATION_MODE_STATE__STRUCT_HPP_
