// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/LocalizationInitializationState.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LOCALIZATION_INITIALIZATION_STATE__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LOCALIZATION_INITIALIZATION_STATE__STRUCT_HPP_

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
# define DEPRECATED__autoware_manual_control__msg__LocalizationInitializationState __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__LocalizationInitializationState __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalizationInitializationState_
{
  using Type = LocalizationInitializationState_<ContainerAllocator>;

  explicit LocalizationInitializationState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit LocalizationInitializationState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _stamp_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _state_type =
    uint16_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__stamp(
    const autoware_manual_control::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__state(
    const uint16_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t UNKNOWN =
    0u;
  static constexpr uint16_t UNINITIALIZED =
    1u;
  static constexpr uint16_t INITIALIZING =
    2u;
  static constexpr uint16_t INITIALIZED =
    3u;

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__LocalizationInitializationState
    std::shared_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__LocalizationInitializationState
    std::shared_ptr<autoware_manual_control::msg::LocalizationInitializationState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalizationInitializationState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalizationInitializationState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalizationInitializationState_

// alias to use template instance with default allocator
using LocalizationInitializationState =
  autoware_manual_control::msg::LocalizationInitializationState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t LocalizationInitializationState_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t LocalizationInitializationState_<ContainerAllocator>::UNINITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t LocalizationInitializationState_<ContainerAllocator>::INITIALIZING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t LocalizationInitializationState_<ContainerAllocator>::INITIALIZED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__LOCALIZATION_INITIALIZATION_STATE__STRUCT_HPP_
