// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:msg/GearCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_COMMAND__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__autoware_manual_control__msg__GearCommand __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__msg__GearCommand __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GearCommand_
{
  using Type = GearCommand_<ContainerAllocator>;

  explicit GearCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  explicit GearCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  // field types and members
  using _stamp_type =
    autoware_manual_control::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _command_type =
    uint8_t;
  _command_type command;

  // setters for named parameter idiom
  Type & set__stamp(
    const autoware_manual_control::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NONE =
    0u;
  static constexpr uint8_t NEUTRAL =
    1u;
  static constexpr uint8_t DRIVE =
    2u;
  static constexpr uint8_t DRIVE_2 =
    3u;
  static constexpr uint8_t DRIVE_3 =
    4u;
  static constexpr uint8_t DRIVE_4 =
    5u;
  static constexpr uint8_t DRIVE_5 =
    6u;
  static constexpr uint8_t DRIVE_6 =
    7u;
  static constexpr uint8_t DRIVE_7 =
    8u;
  static constexpr uint8_t DRIVE_8 =
    9u;
  static constexpr uint8_t DRIVE_9 =
    10u;
  static constexpr uint8_t DRIVE_10 =
    11u;
  static constexpr uint8_t DRIVE_11 =
    12u;
  static constexpr uint8_t DRIVE_12 =
    13u;
  static constexpr uint8_t DRIVE_13 =
    14u;
  static constexpr uint8_t DRIVE_14 =
    15u;
  static constexpr uint8_t DRIVE_15 =
    16u;
  static constexpr uint8_t DRIVE_16 =
    17u;
  static constexpr uint8_t DRIVE_17 =
    18u;
  static constexpr uint8_t DRIVE_18 =
    19u;
  static constexpr uint8_t REVERSE =
    20u;
  static constexpr uint8_t REVERSE_2 =
    21u;
  static constexpr uint8_t PARK =
    22u;
  static constexpr uint8_t LOW =
    23u;
  static constexpr uint8_t LOW_2 =
    24u;

  // pointer types
  using RawPtr =
    autoware_manual_control::msg::GearCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::msg::GearCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::GearCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::msg::GearCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__msg__GearCommand
    std::shared_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__msg__GearCommand
    std::shared_ptr<autoware_manual_control::msg::GearCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GearCommand_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    return true;
  }
  bool operator!=(const GearCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GearCommand_

// alias to use template instance with default allocator
using GearCommand =
  autoware_manual_control::msg::GearCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::NEUTRAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_3;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_4;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_5;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_6;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_7;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_8;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_9;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_10;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_11;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_12;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_13;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_14;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_15;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_16;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_17;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::DRIVE_18;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::REVERSE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::REVERSE_2;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::PARK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::LOW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GearCommand_<ContainerAllocator>::LOW_2;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__GEAR_COMMAND__STRUCT_HPP_
