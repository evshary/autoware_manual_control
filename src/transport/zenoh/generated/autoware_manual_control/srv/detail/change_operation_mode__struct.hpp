// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from autoware_manual_control:srv/ChangeOperationMode.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__STRUCT_HPP_
#define AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Request __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ChangeOperationMode_Request_
{
  using Type = ChangeOperationMode_Request_<ContainerAllocator>;

  explicit ChangeOperationMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ChangeOperationMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Request
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Request
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChangeOperationMode_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChangeOperationMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChangeOperationMode_Request_

// alias to use template instance with default allocator
using ChangeOperationMode_Request =
  autoware_manual_control::srv::ChangeOperationMode_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace autoware_manual_control


// Include directives for member types
// Member 'status'
#include "autoware_manual_control/msg/detail/response_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Response __declspec(deprecated)
#endif

namespace autoware_manual_control
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ChangeOperationMode_Response_
{
  using Type = ChangeOperationMode_Response_<ContainerAllocator>;

  explicit ChangeOperationMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_init)
  {
    (void)_init;
  }

  explicit ChangeOperationMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _status_type =
    autoware_manual_control::msg::ResponseStatus_<ContainerAllocator>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const autoware_manual_control::msg::ResponseStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t ERROR_NOT_AVAILABLE =
    1u;
  static constexpr uint16_t ERROR_IN_TRANSITION =
    2u;

  // pointer types
  using RawPtr =
    autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Response
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__autoware_manual_control__srv__ChangeOperationMode_Response
    std::shared_ptr<autoware_manual_control::srv::ChangeOperationMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChangeOperationMode_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChangeOperationMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChangeOperationMode_Response_

// alias to use template instance with default allocator
using ChangeOperationMode_Response =
  autoware_manual_control::srv::ChangeOperationMode_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChangeOperationMode_Response_<ContainerAllocator>::ERROR_NOT_AVAILABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t ChangeOperationMode_Response_<ContainerAllocator>::ERROR_IN_TRANSITION;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace autoware_manual_control

namespace autoware_manual_control
{

namespace srv
{

struct ChangeOperationMode
{
  using Request = autoware_manual_control::srv::ChangeOperationMode_Request;
  using Response = autoware_manual_control::srv::ChangeOperationMode_Response;
};

}  // namespace srv

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__SRV__DETAIL__CHANGE_OPERATION_MODE__STRUCT_HPP_
