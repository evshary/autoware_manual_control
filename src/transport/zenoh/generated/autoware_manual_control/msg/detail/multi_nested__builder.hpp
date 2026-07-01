// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from autoware_manual_control:msg/MultiNested.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MULTI_NESTED__BUILDER_HPP_
#define AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MULTI_NESTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "autoware_manual_control/msg/detail/multi_nested__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace autoware_manual_control
{

namespace msg
{

namespace builder
{

class Init_MultiNested_unbounded_sequence_of_unbounded_sequences
{
public:
  explicit Init_MultiNested_unbounded_sequence_of_unbounded_sequences(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  ::autoware_manual_control::msg::MultiNested unbounded_sequence_of_unbounded_sequences(::autoware_manual_control::msg::MultiNested::_unbounded_sequence_of_unbounded_sequences_type arg)
  {
    msg_.unbounded_sequence_of_unbounded_sequences = std::move(arg);
    return std::move(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_unbounded_sequence_of_bounded_sequences
{
public:
  explicit Init_MultiNested_unbounded_sequence_of_bounded_sequences(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  Init_MultiNested_unbounded_sequence_of_unbounded_sequences unbounded_sequence_of_bounded_sequences(::autoware_manual_control::msg::MultiNested::_unbounded_sequence_of_bounded_sequences_type arg)
  {
    msg_.unbounded_sequence_of_bounded_sequences = std::move(arg);
    return Init_MultiNested_unbounded_sequence_of_unbounded_sequences(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_unbounded_sequence_of_arrays
{
public:
  explicit Init_MultiNested_unbounded_sequence_of_arrays(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  Init_MultiNested_unbounded_sequence_of_bounded_sequences unbounded_sequence_of_arrays(::autoware_manual_control::msg::MultiNested::_unbounded_sequence_of_arrays_type arg)
  {
    msg_.unbounded_sequence_of_arrays = std::move(arg);
    return Init_MultiNested_unbounded_sequence_of_bounded_sequences(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_bounded_sequence_of_unbounded_sequences
{
public:
  explicit Init_MultiNested_bounded_sequence_of_unbounded_sequences(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  Init_MultiNested_unbounded_sequence_of_arrays bounded_sequence_of_unbounded_sequences(::autoware_manual_control::msg::MultiNested::_bounded_sequence_of_unbounded_sequences_type arg)
  {
    msg_.bounded_sequence_of_unbounded_sequences = std::move(arg);
    return Init_MultiNested_unbounded_sequence_of_arrays(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_bounded_sequence_of_bounded_sequences
{
public:
  explicit Init_MultiNested_bounded_sequence_of_bounded_sequences(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  Init_MultiNested_bounded_sequence_of_unbounded_sequences bounded_sequence_of_bounded_sequences(::autoware_manual_control::msg::MultiNested::_bounded_sequence_of_bounded_sequences_type arg)
  {
    msg_.bounded_sequence_of_bounded_sequences = std::move(arg);
    return Init_MultiNested_bounded_sequence_of_unbounded_sequences(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_bounded_sequence_of_arrays
{
public:
  explicit Init_MultiNested_bounded_sequence_of_arrays(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  Init_MultiNested_bounded_sequence_of_bounded_sequences bounded_sequence_of_arrays(::autoware_manual_control::msg::MultiNested::_bounded_sequence_of_arrays_type arg)
  {
    msg_.bounded_sequence_of_arrays = std::move(arg);
    return Init_MultiNested_bounded_sequence_of_bounded_sequences(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_array_of_unbounded_sequences
{
public:
  explicit Init_MultiNested_array_of_unbounded_sequences(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  Init_MultiNested_bounded_sequence_of_arrays array_of_unbounded_sequences(::autoware_manual_control::msg::MultiNested::_array_of_unbounded_sequences_type arg)
  {
    msg_.array_of_unbounded_sequences = std::move(arg);
    return Init_MultiNested_bounded_sequence_of_arrays(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_array_of_bounded_sequences
{
public:
  explicit Init_MultiNested_array_of_bounded_sequences(::autoware_manual_control::msg::MultiNested & msg)
  : msg_(msg)
  {}
  Init_MultiNested_array_of_unbounded_sequences array_of_bounded_sequences(::autoware_manual_control::msg::MultiNested::_array_of_bounded_sequences_type arg)
  {
    msg_.array_of_bounded_sequences = std::move(arg);
    return Init_MultiNested_array_of_unbounded_sequences(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

class Init_MultiNested_array_of_arrays
{
public:
  Init_MultiNested_array_of_arrays()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiNested_array_of_bounded_sequences array_of_arrays(::autoware_manual_control::msg::MultiNested::_array_of_arrays_type arg)
  {
    msg_.array_of_arrays = std::move(arg);
    return Init_MultiNested_array_of_bounded_sequences(msg_);
  }

private:
  ::autoware_manual_control::msg::MultiNested msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::autoware_manual_control::msg::MultiNested>()
{
  return autoware_manual_control::msg::builder::Init_MultiNested_array_of_arrays();
}

}  // namespace autoware_manual_control

#endif  // AUTOWARE_MANUAL_CONTROL__MSG__DETAIL__MULTI_NESTED__BUILDER_HPP_
