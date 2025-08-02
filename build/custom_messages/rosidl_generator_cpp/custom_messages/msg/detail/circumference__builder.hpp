// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/Circumference.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/circumference__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_Circumference_circumference
{
public:
  Init_Circumference_circumference()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_messages::msg::Circumference circumference(::custom_messages::msg::Circumference::_circumference_type arg)
  {
    msg_.circumference = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::Circumference msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::Circumference>()
{
  return custom_messages::msg::builder::Init_Circumference_circumference();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__BUILDER_HPP_
