// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_Object_possible_trajectories
{
public:
  explicit Init_Object_possible_trajectories(::custom_messages::msg::Object & msg)
  : msg_(msg)
  {}
  ::custom_messages::msg::Object possible_trajectories(::custom_messages::msg::Object::_possible_trajectories_type arg)
  {
    msg_.possible_trajectories = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::Object msg_;
};

class Init_Object_shape
{
public:
  explicit Init_Object_shape(::custom_messages::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_possible_trajectories shape(::custom_messages::msg::Object::_shape_type arg)
  {
    msg_.shape = std::move(arg);
    return Init_Object_possible_trajectories(msg_);
  }

private:
  ::custom_messages::msg::Object msg_;
};

class Init_Object_target
{
public:
  Init_Object_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_shape target(::custom_messages::msg::Object::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_Object_shape(msg_);
  }

private:
  ::custom_messages::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::Object>()
{
  return custom_messages::msg::builder::Init_Object_target();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__BUILDER_HPP_
