// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_top_right
{
public:
  explicit Init_BoundingBox_top_right(::custom_messages::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::custom_messages::msg::BoundingBox top_right(::custom_messages::msg::BoundingBox::_top_right_type arg)
  {
    msg_.top_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::BoundingBox msg_;
};

class Init_BoundingBox_low_left
{
public:
  Init_BoundingBox_low_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_top_right low_left(::custom_messages::msg::BoundingBox::_low_left_type arg)
  {
    msg_.low_left = std::move(arg);
    return Init_BoundingBox_top_right(msg_);
  }

private:
  ::custom_messages::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::BoundingBox>()
{
  return custom_messages::msg::builder::Init_BoundingBox_low_left();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
