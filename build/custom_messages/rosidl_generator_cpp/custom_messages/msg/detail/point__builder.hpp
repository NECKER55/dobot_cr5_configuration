// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__POINT__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_Point_z
{
public:
  explicit Init_Point_z(::custom_messages::msg::Point & msg)
  : msg_(msg)
  {}
  ::custom_messages::msg::Point z(::custom_messages::msg::Point::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::Point msg_;
};

class Init_Point_y
{
public:
  explicit Init_Point_y(::custom_messages::msg::Point & msg)
  : msg_(msg)
  {}
  Init_Point_z y(::custom_messages::msg::Point::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Point_z(msg_);
  }

private:
  ::custom_messages::msg::Point msg_;
};

class Init_Point_x
{
public:
  Init_Point_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point_y x(::custom_messages::msg::Point::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Point_y(msg_);
  }

private:
  ::custom_messages::msg::Point msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::Point>()
{
  return custom_messages::msg::builder::Init_Point_x();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__POINT__BUILDER_HPP_
