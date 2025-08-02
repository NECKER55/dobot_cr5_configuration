// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/OptimalPoint.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__OPTIMAL_POINT__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__OPTIMAL_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/optimal_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_OptimalPoint_optimality
{
public:
  explicit Init_OptimalPoint_optimality(::custom_messages::msg::OptimalPoint & msg)
  : msg_(msg)
  {}
  ::custom_messages::msg::OptimalPoint optimality(::custom_messages::msg::OptimalPoint::_optimality_type arg)
  {
    msg_.optimality = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::OptimalPoint msg_;
};

class Init_OptimalPoint_z
{
public:
  explicit Init_OptimalPoint_z(::custom_messages::msg::OptimalPoint & msg)
  : msg_(msg)
  {}
  Init_OptimalPoint_optimality z(::custom_messages::msg::OptimalPoint::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_OptimalPoint_optimality(msg_);
  }

private:
  ::custom_messages::msg::OptimalPoint msg_;
};

class Init_OptimalPoint_y
{
public:
  explicit Init_OptimalPoint_y(::custom_messages::msg::OptimalPoint & msg)
  : msg_(msg)
  {}
  Init_OptimalPoint_z y(::custom_messages::msg::OptimalPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_OptimalPoint_z(msg_);
  }

private:
  ::custom_messages::msg::OptimalPoint msg_;
};

class Init_OptimalPoint_x
{
public:
  Init_OptimalPoint_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OptimalPoint_y x(::custom_messages::msg::OptimalPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_OptimalPoint_y(msg_);
  }

private:
  ::custom_messages::msg::OptimalPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::OptimalPoint>()
{
  return custom_messages::msg::builder::Init_OptimalPoint_x();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__OPTIMAL_POINT__BUILDER_HPP_
