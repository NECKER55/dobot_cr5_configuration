// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_messages:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__MAP__BUILDER_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_messages/msg/detail/map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_messages
{

namespace msg
{

namespace builder
{

class Init_Map_objects
{
public:
  explicit Init_Map_objects(::custom_messages::msg::Map & msg)
  : msg_(msg)
  {}
  ::custom_messages::msg::Map objects(::custom_messages::msg::Map::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_messages::msg::Map msg_;
};

class Init_Map_work_space
{
public:
  Init_Map_work_space()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Map_objects work_space(::custom_messages::msg::Map::_work_space_type arg)
  {
    msg_.work_space = std::move(arg);
    return Init_Map_objects(msg_);
  }

private:
  ::custom_messages::msg::Map msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_messages::msg::Map>()
{
  return custom_messages::msg::builder::Init_Map_work_space();
}

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__MAP__BUILDER_HPP_
