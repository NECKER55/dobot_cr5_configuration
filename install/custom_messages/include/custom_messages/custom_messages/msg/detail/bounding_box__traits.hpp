// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_messages:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_messages/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'low_left'
// Member 'top_right'
#include "custom_messages/msg/detail/point__traits.hpp"

namespace custom_messages
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoundingBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: low_left
  {
    out << "low_left: ";
    to_flow_style_yaml(msg.low_left, out);
    out << ", ";
  }

  // member: top_right
  {
    out << "top_right: ";
    to_flow_style_yaml(msg.top_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: low_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "low_left:\n";
    to_block_style_yaml(msg.low_left, out, indentation + 2);
  }

  // member: top_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "top_right:\n";
    to_block_style_yaml(msg.top_right, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoundingBox & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_messages

namespace rosidl_generator_traits
{

[[deprecated("use custom_messages::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_messages::msg::BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_messages::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_messages::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_messages::msg::BoundingBox & msg)
{
  return custom_messages::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_messages::msg::BoundingBox>()
{
  return "custom_messages::msg::BoundingBox";
}

template<>
inline const char * name<custom_messages::msg::BoundingBox>()
{
  return "custom_messages/msg/BoundingBox";
}

template<>
struct has_fixed_size<custom_messages::msg::BoundingBox>
  : std::integral_constant<bool, has_fixed_size<custom_messages::msg::Point>::value> {};

template<>
struct has_bounded_size<custom_messages::msg::BoundingBox>
  : std::integral_constant<bool, has_bounded_size<custom_messages::msg::Point>::value> {};

template<>
struct is_message<custom_messages::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
