// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_messages:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'low_left'
// Member 'top_right'
#include "custom_messages/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_messages__msg__BoundingBox __attribute__((deprecated))
#else
# define DEPRECATED__custom_messages__msg__BoundingBox __declspec(deprecated)
#endif

namespace custom_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox_
{
  using Type = BoundingBox_<ContainerAllocator>;

  explicit BoundingBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : low_left(_init),
    top_right(_init)
  {
    (void)_init;
  }

  explicit BoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : low_left(_alloc, _init),
    top_right(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _low_left_type =
    custom_messages::msg::Point_<ContainerAllocator>;
  _low_left_type low_left;
  using _top_right_type =
    custom_messages::msg::Point_<ContainerAllocator>;
  _top_right_type top_right;

  // setters for named parameter idiom
  Type & set__low_left(
    const custom_messages::msg::Point_<ContainerAllocator> & _arg)
  {
    this->low_left = _arg;
    return *this;
  }
  Type & set__top_right(
    const custom_messages::msg::Point_<ContainerAllocator> & _arg)
  {
    this->top_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_messages::msg::BoundingBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_messages::msg::BoundingBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::BoundingBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::BoundingBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_messages__msg__BoundingBox
    std::shared_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_messages__msg__BoundingBox
    std::shared_ptr<custom_messages::msg::BoundingBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox_ & other) const
  {
    if (this->low_left != other.low_left) {
      return false;
    }
    if (this->top_right != other.top_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox_

// alias to use template instance with default allocator
using BoundingBox =
  custom_messages::msg::BoundingBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
