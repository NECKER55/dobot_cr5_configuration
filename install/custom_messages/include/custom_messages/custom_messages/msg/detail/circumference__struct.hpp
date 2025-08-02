// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_messages:msg/Circumference.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__STRUCT_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'circumference'
#include "custom_messages/msg/detail/optimal_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_messages__msg__Circumference __attribute__((deprecated))
#else
# define DEPRECATED__custom_messages__msg__Circumference __declspec(deprecated)
#endif

namespace custom_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Circumference_
{
  using Type = Circumference_<ContainerAllocator>;

  explicit Circumference_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Circumference_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _circumference_type =
    std::vector<custom_messages::msg::OptimalPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_messages::msg::OptimalPoint_<ContainerAllocator>>>;
  _circumference_type circumference;

  // setters for named parameter idiom
  Type & set__circumference(
    const std::vector<custom_messages::msg::OptimalPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_messages::msg::OptimalPoint_<ContainerAllocator>>> & _arg)
  {
    this->circumference = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_messages::msg::Circumference_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_messages::msg::Circumference_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_messages::msg::Circumference_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_messages::msg::Circumference_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Circumference_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Circumference_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Circumference_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Circumference_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_messages::msg::Circumference_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_messages::msg::Circumference_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_messages__msg__Circumference
    std::shared_ptr<custom_messages::msg::Circumference_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_messages__msg__Circumference
    std::shared_ptr<custom_messages::msg::Circumference_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Circumference_ & other) const
  {
    if (this->circumference != other.circumference) {
      return false;
    }
    return true;
  }
  bool operator!=(const Circumference_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Circumference_

// alias to use template instance with default allocator
using Circumference =
  custom_messages::msg::Circumference_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__STRUCT_HPP_
