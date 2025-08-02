// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_messages:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__STRUCT_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'shape'
#include "custom_messages/msg/detail/bounding_box__struct.hpp"
// Member 'possible_trajectories'
#include "custom_messages/msg/detail/circumference__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_messages__msg__Object __attribute__((deprecated))
#else
# define DEPRECATED__custom_messages__msg__Object __declspec(deprecated)
#endif

namespace custom_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Object_
{
  using Type = Object_<ContainerAllocator>;

  explicit Object_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : shape(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = false;
    }
  }

  explicit Object_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : shape(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target = false;
    }
  }

  // field types and members
  using _target_type =
    bool;
  _target_type target;
  using _shape_type =
    custom_messages::msg::BoundingBox_<ContainerAllocator>;
  _shape_type shape;
  using _possible_trajectories_type =
    std::vector<custom_messages::msg::Circumference_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_messages::msg::Circumference_<ContainerAllocator>>>;
  _possible_trajectories_type possible_trajectories;

  // setters for named parameter idiom
  Type & set__target(
    const bool & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__shape(
    const custom_messages::msg::BoundingBox_<ContainerAllocator> & _arg)
  {
    this->shape = _arg;
    return *this;
  }
  Type & set__possible_trajectories(
    const std::vector<custom_messages::msg::Circumference_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_messages::msg::Circumference_<ContainerAllocator>>> & _arg)
  {
    this->possible_trajectories = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_messages::msg::Object_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_messages::msg::Object_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_messages::msg::Object_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_messages::msg::Object_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Object_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Object_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Object_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Object_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_messages::msg::Object_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_messages::msg::Object_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_messages__msg__Object
    std::shared_ptr<custom_messages::msg::Object_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_messages__msg__Object
    std::shared_ptr<custom_messages::msg::Object_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Object_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    if (this->shape != other.shape) {
      return false;
    }
    if (this->possible_trajectories != other.possible_trajectories) {
      return false;
    }
    return true;
  }
  bool operator!=(const Object_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Object_

// alias to use template instance with default allocator
using Object =
  custom_messages::msg::Object_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__STRUCT_HPP_
