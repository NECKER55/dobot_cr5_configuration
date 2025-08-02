// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_messages:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__MAP__STRUCT_HPP_
#define CUSTOM_MESSAGES__MSG__DETAIL__MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'work_space'
#include "custom_messages/msg/detail/bounding_box__struct.hpp"
// Member 'objects'
#include "custom_messages/msg/detail/object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_messages__msg__Map __attribute__((deprecated))
#else
# define DEPRECATED__custom_messages__msg__Map __declspec(deprecated)
#endif

namespace custom_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Map_
{
  using Type = Map_<ContainerAllocator>;

  explicit Map_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : work_space(_init)
  {
    (void)_init;
  }

  explicit Map_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : work_space(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _work_space_type =
    custom_messages::msg::BoundingBox_<ContainerAllocator>;
  _work_space_type work_space;
  using _objects_type =
    std::vector<custom_messages::msg::Object_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_messages::msg::Object_<ContainerAllocator>>>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__work_space(
    const custom_messages::msg::BoundingBox_<ContainerAllocator> & _arg)
  {
    this->work_space = _arg;
    return *this;
  }
  Type & set__objects(
    const std::vector<custom_messages::msg::Object_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<custom_messages::msg::Object_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_messages::msg::Map_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_messages::msg::Map_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_messages::msg::Map_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_messages::msg::Map_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Map_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Map_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_messages::msg::Map_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_messages::msg::Map_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_messages::msg::Map_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_messages::msg::Map_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_messages__msg__Map
    std::shared_ptr<custom_messages::msg::Map_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_messages__msg__Map
    std::shared_ptr<custom_messages::msg::Map_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Map_ & other) const
  {
    if (this->work_space != other.work_space) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const Map_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Map_

// alias to use template instance with default allocator
using Map =
  custom_messages::msg::Map_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_messages

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__MAP__STRUCT_HPP_
