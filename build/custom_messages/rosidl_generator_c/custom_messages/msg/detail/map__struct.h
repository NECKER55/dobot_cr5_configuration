// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_messages:msg/Map.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__MAP__STRUCT_H_
#define CUSTOM_MESSAGES__MSG__DETAIL__MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'work_space'
#include "custom_messages/msg/detail/bounding_box__struct.h"
// Member 'objects'
#include "custom_messages/msg/detail/object__struct.h"

/// Struct defined in msg/Map in the package custom_messages.
typedef struct custom_messages__msg__Map
{
  custom_messages__msg__BoundingBox work_space;
  custom_messages__msg__Object__Sequence objects;
} custom_messages__msg__Map;

// Struct for a sequence of custom_messages__msg__Map.
typedef struct custom_messages__msg__Map__Sequence
{
  custom_messages__msg__Map * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_messages__msg__Map__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__MAP__STRUCT_H_
