// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_messages:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__STRUCT_H_
#define CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'shape'
#include "custom_messages/msg/detail/bounding_box__struct.h"
// Member 'possible_trajectories'
#include "custom_messages/msg/detail/circumference__struct.h"

/// Struct defined in msg/Object in the package custom_messages.
typedef struct custom_messages__msg__Object
{
  bool target;
  custom_messages__msg__BoundingBox shape;
  custom_messages__msg__Circumference__Sequence possible_trajectories;
} custom_messages__msg__Object;

// Struct for a sequence of custom_messages__msg__Object.
typedef struct custom_messages__msg__Object__Sequence
{
  custom_messages__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_messages__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__OBJECT__STRUCT_H_
