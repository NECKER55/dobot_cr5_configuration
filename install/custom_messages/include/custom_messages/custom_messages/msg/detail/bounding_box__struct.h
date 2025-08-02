// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_messages:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'low_left'
// Member 'top_right'
#include "custom_messages/msg/detail/point__struct.h"

/// Struct defined in msg/BoundingBox in the package custom_messages.
typedef struct custom_messages__msg__BoundingBox
{
  custom_messages__msg__Point low_left;
  custom_messages__msg__Point top_right;
} custom_messages__msg__BoundingBox;

// Struct for a sequence of custom_messages__msg__BoundingBox.
typedef struct custom_messages__msg__BoundingBox__Sequence
{
  custom_messages__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_messages__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
