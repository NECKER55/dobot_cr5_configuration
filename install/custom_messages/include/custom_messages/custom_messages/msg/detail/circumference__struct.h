// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_messages:msg/Circumference.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__STRUCT_H_
#define CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'circumference'
#include "custom_messages/msg/detail/optimal_point__struct.h"

/// Struct defined in msg/Circumference in the package custom_messages.
typedef struct custom_messages__msg__Circumference
{
  custom_messages__msg__OptimalPoint__Sequence circumference;
} custom_messages__msg__Circumference;

// Struct for a sequence of custom_messages__msg__Circumference.
typedef struct custom_messages__msg__Circumference__Sequence
{
  custom_messages__msg__Circumference * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_messages__msg__Circumference__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGES__MSG__DETAIL__CIRCUMFERENCE__STRUCT_H_
