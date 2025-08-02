// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_messages:msg/Object.idl
// generated code does not contain a copyright notice
#include "custom_messages/msg/detail/object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `shape`
#include "custom_messages/msg/detail/bounding_box__functions.h"
// Member `possible_trajectories`
#include "custom_messages/msg/detail/circumference__functions.h"

bool
custom_messages__msg__Object__init(custom_messages__msg__Object * msg)
{
  if (!msg) {
    return false;
  }
  // target
  // shape
  if (!custom_messages__msg__BoundingBox__init(&msg->shape)) {
    custom_messages__msg__Object__fini(msg);
    return false;
  }
  // possible_trajectories
  if (!custom_messages__msg__Circumference__Sequence__init(&msg->possible_trajectories, 0)) {
    custom_messages__msg__Object__fini(msg);
    return false;
  }
  return true;
}

void
custom_messages__msg__Object__fini(custom_messages__msg__Object * msg)
{
  if (!msg) {
    return;
  }
  // target
  // shape
  custom_messages__msg__BoundingBox__fini(&msg->shape);
  // possible_trajectories
  custom_messages__msg__Circumference__Sequence__fini(&msg->possible_trajectories);
}

bool
custom_messages__msg__Object__are_equal(const custom_messages__msg__Object * lhs, const custom_messages__msg__Object * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target
  if (lhs->target != rhs->target) {
    return false;
  }
  // shape
  if (!custom_messages__msg__BoundingBox__are_equal(
      &(lhs->shape), &(rhs->shape)))
  {
    return false;
  }
  // possible_trajectories
  if (!custom_messages__msg__Circumference__Sequence__are_equal(
      &(lhs->possible_trajectories), &(rhs->possible_trajectories)))
  {
    return false;
  }
  return true;
}

bool
custom_messages__msg__Object__copy(
  const custom_messages__msg__Object * input,
  custom_messages__msg__Object * output)
{
  if (!input || !output) {
    return false;
  }
  // target
  output->target = input->target;
  // shape
  if (!custom_messages__msg__BoundingBox__copy(
      &(input->shape), &(output->shape)))
  {
    return false;
  }
  // possible_trajectories
  if (!custom_messages__msg__Circumference__Sequence__copy(
      &(input->possible_trajectories), &(output->possible_trajectories)))
  {
    return false;
  }
  return true;
}

custom_messages__msg__Object *
custom_messages__msg__Object__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__Object * msg = (custom_messages__msg__Object *)allocator.allocate(sizeof(custom_messages__msg__Object), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_messages__msg__Object));
  bool success = custom_messages__msg__Object__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_messages__msg__Object__destroy(custom_messages__msg__Object * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_messages__msg__Object__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_messages__msg__Object__Sequence__init(custom_messages__msg__Object__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__Object * data = NULL;

  if (size) {
    data = (custom_messages__msg__Object *)allocator.zero_allocate(size, sizeof(custom_messages__msg__Object), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_messages__msg__Object__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_messages__msg__Object__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_messages__msg__Object__Sequence__fini(custom_messages__msg__Object__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_messages__msg__Object__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_messages__msg__Object__Sequence *
custom_messages__msg__Object__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__Object__Sequence * array = (custom_messages__msg__Object__Sequence *)allocator.allocate(sizeof(custom_messages__msg__Object__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_messages__msg__Object__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_messages__msg__Object__Sequence__destroy(custom_messages__msg__Object__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_messages__msg__Object__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_messages__msg__Object__Sequence__are_equal(const custom_messages__msg__Object__Sequence * lhs, const custom_messages__msg__Object__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_messages__msg__Object__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_messages__msg__Object__Sequence__copy(
  const custom_messages__msg__Object__Sequence * input,
  custom_messages__msg__Object__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_messages__msg__Object);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_messages__msg__Object * data =
      (custom_messages__msg__Object *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_messages__msg__Object__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_messages__msg__Object__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_messages__msg__Object__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
