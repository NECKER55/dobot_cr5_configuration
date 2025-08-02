// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_messages:msg/Map.idl
// generated code does not contain a copyright notice
#include "custom_messages/msg/detail/map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `work_space`
#include "custom_messages/msg/detail/bounding_box__functions.h"
// Member `objects`
#include "custom_messages/msg/detail/object__functions.h"

bool
custom_messages__msg__Map__init(custom_messages__msg__Map * msg)
{
  if (!msg) {
    return false;
  }
  // work_space
  if (!custom_messages__msg__BoundingBox__init(&msg->work_space)) {
    custom_messages__msg__Map__fini(msg);
    return false;
  }
  // objects
  if (!custom_messages__msg__Object__Sequence__init(&msg->objects, 0)) {
    custom_messages__msg__Map__fini(msg);
    return false;
  }
  return true;
}

void
custom_messages__msg__Map__fini(custom_messages__msg__Map * msg)
{
  if (!msg) {
    return;
  }
  // work_space
  custom_messages__msg__BoundingBox__fini(&msg->work_space);
  // objects
  custom_messages__msg__Object__Sequence__fini(&msg->objects);
}

bool
custom_messages__msg__Map__are_equal(const custom_messages__msg__Map * lhs, const custom_messages__msg__Map * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // work_space
  if (!custom_messages__msg__BoundingBox__are_equal(
      &(lhs->work_space), &(rhs->work_space)))
  {
    return false;
  }
  // objects
  if (!custom_messages__msg__Object__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
custom_messages__msg__Map__copy(
  const custom_messages__msg__Map * input,
  custom_messages__msg__Map * output)
{
  if (!input || !output) {
    return false;
  }
  // work_space
  if (!custom_messages__msg__BoundingBox__copy(
      &(input->work_space), &(output->work_space)))
  {
    return false;
  }
  // objects
  if (!custom_messages__msg__Object__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

custom_messages__msg__Map *
custom_messages__msg__Map__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__Map * msg = (custom_messages__msg__Map *)allocator.allocate(sizeof(custom_messages__msg__Map), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_messages__msg__Map));
  bool success = custom_messages__msg__Map__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_messages__msg__Map__destroy(custom_messages__msg__Map * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_messages__msg__Map__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_messages__msg__Map__Sequence__init(custom_messages__msg__Map__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__Map * data = NULL;

  if (size) {
    data = (custom_messages__msg__Map *)allocator.zero_allocate(size, sizeof(custom_messages__msg__Map), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_messages__msg__Map__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_messages__msg__Map__fini(&data[i - 1]);
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
custom_messages__msg__Map__Sequence__fini(custom_messages__msg__Map__Sequence * array)
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
      custom_messages__msg__Map__fini(&array->data[i]);
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

custom_messages__msg__Map__Sequence *
custom_messages__msg__Map__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__Map__Sequence * array = (custom_messages__msg__Map__Sequence *)allocator.allocate(sizeof(custom_messages__msg__Map__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_messages__msg__Map__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_messages__msg__Map__Sequence__destroy(custom_messages__msg__Map__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_messages__msg__Map__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_messages__msg__Map__Sequence__are_equal(const custom_messages__msg__Map__Sequence * lhs, const custom_messages__msg__Map__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_messages__msg__Map__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_messages__msg__Map__Sequence__copy(
  const custom_messages__msg__Map__Sequence * input,
  custom_messages__msg__Map__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_messages__msg__Map);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_messages__msg__Map * data =
      (custom_messages__msg__Map *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_messages__msg__Map__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_messages__msg__Map__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_messages__msg__Map__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
