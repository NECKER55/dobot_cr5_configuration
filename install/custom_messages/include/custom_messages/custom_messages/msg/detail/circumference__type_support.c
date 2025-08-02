// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_messages:msg/Circumference.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_messages/msg/detail/circumference__rosidl_typesupport_introspection_c.h"
#include "custom_messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_messages/msg/detail/circumference__functions.h"
#include "custom_messages/msg/detail/circumference__struct.h"


// Include directives for member types
// Member `circumference`
#include "custom_messages/msg/optimal_point.h"
// Member `circumference`
#include "custom_messages/msg/detail/optimal_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_messages__msg__Circumference__init(message_memory);
}

void custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_fini_function(void * message_memory)
{
  custom_messages__msg__Circumference__fini(message_memory);
}

size_t custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__size_function__Circumference__circumference(
  const void * untyped_member)
{
  const custom_messages__msg__OptimalPoint__Sequence * member =
    (const custom_messages__msg__OptimalPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__get_const_function__Circumference__circumference(
  const void * untyped_member, size_t index)
{
  const custom_messages__msg__OptimalPoint__Sequence * member =
    (const custom_messages__msg__OptimalPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__get_function__Circumference__circumference(
  void * untyped_member, size_t index)
{
  custom_messages__msg__OptimalPoint__Sequence * member =
    (custom_messages__msg__OptimalPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__fetch_function__Circumference__circumference(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const custom_messages__msg__OptimalPoint * item =
    ((const custom_messages__msg__OptimalPoint *)
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__get_const_function__Circumference__circumference(untyped_member, index));
  custom_messages__msg__OptimalPoint * value =
    (custom_messages__msg__OptimalPoint *)(untyped_value);
  *value = *item;
}

void custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__assign_function__Circumference__circumference(
  void * untyped_member, size_t index, const void * untyped_value)
{
  custom_messages__msg__OptimalPoint * item =
    ((custom_messages__msg__OptimalPoint *)
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__get_function__Circumference__circumference(untyped_member, index));
  const custom_messages__msg__OptimalPoint * value =
    (const custom_messages__msg__OptimalPoint *)(untyped_value);
  *item = *value;
}

bool custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__resize_function__Circumference__circumference(
  void * untyped_member, size_t size)
{
  custom_messages__msg__OptimalPoint__Sequence * member =
    (custom_messages__msg__OptimalPoint__Sequence *)(untyped_member);
  custom_messages__msg__OptimalPoint__Sequence__fini(member);
  return custom_messages__msg__OptimalPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_member_array[1] = {
  {
    "circumference",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_messages__msg__Circumference, circumference),  // bytes offset in struct
    NULL,  // default value
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__size_function__Circumference__circumference,  // size() function pointer
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__get_const_function__Circumference__circumference,  // get_const(index) function pointer
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__get_function__Circumference__circumference,  // get(index) function pointer
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__fetch_function__Circumference__circumference,  // fetch(index, &value) function pointer
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__assign_function__Circumference__circumference,  // assign(index, value) function pointer
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__resize_function__Circumference__circumference  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_members = {
  "custom_messages__msg",  // message namespace
  "Circumference",  // message name
  1,  // number of fields
  sizeof(custom_messages__msg__Circumference),
  custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_member_array,  // message members
  custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_type_support_handle = {
  0,
  &custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_messages, msg, Circumference)() {
  custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_messages, msg, OptimalPoint)();
  if (!custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_type_support_handle.typesupport_identifier) {
    custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_messages__msg__Circumference__rosidl_typesupport_introspection_c__Circumference_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
