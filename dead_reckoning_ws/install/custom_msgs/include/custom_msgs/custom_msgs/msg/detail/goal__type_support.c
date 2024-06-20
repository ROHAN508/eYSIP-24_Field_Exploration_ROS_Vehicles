// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_msgs/msg/detail/goal__rosidl_typesupport_introspection_c.h"
#include "custom_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_msgs/msg/detail/goal__functions.h"
#include "custom_msgs/msg/detail/goal__struct.h"


// Include directives for member types
// Member `x_coordinates`
// Member `y_coordinates`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_msgs__msg__Goal__init(message_memory);
}

void custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_fini_function(void * message_memory)
{
  custom_msgs__msg__Goal__fini(message_memory);
}

size_t custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__size_function__Goal__x_coordinates(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_const_function__Goal__x_coordinates(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_function__Goal__x_coordinates(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__fetch_function__Goal__x_coordinates(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_const_function__Goal__x_coordinates(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__assign_function__Goal__x_coordinates(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_function__Goal__x_coordinates(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__resize_function__Goal__x_coordinates(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__size_function__Goal__y_coordinates(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_const_function__Goal__y_coordinates(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_function__Goal__y_coordinates(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__fetch_function__Goal__y_coordinates(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_const_function__Goal__y_coordinates(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__assign_function__Goal__y_coordinates(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_function__Goal__y_coordinates(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__resize_function__Goal__y_coordinates(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_member_array[2] = {
  {
    "x_coordinates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Goal, x_coordinates),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__size_function__Goal__x_coordinates,  // size() function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_const_function__Goal__x_coordinates,  // get_const(index) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_function__Goal__x_coordinates,  // get(index) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__fetch_function__Goal__x_coordinates,  // fetch(index, &value) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__assign_function__Goal__x_coordinates,  // assign(index, value) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__resize_function__Goal__x_coordinates  // resize(index) function pointer
  },
  {
    "y_coordinates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Goal, y_coordinates),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__size_function__Goal__y_coordinates,  // size() function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_const_function__Goal__y_coordinates,  // get_const(index) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__get_function__Goal__y_coordinates,  // get(index) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__fetch_function__Goal__y_coordinates,  // fetch(index, &value) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__assign_function__Goal__y_coordinates,  // assign(index, value) function pointer
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__resize_function__Goal__y_coordinates  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_members = {
  "custom_msgs__msg",  // message namespace
  "Goal",  // message name
  2,  // number of fields
  sizeof(custom_msgs__msg__Goal),
  false,  // has_any_key_member_
  custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_member_array,  // message members
  custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_type_support_handle = {
  0,
  &custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_members,
  get_message_typesupport_handle_function,
  &custom_msgs__msg__Goal__get_type_hash,
  &custom_msgs__msg__Goal__get_type_description,
  &custom_msgs__msg__Goal__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_msgs, msg, Goal)() {
  if (!custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_type_support_handle.typesupport_identifier) {
    custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_msgs__msg__Goal__rosidl_typesupport_introspection_c__Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
