// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/goal.h"


#ifndef CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'x_coordinates'
// Member 'y_coordinates'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Goal in the package custom_msgs.
typedef struct custom_msgs__msg__Goal
{
  rosidl_runtime_c__double__Sequence x_coordinates;
  rosidl_runtime_c__double__Sequence y_coordinates;
} custom_msgs__msg__Goal;

// Struct for a sequence of custom_msgs__msg__Goal.
typedef struct custom_msgs__msg__Goal__Sequence
{
  custom_msgs__msg__Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Goal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_H_
