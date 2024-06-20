// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

#include "custom_msgs/msg/detail/goal__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_custom_msgs
const rosidl_type_hash_t *
custom_msgs__msg__Goal__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x36, 0x37, 0x69, 0x4d, 0x45, 0x3c, 0x37, 0x3d,
      0x73, 0x8f, 0x03, 0x74, 0x7e, 0xfb, 0x1d, 0x9e,
      0x71, 0x33, 0xcf, 0xb7, 0x95, 0xdc, 0xe0, 0xb5,
      0xe1, 0x4d, 0x5a, 0x3f, 0x96, 0x5a, 0x96, 0xb7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char custom_msgs__msg__Goal__TYPE_NAME[] = "custom_msgs/msg/Goal";

// Define type names, field names, and default values
static char custom_msgs__msg__Goal__FIELD_NAME__x_coordinates[] = "x_coordinates";
static char custom_msgs__msg__Goal__FIELD_NAME__y_coordinates[] = "y_coordinates";

static rosidl_runtime_c__type_description__Field custom_msgs__msg__Goal__FIELDS[] = {
  {
    {custom_msgs__msg__Goal__FIELD_NAME__x_coordinates, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {custom_msgs__msg__Goal__FIELD_NAME__y_coordinates, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
custom_msgs__msg__Goal__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {custom_msgs__msg__Goal__TYPE_NAME, 20, 20},
      {custom_msgs__msg__Goal__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64[] x_coordinates\n"
  "float64[] y_coordinates";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
custom_msgs__msg__Goal__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {custom_msgs__msg__Goal__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 47, 47},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
custom_msgs__msg__Goal__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *custom_msgs__msg__Goal__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
