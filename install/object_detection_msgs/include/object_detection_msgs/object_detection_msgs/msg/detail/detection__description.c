// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from object_detection_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#include "object_detection_msgs/msg/detail/detection__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_object_detection_msgs
const rosidl_type_hash_t *
object_detection_msgs__msg__Detection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x17, 0xb2, 0x0c, 0xf2, 0x6e, 0x57, 0xfb, 0x2e,
      0x8a, 0xc0, 0xe4, 0x5d, 0x28, 0x42, 0xf7, 0x18,
      0xc8, 0xd2, 0xf2, 0xe5, 0xc8, 0x4c, 0x5a, 0x6b,
      0xb5, 0xa4, 0x19, 0xf6, 0xac, 0x64, 0xe2, 0xc7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char object_detection_msgs__msg__Detection__TYPE_NAME[] = "object_detection_msgs/msg/Detection";

// Define type names, field names, and default values
static char object_detection_msgs__msg__Detection__FIELD_NAME__x1[] = "x1";
static char object_detection_msgs__msg__Detection__FIELD_NAME__y1[] = "y1";
static char object_detection_msgs__msg__Detection__FIELD_NAME__x2[] = "x2";
static char object_detection_msgs__msg__Detection__FIELD_NAME__y2[] = "y2";
static char object_detection_msgs__msg__Detection__FIELD_NAME__conf[] = "conf";
static char object_detection_msgs__msg__Detection__FIELD_NAME__distance[] = "distance";

static rosidl_runtime_c__type_description__Field object_detection_msgs__msg__Detection__FIELDS[] = {
  {
    {object_detection_msgs__msg__Detection__FIELD_NAME__x1, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__Detection__FIELD_NAME__y1, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__Detection__FIELD_NAME__x2, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__Detection__FIELD_NAME__y2, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__Detection__FIELD_NAME__conf, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {object_detection_msgs__msg__Detection__FIELD_NAME__distance, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
object_detection_msgs__msg__Detection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {object_detection_msgs__msg__Detection__TYPE_NAME, 35, 35},
      {object_detection_msgs__msg__Detection__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Normalized bounding box coordinates (0.0 to 1.0)\n"
  "float64 x1\n"
  "float64 y1\n"
  "float64 x2\n"
  "float64 y2\n"
  "\n"
  "# Confidence level (0.0 to 1.0)\n"
  "float64 conf\n"
  "\n"
  "# Distance in meters\n"
  "float64 distance";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
object_detection_msgs__msg__Detection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {object_detection_msgs__msg__Detection__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 180, 180},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
object_detection_msgs__msg__Detection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *object_detection_msgs__msg__Detection__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
