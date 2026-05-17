// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from object_detection_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "object_detection_msgs/msg/detection.h"


#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Detection in the package object_detection_msgs.
/**
  * Normalized bounding box coordinates (0.0 to 1.0)
 */
typedef struct object_detection_msgs__msg__Detection
{
  double x1;
  double y1;
  double x2;
  double y2;
  /// Confidence level (0.0 to 1.0)
  double conf;
  /// Distance in meters
  double distance;
} object_detection_msgs__msg__Detection;

// Struct for a sequence of object_detection_msgs__msg__Detection.
typedef struct object_detection_msgs__msg__Detection__Sequence
{
  object_detection_msgs__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} object_detection_msgs__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_H_
