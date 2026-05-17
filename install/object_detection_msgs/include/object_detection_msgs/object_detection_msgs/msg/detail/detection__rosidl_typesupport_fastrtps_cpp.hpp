// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from object_detection_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "object_detection_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "object_detection_msgs/msg/detail/detection__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace object_detection_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
cdr_serialize(
  const object_detection_msgs::msg::Detection & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  object_detection_msgs::msg::Detection & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
get_serialized_size(
  const object_detection_msgs::msg::Detection & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
max_serialized_size_Detection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
cdr_serialize_key(
  const object_detection_msgs::msg::Detection & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
get_serialized_size_key(
  const object_detection_msgs::msg::Detection & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
max_serialized_size_key_Detection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace object_detection_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_object_detection_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, object_detection_msgs, msg, Detection)();

#ifdef __cplusplus
}
#endif

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
