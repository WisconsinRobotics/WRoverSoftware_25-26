// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from object_detection_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "object_detection_msgs/msg/detection.hpp"


#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "object_detection_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace object_detection_msgs
{

namespace msg
{

namespace builder
{

class Init_Detection_distance
{
public:
  explicit Init_Detection_distance(::object_detection_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  ::object_detection_msgs::msg::Detection distance(::object_detection_msgs::msg::Detection::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::object_detection_msgs::msg::Detection msg_;
};

class Init_Detection_conf
{
public:
  explicit Init_Detection_conf(::object_detection_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_distance conf(::object_detection_msgs::msg::Detection::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return Init_Detection_distance(msg_);
  }

private:
  ::object_detection_msgs::msg::Detection msg_;
};

class Init_Detection_y2
{
public:
  explicit Init_Detection_y2(::object_detection_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_conf y2(::object_detection_msgs::msg::Detection::_y2_type arg)
  {
    msg_.y2 = std::move(arg);
    return Init_Detection_conf(msg_);
  }

private:
  ::object_detection_msgs::msg::Detection msg_;
};

class Init_Detection_x2
{
public:
  explicit Init_Detection_x2(::object_detection_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_y2 x2(::object_detection_msgs::msg::Detection::_x2_type arg)
  {
    msg_.x2 = std::move(arg);
    return Init_Detection_y2(msg_);
  }

private:
  ::object_detection_msgs::msg::Detection msg_;
};

class Init_Detection_y1
{
public:
  explicit Init_Detection_y1(::object_detection_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_x2 y1(::object_detection_msgs::msg::Detection::_y1_type arg)
  {
    msg_.y1 = std::move(arg);
    return Init_Detection_x2(msg_);
  }

private:
  ::object_detection_msgs::msg::Detection msg_;
};

class Init_Detection_x1
{
public:
  Init_Detection_x1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_y1 x1(::object_detection_msgs::msg::Detection::_x1_type arg)
  {
    msg_.x1 = std::move(arg);
    return Init_Detection_y1(msg_);
  }

private:
  ::object_detection_msgs::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::object_detection_msgs::msg::Detection>()
{
  return object_detection_msgs::msg::builder::Init_Detection_x1();
}

}  // namespace object_detection_msgs

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
