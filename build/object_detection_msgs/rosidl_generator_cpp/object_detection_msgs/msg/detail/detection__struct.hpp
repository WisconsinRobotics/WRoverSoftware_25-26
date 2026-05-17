// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from object_detection_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "object_detection_msgs/msg/detection.hpp"


#ifndef OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_
#define OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__object_detection_msgs__msg__Detection __attribute__((deprecated))
#else
# define DEPRECATED__object_detection_msgs__msg__Detection __declspec(deprecated)
#endif

namespace object_detection_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection_
{
  using Type = Detection_<ContainerAllocator>;

  explicit Detection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x1 = 0.0;
      this->y1 = 0.0;
      this->x2 = 0.0;
      this->y2 = 0.0;
      this->conf = 0.0;
      this->distance = 0.0;
    }
  }

  explicit Detection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x1 = 0.0;
      this->y1 = 0.0;
      this->x2 = 0.0;
      this->y2 = 0.0;
      this->conf = 0.0;
      this->distance = 0.0;
    }
  }

  // field types and members
  using _x1_type =
    double;
  _x1_type x1;
  using _y1_type =
    double;
  _y1_type y1;
  using _x2_type =
    double;
  _x2_type x2;
  using _y2_type =
    double;
  _y2_type y2;
  using _conf_type =
    double;
  _conf_type conf;
  using _distance_type =
    double;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__x1(
    const double & _arg)
  {
    this->x1 = _arg;
    return *this;
  }
  Type & set__y1(
    const double & _arg)
  {
    this->y1 = _arg;
    return *this;
  }
  Type & set__x2(
    const double & _arg)
  {
    this->x2 = _arg;
    return *this;
  }
  Type & set__y2(
    const double & _arg)
  {
    this->y2 = _arg;
    return *this;
  }
  Type & set__conf(
    const double & _arg)
  {
    this->conf = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    object_detection_msgs::msg::Detection_<ContainerAllocator> *;
  using ConstRawPtr =
    const object_detection_msgs::msg::Detection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      object_detection_msgs::msg::Detection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      object_detection_msgs::msg::Detection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__object_detection_msgs__msg__Detection
    std::shared_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__object_detection_msgs__msg__Detection
    std::shared_ptr<object_detection_msgs::msg::Detection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection_ & other) const
  {
    if (this->x1 != other.x1) {
      return false;
    }
    if (this->y1 != other.y1) {
      return false;
    }
    if (this->x2 != other.x2) {
      return false;
    }
    if (this->y2 != other.y2) {
      return false;
    }
    if (this->conf != other.conf) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection_

// alias to use template instance with default allocator
using Detection =
  object_detection_msgs::msg::Detection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace object_detection_msgs

#endif  // OBJECT_DETECTION_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_
