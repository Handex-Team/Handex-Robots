// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tracer_msgs:msg/TracerLightState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__STRUCT_HPP_
#define TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tracer_msgs__msg__TracerLightState __attribute__((deprecated))
#else
# define DEPRECATED__tracer_msgs__msg__TracerLightState __declspec(deprecated)
#endif

namespace tracer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TracerLightState_
{
  using Type = TracerLightState_<ContainerAllocator>;

  explicit TracerLightState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->custom_value = 0;
    }
  }

  explicit TracerLightState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->custom_value = 0;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _custom_value_type =
    uint8_t;
  _custom_value_type custom_value;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__custom_value(
    const uint8_t & _arg)
  {
    this->custom_value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tracer_msgs::msg::TracerLightState_<ContainerAllocator> *;
  using ConstRawPtr =
    const tracer_msgs::msg::TracerLightState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tracer_msgs::msg::TracerLightState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tracer_msgs::msg::TracerLightState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tracer_msgs__msg__TracerLightState
    std::shared_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tracer_msgs__msg__TracerLightState
    std::shared_ptr<tracer_msgs::msg::TracerLightState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TracerLightState_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->custom_value != other.custom_value) {
      return false;
    }
    return true;
  }
  bool operator!=(const TracerLightState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TracerLightState_

// alias to use template instance with default allocator
using TracerLightState =
  tracer_msgs::msg::TracerLightState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tracer_msgs

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_LIGHT_STATE__STRUCT_HPP_
