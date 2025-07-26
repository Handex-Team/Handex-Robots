// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tracer_msgs:msg/TracerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__STRUCT_HPP_
#define TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tracer_msgs__msg__TracerActuatorState __attribute__((deprecated))
#else
# define DEPRECATED__tracer_msgs__msg__TracerActuatorState __declspec(deprecated)
#endif

namespace tracer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TracerActuatorState_
{
  using Type = TracerActuatorState_<ContainerAllocator>;

  explicit TracerActuatorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm = 0;
      this->current = 0.0;
    }
  }

  explicit TracerActuatorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm = 0;
      this->current = 0.0;
    }
  }

  // field types and members
  using _rpm_type =
    int16_t;
  _rpm_type rpm;
  using _current_type =
    double;
  _current_type current;

  // setters for named parameter idiom
  Type & set__rpm(
    const int16_t & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__current(
    const double & _arg)
  {
    this->current = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tracer_msgs::msg::TracerActuatorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const tracer_msgs::msg::TracerActuatorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tracer_msgs::msg::TracerActuatorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tracer_msgs::msg::TracerActuatorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tracer_msgs__msg__TracerActuatorState
    std::shared_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tracer_msgs__msg__TracerActuatorState
    std::shared_ptr<tracer_msgs::msg::TracerActuatorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TracerActuatorState_ & other) const
  {
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    return true;
  }
  bool operator!=(const TracerActuatorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TracerActuatorState_

// alias to use template instance with default allocator
using TracerActuatorState =
  tracer_msgs::msg::TracerActuatorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tracer_msgs

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__STRUCT_HPP_
