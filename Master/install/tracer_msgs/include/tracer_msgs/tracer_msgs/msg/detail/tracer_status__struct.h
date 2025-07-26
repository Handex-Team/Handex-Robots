// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tracer_msgs:msg/TracerStatus.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_STATUS__STRUCT_H_
#define TRACER_MSGS__MSG__DETAIL__TRACER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MOTOR_ID_FRONT_RIGHT'.
enum
{
  tracer_msgs__msg__TracerStatus__MOTOR_ID_FRONT_RIGHT = 0
};

/// Constant 'MOTOR_ID_FRONT_LEFT'.
enum
{
  tracer_msgs__msg__TracerStatus__MOTOR_ID_FRONT_LEFT = 1
};

/// Constant 'MOTOR_ID_REAR_RIGHT'.
enum
{
  tracer_msgs__msg__TracerStatus__MOTOR_ID_REAR_RIGHT = 2
};

/// Constant 'MOTOR_ID_REAR_LEFT'.
enum
{
  tracer_msgs__msg__TracerStatus__MOTOR_ID_REAR_LEFT = 3
};

/// Constant 'LIGHT_ID_FRONT'.
enum
{
  tracer_msgs__msg__TracerStatus__LIGHT_ID_FRONT = 0
};

/// Constant 'LIGHT_ID_REAR'.
enum
{
  tracer_msgs__msg__TracerStatus__LIGHT_ID_REAR = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'actuator_states'
#include "tracer_msgs/msg/detail/tracer_actuator_state__struct.h"

/// Struct defined in msg/TracerStatus in the package tracer_msgs.
typedef struct tracer_msgs__msg__TracerStatus
{
  std_msgs__msg__Header header;
  /// motion state
  double linear_velocity;
  double angular_velocity;
  ///  base state
  /// uint8 vehicle_state
  uint8_t control_mode;
  uint16_t error_code;
  double battery_voltage;
  /// motor state
  tracer_msgs__msg__TracerActuatorState actuator_states[2];
  bool light_control_enabled;
} tracer_msgs__msg__TracerStatus;

// Struct for a sequence of tracer_msgs__msg__TracerStatus.
typedef struct tracer_msgs__msg__TracerStatus__Sequence
{
  tracer_msgs__msg__TracerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tracer_msgs__msg__TracerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_STATUS__STRUCT_H_
