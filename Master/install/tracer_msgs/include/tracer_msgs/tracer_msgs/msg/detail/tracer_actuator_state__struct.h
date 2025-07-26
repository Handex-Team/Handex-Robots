// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tracer_msgs:msg/TracerActuatorState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__STRUCT_H_
#define TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TracerActuatorState in the package tracer_msgs.
/**
  * define DRIVER_STATE_INPUT_VOLTAGE_LOW_MASK ((uint8_t)0x01)
  * define DRIVER_STATE_MOTOR_OVERHEAT_MASK ((uint8_t)0x02)
  * define DRIVER_STATE_DRIVER_OVERLOAD_MASK ((uint8_t)0x04)
  * define DRIVER_STATE_DRIVER_OVERHEAT_MASK ((uint8_t)0x08)
  * define DRIVER_STATE_SENSOR_FAULT_MASK ((uint8_t)0x10)
  * define DRIVER_STATE_DRIVER_FAULT_MASK ((uint8_t)0x20)
  * define DRIVER_STATE_DRIVER_ENABLED_MASK ((uint8_t)0x40)
  * define DRIVER_STATE_DRIVER_RESET_MASK ((uint8_t)0x80)
 */
typedef struct tracer_msgs__msg__TracerActuatorState
{
  /// uint8 motor_id
  int16_t rpm;
  double current;
} tracer_msgs__msg__TracerActuatorState;

// Struct for a sequence of tracer_msgs__msg__TracerActuatorState.
typedef struct tracer_msgs__msg__TracerActuatorState__Sequence
{
  tracer_msgs__msg__TracerActuatorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tracer_msgs__msg__TracerActuatorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_ACTUATOR_STATE__STRUCT_H_
