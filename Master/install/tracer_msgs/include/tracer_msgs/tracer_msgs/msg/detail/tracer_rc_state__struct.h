// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tracer_msgs:msg/TracerRCState.idl
// generated code does not contain a copyright notice

#ifndef TRACER_MSGS__MSG__DETAIL__TRACER_RC_STATE__STRUCT_H_
#define TRACER_MSGS__MSG__DETAIL__TRACER_RC_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TracerRCState in the package tracer_msgs.
typedef struct tracer_msgs__msg__TracerRCState
{
  uint8_t swa;
  uint8_t swb;
  uint8_t swc;
  uint8_t swd;
  int8_t stick_right_v;
  int8_t stick_right_h;
  int8_t stick_left_v;
  int8_t stick_left_h;
  int8_t var_a;
} tracer_msgs__msg__TracerRCState;

// Struct for a sequence of tracer_msgs__msg__TracerRCState.
typedef struct tracer_msgs__msg__TracerRCState__Sequence
{
  tracer_msgs__msg__TracerRCState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tracer_msgs__msg__TracerRCState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACER_MSGS__MSG__DETAIL__TRACER_RC_STATE__STRUCT_H_
