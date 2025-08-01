// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tracer_msgs:msg/TracerStatus.idl
// generated code does not contain a copyright notice
#include "tracer_msgs/msg/detail/tracer_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `actuator_states`
#include "tracer_msgs/msg/detail/tracer_actuator_state__functions.h"

bool
tracer_msgs__msg__TracerStatus__init(tracer_msgs__msg__TracerStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tracer_msgs__msg__TracerStatus__fini(msg);
    return false;
  }
  // linear_velocity
  // angular_velocity
  // control_mode
  // error_code
  // battery_voltage
  // actuator_states
  for (size_t i = 0; i < 2; ++i) {
    if (!tracer_msgs__msg__TracerActuatorState__init(&msg->actuator_states[i])) {
      tracer_msgs__msg__TracerStatus__fini(msg);
      return false;
    }
  }
  // light_control_enabled
  return true;
}

void
tracer_msgs__msg__TracerStatus__fini(tracer_msgs__msg__TracerStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // linear_velocity
  // angular_velocity
  // control_mode
  // error_code
  // battery_voltage
  // actuator_states
  for (size_t i = 0; i < 2; ++i) {
    tracer_msgs__msg__TracerActuatorState__fini(&msg->actuator_states[i]);
  }
  // light_control_enabled
}

bool
tracer_msgs__msg__TracerStatus__are_equal(const tracer_msgs__msg__TracerStatus * lhs, const tracer_msgs__msg__TracerStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // control_mode
  if (lhs->control_mode != rhs->control_mode) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // actuator_states
  for (size_t i = 0; i < 2; ++i) {
    if (!tracer_msgs__msg__TracerActuatorState__are_equal(
        &(lhs->actuator_states[i]), &(rhs->actuator_states[i])))
    {
      return false;
    }
  }
  // light_control_enabled
  if (lhs->light_control_enabled != rhs->light_control_enabled) {
    return false;
  }
  return true;
}

bool
tracer_msgs__msg__TracerStatus__copy(
  const tracer_msgs__msg__TracerStatus * input,
  tracer_msgs__msg__TracerStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // control_mode
  output->control_mode = input->control_mode;
  // error_code
  output->error_code = input->error_code;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // actuator_states
  for (size_t i = 0; i < 2; ++i) {
    if (!tracer_msgs__msg__TracerActuatorState__copy(
        &(input->actuator_states[i]), &(output->actuator_states[i])))
    {
      return false;
    }
  }
  // light_control_enabled
  output->light_control_enabled = input->light_control_enabled;
  return true;
}

tracer_msgs__msg__TracerStatus *
tracer_msgs__msg__TracerStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tracer_msgs__msg__TracerStatus * msg = (tracer_msgs__msg__TracerStatus *)allocator.allocate(sizeof(tracer_msgs__msg__TracerStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tracer_msgs__msg__TracerStatus));
  bool success = tracer_msgs__msg__TracerStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tracer_msgs__msg__TracerStatus__destroy(tracer_msgs__msg__TracerStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tracer_msgs__msg__TracerStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tracer_msgs__msg__TracerStatus__Sequence__init(tracer_msgs__msg__TracerStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tracer_msgs__msg__TracerStatus * data = NULL;

  if (size) {
    data = (tracer_msgs__msg__TracerStatus *)allocator.zero_allocate(size, sizeof(tracer_msgs__msg__TracerStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tracer_msgs__msg__TracerStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tracer_msgs__msg__TracerStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tracer_msgs__msg__TracerStatus__Sequence__fini(tracer_msgs__msg__TracerStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tracer_msgs__msg__TracerStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tracer_msgs__msg__TracerStatus__Sequence *
tracer_msgs__msg__TracerStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tracer_msgs__msg__TracerStatus__Sequence * array = (tracer_msgs__msg__TracerStatus__Sequence *)allocator.allocate(sizeof(tracer_msgs__msg__TracerStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tracer_msgs__msg__TracerStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tracer_msgs__msg__TracerStatus__Sequence__destroy(tracer_msgs__msg__TracerStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tracer_msgs__msg__TracerStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tracer_msgs__msg__TracerStatus__Sequence__are_equal(const tracer_msgs__msg__TracerStatus__Sequence * lhs, const tracer_msgs__msg__TracerStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tracer_msgs__msg__TracerStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tracer_msgs__msg__TracerStatus__Sequence__copy(
  const tracer_msgs__msg__TracerStatus__Sequence * input,
  tracer_msgs__msg__TracerStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tracer_msgs__msg__TracerStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tracer_msgs__msg__TracerStatus * data =
      (tracer_msgs__msg__TracerStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tracer_msgs__msg__TracerStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tracer_msgs__msg__TracerStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tracer_msgs__msg__TracerStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
