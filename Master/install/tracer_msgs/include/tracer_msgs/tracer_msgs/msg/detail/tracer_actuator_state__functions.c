// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tracer_msgs:msg/TracerActuatorState.idl
// generated code does not contain a copyright notice
#include "tracer_msgs/msg/detail/tracer_actuator_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
tracer_msgs__msg__TracerActuatorState__init(tracer_msgs__msg__TracerActuatorState * msg)
{
  if (!msg) {
    return false;
  }
  // rpm
  // current
  return true;
}

void
tracer_msgs__msg__TracerActuatorState__fini(tracer_msgs__msg__TracerActuatorState * msg)
{
  if (!msg) {
    return;
  }
  // rpm
  // current
}

bool
tracer_msgs__msg__TracerActuatorState__are_equal(const tracer_msgs__msg__TracerActuatorState * lhs, const tracer_msgs__msg__TracerActuatorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rpm
  if (lhs->rpm != rhs->rpm) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  return true;
}

bool
tracer_msgs__msg__TracerActuatorState__copy(
  const tracer_msgs__msg__TracerActuatorState * input,
  tracer_msgs__msg__TracerActuatorState * output)
{
  if (!input || !output) {
    return false;
  }
  // rpm
  output->rpm = input->rpm;
  // current
  output->current = input->current;
  return true;
}

tracer_msgs__msg__TracerActuatorState *
tracer_msgs__msg__TracerActuatorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tracer_msgs__msg__TracerActuatorState * msg = (tracer_msgs__msg__TracerActuatorState *)allocator.allocate(sizeof(tracer_msgs__msg__TracerActuatorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tracer_msgs__msg__TracerActuatorState));
  bool success = tracer_msgs__msg__TracerActuatorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tracer_msgs__msg__TracerActuatorState__destroy(tracer_msgs__msg__TracerActuatorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tracer_msgs__msg__TracerActuatorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tracer_msgs__msg__TracerActuatorState__Sequence__init(tracer_msgs__msg__TracerActuatorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tracer_msgs__msg__TracerActuatorState * data = NULL;

  if (size) {
    data = (tracer_msgs__msg__TracerActuatorState *)allocator.zero_allocate(size, sizeof(tracer_msgs__msg__TracerActuatorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tracer_msgs__msg__TracerActuatorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tracer_msgs__msg__TracerActuatorState__fini(&data[i - 1]);
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
tracer_msgs__msg__TracerActuatorState__Sequence__fini(tracer_msgs__msg__TracerActuatorState__Sequence * array)
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
      tracer_msgs__msg__TracerActuatorState__fini(&array->data[i]);
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

tracer_msgs__msg__TracerActuatorState__Sequence *
tracer_msgs__msg__TracerActuatorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tracer_msgs__msg__TracerActuatorState__Sequence * array = (tracer_msgs__msg__TracerActuatorState__Sequence *)allocator.allocate(sizeof(tracer_msgs__msg__TracerActuatorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tracer_msgs__msg__TracerActuatorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tracer_msgs__msg__TracerActuatorState__Sequence__destroy(tracer_msgs__msg__TracerActuatorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tracer_msgs__msg__TracerActuatorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tracer_msgs__msg__TracerActuatorState__Sequence__are_equal(const tracer_msgs__msg__TracerActuatorState__Sequence * lhs, const tracer_msgs__msg__TracerActuatorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tracer_msgs__msg__TracerActuatorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tracer_msgs__msg__TracerActuatorState__Sequence__copy(
  const tracer_msgs__msg__TracerActuatorState__Sequence * input,
  tracer_msgs__msg__TracerActuatorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tracer_msgs__msg__TracerActuatorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tracer_msgs__msg__TracerActuatorState * data =
      (tracer_msgs__msg__TracerActuatorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tracer_msgs__msg__TracerActuatorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tracer_msgs__msg__TracerActuatorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tracer_msgs__msg__TracerActuatorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
