// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ugv_sensor_sync:msg/UwbRange.idl
// generated code does not contain a copyright notice
#include "ugv_sensor_sync/msg/detail/uwb_range__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ugv_sensor_sync__msg__UwbRange__init(ugv_sensor_sync__msg__UwbRange * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ugv_sensor_sync__msg__UwbRange__fini(msg);
    return false;
  }
  // esp32_time_us
  // range_m
  // range_stddev_m
  // anchor_id
  return true;
}

void
ugv_sensor_sync__msg__UwbRange__fini(ugv_sensor_sync__msg__UwbRange * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // esp32_time_us
  // range_m
  // range_stddev_m
  // anchor_id
}

bool
ugv_sensor_sync__msg__UwbRange__are_equal(const ugv_sensor_sync__msg__UwbRange * lhs, const ugv_sensor_sync__msg__UwbRange * rhs)
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
  // esp32_time_us
  if (lhs->esp32_time_us != rhs->esp32_time_us) {
    return false;
  }
  // range_m
  if (lhs->range_m != rhs->range_m) {
    return false;
  }
  // range_stddev_m
  if (lhs->range_stddev_m != rhs->range_stddev_m) {
    return false;
  }
  // anchor_id
  if (lhs->anchor_id != rhs->anchor_id) {
    return false;
  }
  return true;
}

bool
ugv_sensor_sync__msg__UwbRange__copy(
  const ugv_sensor_sync__msg__UwbRange * input,
  ugv_sensor_sync__msg__UwbRange * output)
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
  // esp32_time_us
  output->esp32_time_us = input->esp32_time_us;
  // range_m
  output->range_m = input->range_m;
  // range_stddev_m
  output->range_stddev_m = input->range_stddev_m;
  // anchor_id
  output->anchor_id = input->anchor_id;
  return true;
}

ugv_sensor_sync__msg__UwbRange *
ugv_sensor_sync__msg__UwbRange__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_sensor_sync__msg__UwbRange * msg = (ugv_sensor_sync__msg__UwbRange *)allocator.allocate(sizeof(ugv_sensor_sync__msg__UwbRange), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ugv_sensor_sync__msg__UwbRange));
  bool success = ugv_sensor_sync__msg__UwbRange__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ugv_sensor_sync__msg__UwbRange__destroy(ugv_sensor_sync__msg__UwbRange * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ugv_sensor_sync__msg__UwbRange__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ugv_sensor_sync__msg__UwbRange__Sequence__init(ugv_sensor_sync__msg__UwbRange__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_sensor_sync__msg__UwbRange * data = NULL;

  if (size) {
    data = (ugv_sensor_sync__msg__UwbRange *)allocator.zero_allocate(size, sizeof(ugv_sensor_sync__msg__UwbRange), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ugv_sensor_sync__msg__UwbRange__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ugv_sensor_sync__msg__UwbRange__fini(&data[i - 1]);
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
ugv_sensor_sync__msg__UwbRange__Sequence__fini(ugv_sensor_sync__msg__UwbRange__Sequence * array)
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
      ugv_sensor_sync__msg__UwbRange__fini(&array->data[i]);
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

ugv_sensor_sync__msg__UwbRange__Sequence *
ugv_sensor_sync__msg__UwbRange__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_sensor_sync__msg__UwbRange__Sequence * array = (ugv_sensor_sync__msg__UwbRange__Sequence *)allocator.allocate(sizeof(ugv_sensor_sync__msg__UwbRange__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ugv_sensor_sync__msg__UwbRange__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ugv_sensor_sync__msg__UwbRange__Sequence__destroy(ugv_sensor_sync__msg__UwbRange__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ugv_sensor_sync__msg__UwbRange__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ugv_sensor_sync__msg__UwbRange__Sequence__are_equal(const ugv_sensor_sync__msg__UwbRange__Sequence * lhs, const ugv_sensor_sync__msg__UwbRange__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ugv_sensor_sync__msg__UwbRange__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ugv_sensor_sync__msg__UwbRange__Sequence__copy(
  const ugv_sensor_sync__msg__UwbRange__Sequence * input,
  ugv_sensor_sync__msg__UwbRange__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ugv_sensor_sync__msg__UwbRange);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ugv_sensor_sync__msg__UwbRange * data =
      (ugv_sensor_sync__msg__UwbRange *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ugv_sensor_sync__msg__UwbRange__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ugv_sensor_sync__msg__UwbRange__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ugv_sensor_sync__msg__UwbRange__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
