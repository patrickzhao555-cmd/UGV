// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ugv_sensor_sync:msg/UwbRange.idl
// generated code does not contain a copyright notice

#ifndef UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__STRUCT_HPP_
#define UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ugv_sensor_sync__msg__UwbRange __attribute__((deprecated))
#else
# define DEPRECATED__ugv_sensor_sync__msg__UwbRange __declspec(deprecated)
#endif

namespace ugv_sensor_sync
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UwbRange_
{
  using Type = UwbRange_<ContainerAllocator>;

  explicit UwbRange_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->esp32_time_us = 0ull;
      this->range_m = 0.0;
      this->range_stddev_m = 0.0;
      this->anchor_id = 0ul;
    }
  }

  explicit UwbRange_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->esp32_time_us = 0ull;
      this->range_m = 0.0;
      this->range_stddev_m = 0.0;
      this->anchor_id = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _esp32_time_us_type =
    uint64_t;
  _esp32_time_us_type esp32_time_us;
  using _range_m_type =
    double;
  _range_m_type range_m;
  using _range_stddev_m_type =
    double;
  _range_stddev_m_type range_stddev_m;
  using _anchor_id_type =
    uint32_t;
  _anchor_id_type anchor_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__esp32_time_us(
    const uint64_t & _arg)
  {
    this->esp32_time_us = _arg;
    return *this;
  }
  Type & set__range_m(
    const double & _arg)
  {
    this->range_m = _arg;
    return *this;
  }
  Type & set__range_stddev_m(
    const double & _arg)
  {
    this->range_stddev_m = _arg;
    return *this;
  }
  Type & set__anchor_id(
    const uint32_t & _arg)
  {
    this->anchor_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ugv_sensor_sync::msg::UwbRange_<ContainerAllocator> *;
  using ConstRawPtr =
    const ugv_sensor_sync::msg::UwbRange_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ugv_sensor_sync::msg::UwbRange_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ugv_sensor_sync::msg::UwbRange_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ugv_sensor_sync__msg__UwbRange
    std::shared_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ugv_sensor_sync__msg__UwbRange
    std::shared_ptr<ugv_sensor_sync::msg::UwbRange_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UwbRange_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->esp32_time_us != other.esp32_time_us) {
      return false;
    }
    if (this->range_m != other.range_m) {
      return false;
    }
    if (this->range_stddev_m != other.range_stddev_m) {
      return false;
    }
    if (this->anchor_id != other.anchor_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const UwbRange_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UwbRange_

// alias to use template instance with default allocator
using UwbRange =
  ugv_sensor_sync::msg::UwbRange_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ugv_sensor_sync

#endif  // UGV_SENSOR_SYNC__MSG__DETAIL__UWB_RANGE__STRUCT_HPP_
