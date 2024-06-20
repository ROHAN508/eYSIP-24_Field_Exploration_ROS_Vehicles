// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/goal.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__Goal __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__Goal __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Goal_
{
  using Type = Goal_<ContainerAllocator>;

  explicit Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _x_coordinates_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_coordinates_type x_coordinates;
  using _y_coordinates_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_coordinates_type y_coordinates;

  // setters for named parameter idiom
  Type & set__x_coordinates(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x_coordinates = _arg;
    return *this;
  }
  Type & set__y_coordinates(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y_coordinates = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__Goal
    std::shared_ptr<custom_msgs::msg::Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__Goal
    std::shared_ptr<custom_msgs::msg::Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Goal_ & other) const
  {
    if (this->x_coordinates != other.x_coordinates) {
      return false;
    }
    if (this->y_coordinates != other.y_coordinates) {
      return false;
    }
    return true;
  }
  bool operator!=(const Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Goal_

// alias to use template instance with default allocator
using Goal =
  custom_msgs::msg::Goal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__GOAL__STRUCT_HPP_
