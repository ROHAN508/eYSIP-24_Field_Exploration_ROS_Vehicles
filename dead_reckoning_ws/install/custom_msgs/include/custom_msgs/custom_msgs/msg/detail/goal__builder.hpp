// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/goal.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Goal_y_coordinates
{
public:
  explicit Init_Goal_y_coordinates(::custom_msgs::msg::Goal & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Goal y_coordinates(::custom_msgs::msg::Goal::_y_coordinates_type arg)
  {
    msg_.y_coordinates = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Goal msg_;
};

class Init_Goal_x_coordinates
{
public:
  Init_Goal_x_coordinates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Goal_y_coordinates x_coordinates(::custom_msgs::msg::Goal::_x_coordinates_type arg)
  {
    msg_.x_coordinates = std::move(arg);
    return Init_Goal_y_coordinates(msg_);
  }

private:
  ::custom_msgs::msg::Goal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Goal>()
{
  return custom_msgs::msg::builder::Init_Goal_x_coordinates();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__GOAL__BUILDER_HPP_
