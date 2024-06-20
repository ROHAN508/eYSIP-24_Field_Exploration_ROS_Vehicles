// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/Goal.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "custom_msgs/msg/goal.hpp"


#ifndef CUSTOM_MSGS__MSG__DETAIL__GOAL__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_coordinates
  {
    if (msg.x_coordinates.size() == 0) {
      out << "x_coordinates: []";
    } else {
      out << "x_coordinates: [";
      size_t pending_items = msg.x_coordinates.size();
      for (auto item : msg.x_coordinates) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y_coordinates
  {
    if (msg.y_coordinates.size() == 0) {
      out << "y_coordinates: []";
    } else {
      out << "y_coordinates: [";
      size_t pending_items = msg.y_coordinates.size();
      for (auto item : msg.y_coordinates) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_coordinates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x_coordinates.size() == 0) {
      out << "x_coordinates: []\n";
    } else {
      out << "x_coordinates:\n";
      for (auto item : msg.x_coordinates) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y_coordinates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y_coordinates.size() == 0) {
      out << "y_coordinates: []\n";
    } else {
      out << "y_coordinates:\n";
      for (auto item : msg.y_coordinates) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::Goal & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::Goal>()
{
  return "custom_msgs::msg::Goal";
}

template<>
inline const char * name<custom_msgs::msg::Goal>()
{
  return "custom_msgs/msg/Goal";
}

template<>
struct has_fixed_size<custom_msgs::msg::Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::msg::Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::msg::Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__GOAL__TRAITS_HPP_
