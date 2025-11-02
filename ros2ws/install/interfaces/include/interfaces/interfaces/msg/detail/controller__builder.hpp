// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Controller.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__CONTROLLER__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__CONTROLLER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/controller__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Controller_yaw
{
public:
  explicit Init_Controller_yaw(::interfaces::msg::Controller & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Controller yaw(::interfaces::msg::Controller::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Controller msg_;
};

class Init_Controller_depth
{
public:
  explicit Init_Controller_depth(::interfaces::msg::Controller & msg)
  : msg_(msg)
  {}
  Init_Controller_yaw depth(::interfaces::msg::Controller::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return Init_Controller_yaw(msg_);
  }

private:
  ::interfaces::msg::Controller msg_;
};

class Init_Controller_y
{
public:
  explicit Init_Controller_y(::interfaces::msg::Controller & msg)
  : msg_(msg)
  {}
  Init_Controller_depth y(::interfaces::msg::Controller::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Controller_depth(msg_);
  }

private:
  ::interfaces::msg::Controller msg_;
};

class Init_Controller_x
{
public:
  Init_Controller_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Controller_y x(::interfaces::msg::Controller::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Controller_y(msg_);
  }

private:
  ::interfaces::msg::Controller msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Controller>()
{
  return interfaces::msg::builder::Init_Controller_x();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CONTROLLER__BUILDER_HPP_
