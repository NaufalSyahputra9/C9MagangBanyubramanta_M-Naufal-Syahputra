// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/Add.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__ADD__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__ADD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/add__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_Add_Request_b
{
public:
  explicit Init_Add_Request_b(::interfaces::srv::Add_Request & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::Add_Request b(::interfaces::srv::Add_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::Add_Request msg_;
};

class Init_Add_Request_a
{
public:
  Init_Add_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Add_Request_b a(::interfaces::srv::Add_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Add_Request_b(msg_);
  }

private:
  ::interfaces::srv::Add_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::Add_Request>()
{
  return interfaces::srv::builder::Init_Add_Request_a();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_Add_Response_sum
{
public:
  Init_Add_Response_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::Add_Response sum(::interfaces::srv::Add_Response::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::Add_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::Add_Response>()
{
  return interfaces::srv::builder::Init_Add_Response_sum();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__ADD__BUILDER_HPP_
