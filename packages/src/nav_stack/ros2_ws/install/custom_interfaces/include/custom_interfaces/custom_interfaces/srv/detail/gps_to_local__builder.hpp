// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/GpsToLocal.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__GPS_TO_LOCAL__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__GPS_TO_LOCAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/gps_to_local__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GpsToLocal_Request_altitude
{
public:
  explicit Init_GpsToLocal_Request_altitude(::custom_interfaces::srv::GpsToLocal_Request & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::GpsToLocal_Request altitude(::custom_interfaces::srv::GpsToLocal_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::GpsToLocal_Request msg_;
};

class Init_GpsToLocal_Request_longitude
{
public:
  explicit Init_GpsToLocal_Request_longitude(::custom_interfaces::srv::GpsToLocal_Request & msg)
  : msg_(msg)
  {}
  Init_GpsToLocal_Request_altitude longitude(::custom_interfaces::srv::GpsToLocal_Request::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GpsToLocal_Request_altitude(msg_);
  }

private:
  ::custom_interfaces::srv::GpsToLocal_Request msg_;
};

class Init_GpsToLocal_Request_latitude
{
public:
  Init_GpsToLocal_Request_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsToLocal_Request_longitude latitude(::custom_interfaces::srv::GpsToLocal_Request::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GpsToLocal_Request_longitude(msg_);
  }

private:
  ::custom_interfaces::srv::GpsToLocal_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::GpsToLocal_Request>()
{
  return custom_interfaces::srv::builder::Init_GpsToLocal_Request_latitude();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GpsToLocal_Response_down
{
public:
  explicit Init_GpsToLocal_Response_down(::custom_interfaces::srv::GpsToLocal_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::GpsToLocal_Response down(::custom_interfaces::srv::GpsToLocal_Response::_down_type arg)
  {
    msg_.down = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::GpsToLocal_Response msg_;
};

class Init_GpsToLocal_Response_east
{
public:
  explicit Init_GpsToLocal_Response_east(::custom_interfaces::srv::GpsToLocal_Response & msg)
  : msg_(msg)
  {}
  Init_GpsToLocal_Response_down east(::custom_interfaces::srv::GpsToLocal_Response::_east_type arg)
  {
    msg_.east = std::move(arg);
    return Init_GpsToLocal_Response_down(msg_);
  }

private:
  ::custom_interfaces::srv::GpsToLocal_Response msg_;
};

class Init_GpsToLocal_Response_north
{
public:
  Init_GpsToLocal_Response_north()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsToLocal_Response_east north(::custom_interfaces::srv::GpsToLocal_Response::_north_type arg)
  {
    msg_.north = std::move(arg);
    return Init_GpsToLocal_Response_east(msg_);
  }

private:
  ::custom_interfaces::srv::GpsToLocal_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::GpsToLocal_Response>()
{
  return custom_interfaces::srv::builder::Init_GpsToLocal_Response_north();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__GPS_TO_LOCAL__BUILDER_HPP_
