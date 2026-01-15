// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/GpsToLocal.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__GPS_TO_LOCAL__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__GPS_TO_LOCAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__GpsToLocal_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__GpsToLocal_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GpsToLocal_Request_
{
  using Type = GpsToLocal_Request_<ContainerAllocator>;

  explicit GpsToLocal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
    }
  }

  explicit GpsToLocal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
    }
  }

  // field types and members
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;

  // setters for named parameter idiom
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__GpsToLocal_Request
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__GpsToLocal_Request
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsToLocal_Request_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsToLocal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsToLocal_Request_

// alias to use template instance with default allocator
using GpsToLocal_Request =
  custom_interfaces::srv::GpsToLocal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__GpsToLocal_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__GpsToLocal_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GpsToLocal_Response_
{
  using Type = GpsToLocal_Response_<ContainerAllocator>;

  explicit GpsToLocal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->north = 0.0;
      this->east = 0.0;
      this->down = 0.0;
    }
  }

  explicit GpsToLocal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->north = 0.0;
      this->east = 0.0;
      this->down = 0.0;
    }
  }

  // field types and members
  using _north_type =
    double;
  _north_type north;
  using _east_type =
    double;
  _east_type east;
  using _down_type =
    double;
  _down_type down;

  // setters for named parameter idiom
  Type & set__north(
    const double & _arg)
  {
    this->north = _arg;
    return *this;
  }
  Type & set__east(
    const double & _arg)
  {
    this->east = _arg;
    return *this;
  }
  Type & set__down(
    const double & _arg)
  {
    this->down = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__GpsToLocal_Response
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__GpsToLocal_Response
    std::shared_ptr<custom_interfaces::srv::GpsToLocal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsToLocal_Response_ & other) const
  {
    if (this->north != other.north) {
      return false;
    }
    if (this->east != other.east) {
      return false;
    }
    if (this->down != other.down) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsToLocal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsToLocal_Response_

// alias to use template instance with default allocator
using GpsToLocal_Response =
  custom_interfaces::srv::GpsToLocal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct GpsToLocal
{
  using Request = custom_interfaces::srv::GpsToLocal_Request;
  using Response = custom_interfaces::srv::GpsToLocal_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__GPS_TO_LOCAL__STRUCT_HPP_
