cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

# Requis pour build ROS2
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Génération des interfaces (srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GpsToLocal.srv"
)

# Export runtime pour que les autres packages puissent l'utiliser
ament_export_dependencies(rosidl_default_runtime)

ament_package()