cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. DÉPENDANCES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- 2. INCLUDES ---
include_directories(include)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
        ${CMAKE_CURRENT_SOURCE_DIR}/gSDK/src/
)

# --- 3. EXÉCUTABLE ---
add_executable(gremsy_node
  src/gremsy.cpp
  gSDK/src/gimbal_interface.cpp 
  gSDK/src/serial_port.cpp
)

# --- 4. LIENS (LINKING) ---
ament_target_dependencies(gremsy_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  custom_interfaces
  tf2
  tf2_geometry_msgs
)

target_link_libraries(gremsy_node pthread)

# --- 5. INSTALLATION ---
install(TARGETS
  gremsy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()