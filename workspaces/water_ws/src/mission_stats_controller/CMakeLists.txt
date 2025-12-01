cmake_minimum_required(VERSION 3.8)
project(mission_stats_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "custom_interfaces"
)

add_library(${PROJECT_NAME}_lib
  src/mission_stats_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp std_msgs custom_interfaces)

add_executable(mission_stats_controller src/mission_stats_controller.cpp)
install(TARGETS mission_stats_controller DESTINATION lib/${PROJECT_NAME})

target_link_libraries(mission_stats_controller
  ${PROJECT_NAME}_lib
  ament_index_cpp::ament_index_cpp
)

target_include_directories(mission_stats_controller_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
