cmake_minimum_required(VERSION 3.8)
project(water_web_server_node)
set(EXECUTABLE_NAME water_web_server_node)
set(PACKAGE_NAME water_web_server_node)
set(DEPENDENCIES
  rclcpp
  std_msgs
  ament_index_cpp
  Boost
  nlohmann_json
  custom_interfaces
)

set(LIB_NAME ${PACKAGE_NAME}_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()


add_library(${PROJECT_NAME}_lib
  src/water_web_server_node.cpp
)

ament_target_dependencies(${LIB_NAME} std_msgs nlohmann_json rclcpp custom_interfaces)

add_executable(${PACKAGE_NAME} src/${EXECUTABLE_NAME}.cpp)
install(TARGETS ${PACKAGE_NAME} DESTINATION lib/${PROJECT_NAME})

target_link_libraries(${PACKAGE_NAME}
  ${PROJECT_NAME}_lib
  ament_index_cpp::ament_index_cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
)

install(DIRECTORY web_components/
  DESTINATION share/${PROJECT_NAME}/web_components
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

