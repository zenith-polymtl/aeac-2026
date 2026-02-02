cmake_minimum_required(VERSION 3.8)
project(water_web_server_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(custom_interfaces REQUIRED)


set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "ament_index_cpp"
  "Boost"
  "nlohmann_json"
  "custom_interfaces"
)

add_library(${PROJECT_NAME}_lib
  src/water_web_server_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib std_msgs nlohmann_json rclcpp custom_interfaces)

add_executable(water_web_server_node src/water_web_server_node.cpp)
install(TARGETS water_web_server_node DESTINATION lib/${PROJECT_NAME})

target_link_libraries(water_web_server_node
  ${PROJECT_NAME}_lib
  ament_index_cpp::ament_index_cpp
)

target_include_directories(water_web_server_node_lib PUBLIC
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

