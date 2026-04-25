cmake_minimum_required(VERSION 3.8)
project(path_follow_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(path_follow_control_core
  src/path_follow_controller.cpp
)

target_include_directories(path_follow_control_core PUBLIC
  include
)

add_executable(path_follow_control_node
  src/path_follow_control_node.cpp
)

target_include_directories(path_follow_control_node PUBLIC
  include
)

target_link_libraries(path_follow_control_node
  path_follow_control_core
)

ament_target_dependencies(path_follow_control_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  path_follow_control_core
  path_follow_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
