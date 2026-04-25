cmake_minimum_required(VERSION 3.8)
project(motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(motion_planning_core
  src/motion_planner.cpp
)

target_include_directories(motion_planning_core PUBLIC
  include
)

ament_target_dependencies(motion_planning_core
  builtin_interfaces
  nav_msgs
  geometry_msgs
)

add_executable(motion_planning_node
  src/motion_planning_node.cpp
)

target_include_directories(motion_planning_node PUBLIC
  include
)

target_link_libraries(motion_planning_node
  motion_planning_core
)

ament_target_dependencies(motion_planning_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  motion_planning_core
  motion_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
