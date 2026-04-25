cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mapping REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(simple_slam_lib
  src/loop_closure.cpp
  src/simple_slam.cpp
  src/slam_localization.cpp
  src/slam_mapper.cpp
)

target_include_directories(simple_slam_lib PUBLIC
  include
)

ament_target_dependencies(simple_slam_lib
  builtin_interfaces
  geometry_msgs
  mapping
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(simple_slam_lib
  mapping::occupancy_mapper_lib
)

add_executable(simple_slam_node
  src/simple_slam_node.cpp
)

target_include_directories(simple_slam_node PUBLIC
  include
)

target_link_libraries(simple_slam_node
  simple_slam_lib
)

ament_target_dependencies(simple_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  simple_slam_lib
  simple_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
