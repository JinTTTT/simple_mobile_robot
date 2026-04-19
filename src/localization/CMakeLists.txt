cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

# 1. tell cmake which source file to complile into executable
add_executable(particle_filter_localization_node 
  src/particle_filter_localization_node.cpp
  src/particle_filter.cpp
)

add_executable(kalman_localization_node
  src/kalman_localization_node.cpp
  src/kalman_filter.cpp
)

# 1. tell cmake where to find header files
target_include_directories(particle_filter_localization_node PUBLIC
  include
)

target_include_directories(kalman_localization_node PUBLIC
  include
)

# 2. link dependecies to executable
ament_target_dependencies(particle_filter_localization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(kalman_localization_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 3. copy compile binary to install/ , ros2 run can find it
install(TARGETS
  particle_filter_localization_node
  kalman_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
