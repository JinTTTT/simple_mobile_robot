cmake_minimum_required(VERSION 3.8)
project(slam_fastslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mapping REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(slam_fastslam_core
  src/fastslam.cpp
  src/likelihood_field.cpp
  src/motion_model.cpp
  src/particle_resampler.cpp
  src/scan_scorer.cpp
)

target_include_directories(slam_fastslam_core PUBLIC
  include
)

ament_target_dependencies(slam_fastslam_core
  builtin_interfaces
  geometry_msgs
  mapping
  nav_msgs
  sensor_msgs
)

target_link_libraries(slam_fastslam_core
  mapping::occupancy_mapper_lib
)

add_executable(fastslam_node
  src/fastslam_node.cpp
  src/utils.cpp
)

target_include_directories(fastslam_node PUBLIC
  include
)

ament_target_dependencies(fastslam_node
  builtin_interfaces
  geometry_msgs
  mapping
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(fastslam_node
  slam_fastslam_core
)

if(BUILD_TESTING)
  ament_add_gtest(test_particle_resampler
    test/test_particle_resampler.cpp
  )

  target_link_libraries(test_particle_resampler
    slam_fastslam_core
  )

  ament_add_gtest(test_likelihood_field
    test/test_likelihood_field.cpp
  )

  target_link_libraries(test_likelihood_field
    slam_fastslam_core
  )

  ament_add_gtest(test_scan_scorer
    test/test_scan_scorer.cpp
  )

  target_link_libraries(test_scan_scorer
    slam_fastslam_core
  )
endif()

install(TARGETS
  slam_fastslam_core
  fastslam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
