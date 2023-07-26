cmake_minimum_required(VERSION 3.5)
project(pgarcia_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(navigation2 REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_util
  nav2_costmap_2d
  nav_msgs
  nav2_core
  tf2
  tf2_geometry_msgs
  tf2_ros
)

include_directories(include)

add_executable(dist_robot_waypoints
  src/dist_robot_waypoints/dist_robot_waypoints_node.cpp
  src/dist_robot_waypoints_main.cpp
)
ament_target_dependencies(dist_robot_waypoints ${dependencies})

install(TARGETS dist_robot_waypoints
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
