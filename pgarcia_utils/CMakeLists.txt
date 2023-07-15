cmake_minimum_required(VERSION 3.5)
project(pgarcia_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_util
)

include_directories(include)

add_executable(dist_robot_waypoints
  src/dist_robot_waypoints/DistRobotWaypointsNode.cpp
  src/dist_robot_waypoints_main.cpp
)
ament_target_dependencies(dist_robot_waypoints ${dependencies})

install(TARGETS dist_robot_waypoints
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  #DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
