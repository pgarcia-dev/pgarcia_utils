// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "pgarcia_utils/DistRobotWaypointsNode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace dist_robot_waypoints; //==============

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto dist_robot_waypoints_node = std::make_shared<dist_robot_waypoints::DistRobotWaypointsNode>();//sin dist_robot_path::?
  rclcpp::spin(dist_robot_waypoints_node);

  rclcpp::shutdown();

  return 0;
}
