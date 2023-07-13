// Copyright (c) 2023 Pablo García. pgarcia.developer@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the “License”);
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an “AS IS” BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. 

#include <utility>
#include "pgarcia_utils/DistRobotWaypointsNode.hpp"

#include "rclcpp/rclcpp.hpp"

namespace dist_robot_waypoints
{

using namespace std::chrono_literals;
using std::placeholders::_1;

DistRobotWaypointsNode::DistRobotWaypointsNode()
: Node("dist_robot_waypoints")
{
 // logger_ = node->get_logger();

  next_waypoint_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "lookahead_point", rclcpp::SensorDataQoS(),
    std::bind(&DistRobotWaypointsNode::next_waypoint_callback, this, _1));

  dist_robot_next_waypoint_pub_ = create_publisher<float>("dist_robot_waypoints", 10);

  timer_ = create_wall_timer(100ms, std::bind(&DistRobotWaypointsNode::control_cycle, this));
}

void DistRobotWaypointsNode::next_waypoint_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
//  last_scan_ = std::move(msg);
  return;
}

void DistRobotWaypointsNode::control_cycle()
{
  // Do nothing until the first sensor read
  //if (last_scan_ == nullptr) {
   // return;
  //}

  //vel_pub_->publish(out_vel);

  //RCLCPP_INFO(logger_, "**************");

  return;
}

void
DistRobotWaypointsNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

}  // namespace dist_robot_waypoints
