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
#include <algorithm>

#include "pgarcia_utils/DistRobotWaypointsNode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dist_robot_waypoints
{

  using std::placeholders::_1;

  DistRobotWaypointsNode::DistRobotWaypointsNode()
  : Node("dist_robot_waypoints")
  {
  // logger_ = node->get_logger();

    next_waypoint_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(     
      "lookahead_point", rclcpp::SensorDataQoS(),
      std::bind(&DistRobotWaypointsNode::next_waypoint_callback, this, _1));

    dist_robot_next_waypoint_pub_ = create_publisher<std_msgs::msg::Float32>("dist_robot_waypoints", 10);
  }

  void DistRobotWaypointsNode::next_waypoint_callback(geometry_msgs::msg::PointStamped::UniquePtr msg)
  {
    static auto message = std_msgs::msg::Float32(); 
    message.data = hypot(msg->point.x, msg->point.y);

    dist_robot_next_waypoint_pub_->publish(message);
  }

}  // namespace dist_robot_waypoints
