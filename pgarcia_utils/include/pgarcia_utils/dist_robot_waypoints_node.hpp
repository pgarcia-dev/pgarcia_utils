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

#ifndef DIST_ROBOT_WAYPOINTS_HPP_
#define DIST_ROBOT_WAYPOINTS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2_ros/transform_listener.h>///////////
#include <tf2_ros/static_transform_broadcaster.h>//////////////
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"

namespace dist_robot_waypoints
{
  
  class DistRobotWaypointsNode : public rclcpp::Node
  {
    public:
      DistRobotWaypointsNode();
      
    private:
      void next_waypoint_callback(geometry_msgs::msg::PointStamped::UniquePtr msg);

      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr next_waypoint_point_sub_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_robot_next_waypoint_pub_;
     
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_; 

  };

}  // namespace dist_robot_waypoints

#endif  // DIST_ROBOT_WAYPOINTS_HPP_
