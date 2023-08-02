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

#include <rclcpp/rclcpp.hpp>
#include "pgarcia_utils/dist_robot_waypoints_node.hpp"

namespace dist_robot_waypoints
{
  using std::placeholders::_1;

  DistRobotWaypointsNode::DistRobotWaypointsNode()
  : Node("dist_robot_waypoints")
  {
    //next_waypoint_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped> ("lookahead_point", rclcpp::SensorDataQoS(), 
     // std::bind(&DistRobotWaypointsNode::next_waypoint_callback, this, _1));

     path_sub_ = create_subscription<nav_msgs::msg::Path> ("plan", rclcpp::SensorDataQoS(), 
      std::bind(&DistRobotWaypointsNode::path_callback, this, _1));

    dist_robot_next_waypoint_pub_ = create_publisher<std_msgs::msg::Float32>("dist_robot_waypoints", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry> ("/odom", rclcpp::SensorDataQoS(), 
      std::bind(&DistRobotWaypointsNode::odom_callback, this, _1));
    



    tf_buffer_= std::make_shared<tf2_ros::Buffer> (get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS> ("my_costmap");//, this->shared_from_this() ); //*tf_listener_); //*this); //==========================
  //  costmap_ros_->start();

    // Handles global path transformations
    path_handler_ = std::make_unique<pgarcia_utils::PathHandler>( tf2::durationFromSec(0.1), tf_buffer_, costmap_ros_); //================

    test_sub_ = create_subscription<std_msgs::msg::String>(
      "test", 10, std::bind(&DistRobotWaypointsNode::test_callback, this, _1));

  }

  void DistRobotWaypointsNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
   // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%f'", msg->pose.pose.position.x);
   return;
  }

  void DistRobotWaypointsNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "========");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%f %ld'", msg->poses.begin()->pose.position.x, msg->poses.size());
  }

/*
  void DistRobotWaypointsNode::next_waypoint_callback(geometry_msgs::msg::PointStamped::UniquePtr msg)
  {
    static auto message = std_msgs::msg::Float32(); 
    message.data = hypot(msg->point.x, msg->point.y);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=== Receivedd point: x:%f, y:%f. Distance from robot from to point:%f", msg->point.x, msg->point.y, message.data );

    dist_robot_next_waypoint_pub_->publish(message);
  }
  */

  

  void DistRobotWaypointsNode::test_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  

}  // namespace dist_robot_waypoints
