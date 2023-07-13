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

namespace dist_robot_waypoints
{

using namespace std::chrono_literals;  // NOLINT

class DistRobotWaypointsNode : public rclcpp::Node
{
public:
  DistRobotWaypointsNode();

private:
  void next_path_point_callback(geometry_msgs::msg::PointStamped::UniquePtr msg);
  void control_cycle();

  static const int FORWARD = 0;
  static const int BACK = 1;
  static const int TURN = 2;
  static const int STOP = 3;
  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_forward_2_back();
  bool check_forward_2_stop();
  bool check_back_2_turn();
  bool check_turn_2_forward();
  bool check_stop_2_forward();

  const rclcpp::Duration TURNING_TIME {2s};
  const rclcpp::Duration BACKING_TIME {2s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.3f;
  static constexpr float OBSTACLE_DISTANCE = 1.0f;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr next_path_point_sub_;
  rclcpp::Publisher<float>::SharedPtr dist_robot_next_path_point_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dist_robot_waypoints

#endif  // DIST_ROBOT_WAYPOINTS_HPP_
