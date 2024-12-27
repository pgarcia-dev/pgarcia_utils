#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Author: Pablo García
Email: pgarcia.developer@gmail.com

This ROS 2 node subscribes to a laser scan topic, then publishes a line marker
in RViz2 for each laser beam. The line marker extends from the robot's origin
(rplidar_link) to the point where the laser beam collides with an obstacle.
This way, you can visualize not only the hit points, but the entire ray.

Make sure to:
- Update the laser frame and topic if they differ from your robot configuration.
- Add the Marker display in RViz2 and select the correct topic.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LaserRayVisualizer(Node):
    def __init__(self):
        super().__init__('laser_ray_visualizer')

        # Subscribe to the laser scan topic
        # Update '/scan' if your laser scan topic name differs
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Publisher for the marker
        self.marker_pub = self.create_publisher(Marker, 'laser_rays', 10)

        # Laser frame (common for TurtleBot 4 might be 'base_scan' or 'lidar_link')
        self.laser_frame = 'rplidar_link'  
        
        # Timer to periodically publish if needed, or we can just publish on callback
        # but LaserScan is usually frequent enough, so no need for extra timers.
    
    def laser_callback(self, msg: LaserScan):
        # Create the marker for the rays
        marker = Marker()
        marker.header.frame_id = self.laser_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # TYPE_LINE_LIST: Each two points form a line segment
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # Set marker properties
        # Set a color visible on RViz2 (e.g., blue lines)
        marker.scale.x = 0.01  # Thickness of the line
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Position is defined by pairs of points (start and end)
        points = []
        
        # We'll use the angle increment to get direction of each ray
        angle = msg.angle_min
        
        # Iterate through each laser beam
        # For each beam, we create two points: start (0,0,0) and end at the obstacle
        for r in msg.ranges:
            # If the reading is valid (not inf and less than max range)
            if r < msg.range_max and r > msg.range_min:
                # Calculate the endpoint of the beam
                # Laser frame assumed at (0,0)
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # Start point of the line (robot origin)
                start_point = Point()
                start_point.x = 0.0
                start_point.y = 0.0
                start_point.z = 0.0
                
                # End point of the line (collision point)
                end_point = Point()
                end_point.x = x
                end_point.y = y
                end_point.z = 0.0
                
                points.append(start_point)
                points.append(end_point)
            
            # Increment the angle for the next beam
            angle += msg.angle_increment
        
        # Assign the points to the marker
        marker.points = points
        
        # Publish the marker
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LaserRayVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
