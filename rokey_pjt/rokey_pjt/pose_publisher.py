#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory

class WaypointIdentifier(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.declare_parameter('robot_name', 'robot0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_name}/amcl_pose',
            self.pose_callback,
            10
        )

        self.id_pub = self.create_publisher(
            String,
            f'/{self.robot_name}/bfs/state_pose',
            10
        )

        # Load waypoint YAML
        try:
            yaml_path = os.path.join(
                get_package_share_directory('rokey_pjt'),
                'config',
                'waypoints_test.yaml'
            )
            with open(yaml_path, 'r') as f:
                self.waypoints = yaml.safe_load(f)['waypoints']
                self.get_logger().info(f'Loaded waypoints from: {yaml_path}')
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoint config: {e}")
            self.waypoints = []

        self.threshold = 0.5  # meters

    def pose_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        closest_id, min_dist = None, float('inf')

        for wp in self.waypoints:
            dx = current_x - wp['x']
            dy = current_y - wp['y']
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                closest_id = wp['id']

        if min_dist <= self.threshold and closest_id:
            msg = String()
            msg.data = closest_id
            self.id_pub.publish(msg)
            self.get_logger().info(f'Nearby waypoint detected: {closest_id} (distance: {min_dist:.2f} m)')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointIdentifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
