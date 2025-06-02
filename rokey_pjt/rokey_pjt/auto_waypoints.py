#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
import yaml
import os

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('auto_waypoints')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.declare_parameter(
            'waypoints_P1_yaml',
            '/home/weed/rokey_ws/src/rokey_pjt/config/waypoints.yaml'
        )


    def load_waypoints(self, ids: list):
        path = self.get_parameter('waypoints_yaml').get_parameter_value().string_value
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        waypoints = []
        for wp_id in ids:
            match = next((w for w in data['waypoints'] if w['id'] == wp_id), None)
            if match is None:
                self.get_logger().error(f"Waypoint '{wp_id}' not found in YAML")
                continue

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = match['x']
            pose.pose.position.y = match['y']
            q = quaternion_from_euler(0, 0, match['yaw'])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            waypoints.append(pose)

        return waypoints

    def navigate_sequence(self):
        sequence = ['docking_point', 'front_elevator', 'elevator']
        waypoints = self.load_waypoints(sequence)
        self.navigator.followWaypoints(waypoints)
        self.get_logger().info('Navigating through waypoints...')
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Navigation complete')


def main():
    rclpy.init()
    node = WaypointNavigator()
    node.navigate_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
