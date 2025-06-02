#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from ament_index_python.packages import get_package_share_directory
import yaml
import os
import math
import time
from collections import deque

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)

class BFSNavigator(Node):
    def __init__(self):
        super().__init__('bfs_navigator_v2')
        self.graph = {}
        self.waypoints_pos = {}

        try:
            yaml_path = os.path.join(
                get_package_share_directory('rokey_pjt'), 'config', 'waypoints_test.yaml'
            )
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                for wp in data['waypoints']:
                    wp_id = wp['id']
                    x, y = float(wp['x']), float(wp['y'])
                    yaw = float(wp['yaw']) * math.pi / 180.0
                    neighbors = wp.get('neighbors', [])

                    self.graph.setdefault(wp_id, [])
                    for nbr in neighbors:
                        self.graph[wp_id].append(nbr)
                        self.graph.setdefault(nbr, [])
                        if wp_id not in self.graph[nbr]:
                            self.graph[nbr].append(wp_id)

                    self.waypoints_pos[wp_id] = (x, y, yaw)
            self.get_logger().info(f"Loaded waypoints from: {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")

        self.current_pose_id = None
        self.goal_pose_id = None
        self.path_computed = False

        self.create_subscription(String, '/robot0/bfs/state_pose', self.state_callback, 10)
        self.create_subscription(String, '/robot0/bfs/goal_pose', self.goal_callback, 10)

        self.navigator = BasicNavigator(node_name='bfs_nav2_client')
        self.dock_navigator = TurtleBot4Navigator()
        self.get_logger().info("BFS Navigator Node Initialized")

    def state_callback(self, msg: String):
        self.current_pose_id = msg.data.strip()
        self.get_logger().info(f"[state_callback] Current pose ID = {self.current_pose_id}")
        self.try_compute_and_navigate()

    def goal_callback(self, msg: String):
        self.goal_pose_id = msg.data.strip()
        self.get_logger().info(f"[goal_callback]   Goal    pose ID = {self.goal_pose_id}")
        self.try_compute_and_navigate()

    def try_compute_and_navigate(self):
        if (self.current_pose_id is None) or (self.goal_pose_id is None):
            return

        if self.goal_pose_id == 'p1_0':
            self.get_logger().info("Goal is 'p1_0' → Docking 시작")
            self.dock_navigator.dock()
            self.get_logger().info("Dock command sent")
            return

        if self.dock_navigator.getDockedStatus():
            self.get_logger().info("Currently docked → Undocking 시작")
            self.dock_navigator.undock()
            self.get_logger().info("Undock complete")
            time.sleep(2.0)

        if self.path_computed:
            return

        path_ids = self.bfs(self.current_pose_id, self.goal_pose_id)
        if not path_ids:
            self.get_logger().warn(f"No path found: {self.current_pose_id} → {self.goal_pose_id}")
            return

        self.get_logger().info(f"BFS found path: {path_ids}")
        self.path_computed = True
        self.navigate_along(path_ids)

    def bfs(self, start_id: str, goal_id: str):
        visited = set([start_id])
        queue = deque([(start_id, [start_id])])

        while queue:
            current, route = queue.popleft()
            if current == goal_id:
                return route
            for nbr in self.graph.get(current, []):
                if nbr not in visited:
                    visited.add(nbr)
                    queue.append((nbr, route + [nbr]))
        return None

    def navigate_along(self, path_ids: list):
        for wp_id in path_ids:
            if wp_id not in self.waypoints_pos:
                self.get_logger().error(f"Waypoint ID '{wp_id}' not found!")
                continue

            x, y, yaw = self.waypoints_pos[wp_id]
            self.get_logger().info(f"[navigate] Waypoint '{wp_id}' → ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw

            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and feedback.current_pose:
                    cx = feedback.current_pose.pose.position.x
                    cy = feedback.current_pose.pose.position.y
                    self.get_logger().info(f"    → 현재 위치: ({cx:.2f}, {cy:.2f})")
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"    ✓ Waypoint '{wp_id}' 도착 완료.")
                if wp_id == 'p1_0' and self.goal_pose_id == 'p1_0':
                    self.get_logger().info("도착한 waypoint가 'p1_0'이고 최종 목적지임 → Docking 시작")
                    self.dock_navigator.dock()
                    self.get_logger().info("Dock 명령 전송 완료")
                    time.sleep(2.0)
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"    ⚠️ Waypoint '{wp_id}' 이동 취소됨.")
            elif result == TaskResult.FAILED:
                err_code, err_msg = self.navigator.getTaskError()
                self.get_logger().error(f"    ❌ Waypoint '{wp_id}' 이동 실패: {err_code} - {err_msg}")
            else:
                self.get_logger().warn(f"    ❓ 알 수 없는 TaskResult: {result}")

        self.get_logger().info("========== 모든 웨이포인트 경로 탐색 완료 ==========")

def main(args=None):
    rclpy.init(args=args)
    node = BFSNavigator()
    node.get_logger().info("Initializing Nav2 server...")
    node.navigator.waitUntilNav2Active()
    node.get_logger().info("Nav2 is active. Now waiting for state/goal topics...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()