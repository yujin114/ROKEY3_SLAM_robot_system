#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from collections import deque

class BFSPathfinder(Node):
    def __init__(self):
        super().__init__('bfs_path_finder')

        # Load waypoint graph
        try:
            yaml_path = os.path.join(
                get_package_share_directory('rokey_pjt'),
                'config',
                'waypoints_test.yaml'
            )

            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.graph = {}
                for wp in data['waypoints']:
                    self.graph[wp['id']] = wp.get('neighbors', [])
                for node, neighbors in self.graph.items():
                    for nbr in neighbors:
                        self.graph.setdefault(nbr, [])
                        if node not in self.graph[nbr]:
                            self.graph[nbr].append(node)

            self.get_logger().info(f"Waypoint graph loaded from: {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            self.graph = {}

        # State
        self.current_pose_id = None
        self.goal_pose_id = None

        # Subscribers
        self.create_subscription(String, '/robot0/bfs/state_pose', self.state_callback, 10)
        self.create_subscription(String, '/robot0/bfs/goal_pose', self.goal_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(String, 'robot0/bfs/path', 10)

        self.get_logger().info("BFS Pathfinder Topic Node Initialized")

    def state_callback(self, msg):
        self.current_pose_id = msg.data
        self.get_logger().info(f"Current pose set to: {self.current_pose_id}")
        self.try_compute_path()

    def goal_callback(self, msg):
        self.goal_pose_id = msg.data
        self.get_logger().info(f"Goal pose set to: {self.goal_pose_id}")
        self.try_compute_path()

    def try_compute_path(self):
        if self.current_pose_id and self.goal_pose_id:
            if self.current_pose_id == self.goal_pose_id:
                self.get_logger().warn("Current pose and goal pose are the same. No path needed.")
                self.path_pub.publish(String(data=""))  # 또는 현재 위치만 포함: data=self.current_pose_id
                return

            path = self.bfs(self.current_pose_id, self.goal_pose_id)
            if path:
                path_str = ','.join(path)
                self.path_pub.publish(String(data=path_str))
                self.get_logger().info(f"Published path: {path_str}")
            else:
                self.path_pub.publish(String(data=""))
                self.get_logger().warn(f"No path found from {self.current_pose_id} to {self.goal_pose_id}")


    def bfs(self, start_id, goal_id):
        visited = set([start_id])
        queue = deque([(start_id, [start_id])])

        while queue:
            current, path = queue.popleft()
            if current == goal_id:
                return path
            for neighbor in self.graph.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None

def main(args=None):
    rclpy.init(args=args)
    node = BFSPathfinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
