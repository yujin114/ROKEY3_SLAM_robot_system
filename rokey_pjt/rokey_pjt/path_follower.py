#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math
import time

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ─── 1) BasicNavigator 인스턴스 생성 ──────────────────────────────────────────
        self.nav_navigator = BasicNavigator()
        # Nav2가 완전히 활성화될 때까지 대기
        #self.nav_navigator.waitUntilNav2Active()
        self.get_logger().info('[PathFollower] Nav2 is active.')

        # ─── 2) Waypoint 좌표 로드 (YAML → dict) ─────────────────────────────────────
        try:
            yaml_path = os.path.join(
                get_package_share_directory('rokey_pjt'),
                'config',
                'waypoints_test.yaml'
            )
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            # { 'p1_0': {'x': ..., 'y': ..., 'yaw': ...}, ... }
            self.waypoint_dict = {}
            for wp in data.get('waypoints', []):
                self.waypoint_dict[wp['id']] = {
                    'x': float(wp['x']),
                    'y': float(wp['y']),
                    'yaw': float(wp.get('yaw', 0.0))
                }
            self.get_logger().info(f'[PathFollower] Loaded waypoints from: {yaml_path}')
        except Exception as e:
            self.get_logger().error(f'[PathFollower] Failed to load waypoints: {e}')
            self.waypoint_dict = {}

        # ─── 3) 구독: /robot0/bfs/path ───────────────────────────────────────────────
        self.create_subscription(
            String,
            '/robot0/bfs/path',
            self.path_callback,
            10
        )
        self.get_logger().info('[PathFollower] Subscribed to /robot0/bfs/path')

        # ─── 4) 내부 상태 초기화 ─────────────────────────────────────────────────────
        self.is_navigating = False

    def path_callback(self, msg: String):
        """
        msg.data: "p1_0,p1_1,p1_2,..." 형태의 waypoint ID 문자열
        """
        if self.is_navigating:
            self.get_logger().warn('[PathFollower] 이미 경로 이동 중입니다. 무시...')
            return

        path_str = msg.data.strip()
        if not path_str:
            self.get_logger().warn('[PathFollower] 빈 경로가 도착했습니다.')
            return

        waypoint_ids = [wp_id.strip() for wp_id in path_str.split(',') if wp_id.strip()]
        self.get_logger().info(f'[PathFollower] Received path: {waypoint_ids}')

        # 현재 위치만 주어진 경우 이동 안 함
        if len(waypoint_ids) <= 1:
            self.get_logger().warn('[PathFollower] 이동할 경로가 없습니다. (목표가 하나 이하)')
            return

        missing = [wp for wp in waypoint_ids if wp not in self.waypoint_dict]
        if missing:
            self.get_logger().error(f'[PathFollower] 다음 ID의 좌표가 YAML에 없습니다: {missing}')
            return

        self.is_navigating = True
        for idx, wp_id in enumerate(waypoint_ids[1:]):  # 첫 번째는 현재 위치로 간주하고 skip
            coords = self.waypoint_dict[wp_id]

            # ▶ 추가: 이동 시작 로그
            self.get_logger().info(f"[PathFollower] 이동 시작: '{wp_id}'")

            target_pose = self.build_pose(coords['x'], coords['y'], coords['yaw'])
            self.get_logger().info(f"[PathFollower] ({idx+1}/{len(waypoint_ids)-1}) → '{wp_id}'로 이동 시도: "
                                f"x={coords['x']:.3f}, y={coords['y']:.3f}, yaw={coords['yaw']:.3f}")

            self.nav_navigator.goToPose(target_pose)
            result = self.nav_navigator.waitForTaskComplete()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"[PathFollower] '{wp_id}' 도착 완료")
                time.sleep(1.0)
            else:
                self.get_logger().warn(f"[PathFollower] '{wp_id}' 이동 실패 or 취소됨 (result={result}). 다음 목표로 계속.")
                time.sleep(1.0)


        self.get_logger().info('[PathFollower] 경로 이동이 모두 완료되었습니다.')
        self.is_navigating = False


    def build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        x, y (meter)와 yaw (rad)를 PoseStamped로 변환
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav_navigator.get_clock().now().to_msg()

        pose.pose.position = Point(x=x, y=y, z=0.0)
        # yaw(rad) → quaternion
        q = self.euler_to_quaternion(0.0, 0.0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float):
        """
        단순한 Euler → Quaternion 변환 (자체 구현)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
