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
import threading
from collections import deque

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # ─── 1) BasicNavigator 인스턴스 생성 ──────────────────────────────────────────
        self.nav_navigator = BasicNavigator()
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

        # ─── 3) 경로 요청을 모아둘 큐 준비 ──────────────────────────────────────────────
        self.path_queue = deque()
        self.queue_lock = threading.Lock()
        self.is_navigating = False

        # ─── 4) 구독: /robot0/bfs/path ───────────────────────────────────────────────
        self.create_subscription(
            String,
            '/robot0/bfs/path',
            self.path_callback,
            10
        )
        self.get_logger().info('[PathFollower] Subscribed to /robot0/bfs/path')

        # ─── 5) 백그라운드 워커 스레드 시작 ─────────────────────────────────────────
        t = threading.Thread(target=self.navigation_worker, daemon=True)
        t.start()

    def path_callback(self, msg: String):
        """
        msg.data: "p1_0,p1_1,p1_2,..." 형태의 전체 waypoint ID 문자열
        이 중 첫 번째 인덱스(현재 위치)와 두 번째 인덱스(다음 목적지)만 추출하여 큐에 저장
        """
        path_str = msg.data.strip()
        if not path_str:
            self.get_logger().warn('[PathFollower] 빈 경로가 도착했습니다.')
            return

        waypoint_ids = [wp_id.strip() for wp_id in path_str.split(',') if wp_id.strip()]
        self.get_logger().info(f'[PathFollower] Received full path: {waypoint_ids}')

        # “현재 위치 + 다음 목적지”가 둘 다 있어야 함
        if len(waypoint_ids) <= 1:
            self.get_logger().warn('[PathFollower] 이동할 경로가 없습니다. (목표가 하나 이하)')
            return

        current_id = waypoint_ids[0]
        next_id    = waypoint_ids[1]

        # 다음 목적지가 waypoint_dict에 반드시 있어야 함
        if next_id not in self.waypoint_dict:
            self.get_logger().error(f'[PathFollower] 다음 목적지 ID가 없습니다: {next_id}')
            return

        # 큐 초기화 후 “[현재, 다음]” 페어만 저장
        with self.queue_lock:
            self.path_queue.clear()
            self.path_queue.append([current_id, next_id])
        self.get_logger().info(f'[PathFollower] 오직 다음 목표만 큐에 추가됨 → [{current_id} → {next_id}]')

    def navigation_worker(self):
        """
        별도 스레드에서 큐를 모니터링하다가, 경로가 들어오면 goToPose → waitForTaskComplete 처리
        """
        while rclpy.ok():
            if not self.is_navigating:
                with self.queue_lock:
                    if self.path_queue:
                        waypoint_pair = self.path_queue.popleft()
                        self.is_navigating = True
                    else:
                        waypoint_pair = None

                if waypoint_pair:
                    current_id, next_id = waypoint_pair
                    self.get_logger().info(f'[NavigationWorker] 새 목표 시작: {current_id} → {next_id}')

                    # 다음 목적지 하나만 이동
                    coords = self.waypoint_dict[next_id]
                    self.get_logger().info(f"[NavigationWorker] → '{next_id}' 이동 시도: "
                                           f"x={coords['x']:.3f}, y={coords['y']:.3f}, yaw={coords['yaw']:.3f}")
                    target_pose = self.build_pose(coords['x'], coords['y'], coords['yaw'])

                    self.nav_navigator.goToPose(target_pose)
                    result = self.nav_navigator.waitForTaskComplete()

                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info(f"[NavigationWorker] '{next_id}' 도착 완료")
                    else:
                        self.get_logger().warn(f"[NavigationWorker] '{next_id}' 실패 또는 취소 (result={result})")

                    self.get_logger().info('[NavigationWorker] 단일 목표 이동 끝')
                    self.is_navigating = False

            time.sleep(0.1)  # CPU 과부하 방지

    def build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        x, y (meter)와 yaw (rad)를 PoseStamped로 변환
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav_navigator.get_clock().now().to_msg()
        pose.pose.position = Point(x=x, y=y, z=0.0)

        q = self.euler_to_quaternion(0.0, 0.0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float):
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
