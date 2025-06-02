#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from ament_index_python.packages import get_package_share_directory
import yaml
import os
import math
from collections import deque


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """
    간단한 Euler → Quaternion 변환 함수
    """
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
        super().__init__('bfs_navigator')

        # ──────────────────────────────────────────────────────────
        # 1) waypoints_test.yaml 로드 → graph, 좌표 mapping 구성
        # ──────────────────────────────────────────────────────────
        self.graph = {}
        self.waypoints_pos = {}    # id → (x, y, yaw_rad) 저장용

        try:
            yaml_path = os.path.join(
                get_package_share_directory('rokey_pjt'),
                'config',
                'waypoints_test.yaml'
            )
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

                for wp in data['waypoints']:
                    wp_id = wp['id']
                    x = float(wp['x'])
                    y = float(wp['y'])
                    yaw_deg = float(wp['yaw'])
                    yaw_rad = yaw_deg * math.pi / 180.0

                    neighbors = wp.get('neighbors', [])

                    # 그래프에 양방향으로 추가
                    self.graph.setdefault(wp_id, [])
                    for nbr in neighbors:
                        self.graph[wp_id].append(nbr)
                        self.graph.setdefault(nbr, [])
                        if wp_id not in self.graph[nbr]:
                            self.graph[nbr].append(wp_id)

                    # 좌표 정보 저장
                    self.waypoints_pos[wp_id] = (x, y, yaw_rad)

            self.get_logger().info(f"Loaded waypoints from: {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            self.graph = {}
            self.waypoints_pos = {}

        # ──────────────────────────────────────────────────────────
        # 2) State 값 저장용 변수
        # ──────────────────────────────────────────────────────────
        self.current_pose_id = None
        self.goal_pose_id = None
        self.path_computed = False  # 일단 한 번만 경로 계산/전송하도록 체크

        # ──────────────────────────────────────────────────────────
        # 3) Subscribers
        #    - '/robot0/bfs/state_pose' 토픽(String) → 현재 ID
        #    - '/robot0/bfs/goal_pose' 토픽(String) → 목표    ID
        # ──────────────────────────────────────────────────────────
        self.create_subscription(
            String,
            '/robot0/bfs/state_pose',
            self.state_callback,
            10
        )
        self.create_subscription(
            String,
            '/robot0/bfs/goal_pose',
            self.goal_callback,
            10
        )

        # ──────────────────────────────────────────────────────────
        # 4) Nav2 Action Client 준비
        #    → 최종적으로 goToPose() 호출
        # ──────────────────────────────────────────────────────────
        self.navigator = BasicNavigator(node_name='bfs_nav2_client')
        # 초기 pose (AMCL 등) 대기용 sleep/초기화는 main()에서 처리

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
        """
        state_id, goal_id가 모두 들어왔고 아직 경로를 계산/전송하지 않았다면
        1) BFS로 path (ID 리스트) 계산
        2) 순차적으로 Nav2 goToPose() 실행
        """
        if (self.current_pose_id is None) or (self.goal_pose_id is None):
            return

        # 1) 같을 경우 → 경로 불필요
        if self.current_pose_id == self.goal_pose_id:
            self.get_logger().warn("Current pose == Goal pose → 경로 탐색 불필요")
            self.path_computed = True
            return

        # 2) 이미 경로를 계산/전송했으면 아무것도 안 함
        if self.path_computed:
            return

        # 3) BFS 탐색
        path_ids = self.bfs(self.current_pose_id, self.goal_pose_id)
        if not path_ids:
            self.get_logger().warn(f"No path found: {self.current_pose_id} → {self.goal_pose_id}")
            self.path_computed = True
            return

        self.get_logger().info(f"BFS found path: {path_ids}")
        self.path_computed = True

        # 4) 순차적으로 Nav2 goToPose() 전송
        self.navigate_along(path_ids)


    def bfs(self, start_id: str, goal_id: str):
        """
        기본 BFS 로직 (ID → ID 최단 경로 찾기)
        """
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
        """
        1) path_ids 리스트에서 각 웨이포인트 ID → 좌표로 변환
        2) BasicNavigator.goToPose()로 순차 전송
        3) 도착 여부를 확인한 뒤 다음 웨이포인트 진행
        """
        for idx, wp_id in enumerate(path_ids):
            # 1) 존재하는 ID인지 확인
            if wp_id not in self.waypoints_pos:
                self.get_logger().error(f"Waypoint ID '{wp_id}' not found in waypoints_pos!")
                continue

            x, y, yaw = self.waypoints_pos[wp_id]
            self.get_logger().info(f"[navigate] Sending waypoint '{wp_id}' → ({x:.3f}, {y:.3f}, yaw={yaw:.3f})")

            # 2) PoseStamped 생성
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw

            # 3) goToPose 호출
            self.navigator.goToPose(goal_pose)

            # 4) 도착 여부 체크 루프
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and feedback.current_pose:
                    cx = feedback.current_pose.pose.position.x
                    cy = feedback.current_pose.pose.position.y
                    self.get_logger().info(f"    → 현재 위치: ({cx:.2f}, {cy:.2f})")
                rclpy.spin_once(self, timeout_sec=0.1)

            # 5) 결과 확인
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"    ✓ Waypoint '{wp_id}' 도착 완료.")
                # 웨이포인트별로 잠시 대기할 필요가 있으면 time.sleep() 추가
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"    ⚠️ Waypoint '{wp_id}' 이동 취소됨.")
            elif result == TaskResult.FAILED:
                err_code, err_msg = self.navigator.getTaskError()
                self.get_logger().error(f"    ❌ Waypoint '{wp_id}' 이동 실패: {err_code} - {err_msg}")
            else:
                self.get_logger().warn(f"    ❓ 알 수 없는 TaskResult: {result}")
        # for 루프 끝
        self.get_logger().info("========== 모든 웨이포인트 경로 탐색 완료 ==========")


def main(args=None):
    rclpy.init(args=args)
    node = BFSNavigator()

    # ────────────────────────────────────────────────────────
    # AMCL, TF, Nav2 시스템이 준비될 때까지 잠시 대기 + 활성화
    # (실제 환경에 맞게 시간 늘려도 무방)
    node.get_logger().info("Initializing Nav2 server...")
    node.navigator.waitUntilNav2Active()
    node.get_logger().info("Nav2 is active. Now waiting for state/goal topics...")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
