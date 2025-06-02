#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, PoseStamped
from nav2_msgs.srv import LoadMap
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import NavigateToPose
import tf_transformations
import yaml
import math
import time


class WaypointMapSwitcher(Node):
    def __init__(self):
        super().__init__('map_switcher')

        # ─── 1) 노드 파라미터 선언 ─────────────────────────────────────────────────
        # 네임스페이스 (예: robot0, robot1 등)
        self.declare_parameter('namespace', 'robot0')
        # P1 맵 yaml 경로, P2 맵 yaml 경로
        self.declare_parameter('map_p1', 'maps/P1.yaml')
        self.declare_parameter('map_p2', 'maps/P2.yaml')
        self.declare_parameter('waypoints_p1', 'config/waypoints_P1.yaml')
        self.declare_parameter('waypoints_p2', 'config/waypoints_P2.yaml')
        # 전환 기준이 되는 waypoint ID
        self.declare_parameter('trigger_id', 'elevator')
        # “도달” 판정용 허용 오차 (meter)
        self.declare_parameter('threshold_dist', 0.5)

        # 파라미터 값 읽기
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.map_p1_path = self.get_parameter('map_p1').get_parameter_value().string_value
        self.map_p2_path = self.get_parameter('map_p2').get_parameter_value().string_value
        self.waypoints_p1_path = self.get_parameter('waypoints_p1').get_parameter_value().string_value
        self.waypoints_p2_path = self.get_parameter('waypoints_p2').get_parameter_value().string_value
        self.trigger_id = self.get_parameter('trigger_id').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold_dist').get_parameter_value().double_value

        # 현재 어떤 맵을 사용 중인지 (초기값: P1)
        self.current_map = 'P1'

        # ─── 2) 웨이포인트 파일 로드 ─────────────────────────────────────────────────
        self.waypoints_p1 = self.load_waypoints(self.waypoints_p1_path)
        self.waypoints_p2 = self.load_waypoints(self.waypoints_p2_path)

        # 각 웨이포인트 dict 구조:
        #   {'elevator': {'x':3.2, 'y':1.5, 'yaw':0.0}, ... }

        # ─── 3) 퍼블리셔 & 서비스 클라이언트 생성 ─────────────────────────────────────
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{self.namespace}/initialpose',
            10
        )

        # MAP 서버의 LoadMap 서비스 (nav2_map_server)
        self.cli_load_map = self.create_client(
            LoadMap,
            f'/{self.namespace}/map_server/load_map'
        )

        # Lifecycle Manager (localization) 에게 state change 요청
        self.cli_lifecycle = self.create_client(
            ChangeState,
            f'/{self.namespace}/lifecycle_manager_localization/change_state'
        )

        # AMCL 위치 정보를 얻기 위해 /amcl_pose 구독
        self.amcl_sub = self.create_subscription(
            PoseStamped,
            f'/{self.namespace}/amcl_pose',
            self.amcl_callback,
            10
        )

        # ─── 4) 서비스 준비 대기 ─────────────────────────────────────────────────────
        while not self.cli_load_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[waypoint_map_switcher] Waiting for map_service...')
        while not self.cli_lifecycle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[waypoint_map_switcher] Waiting for lifecycle_service...')

        # ─── 5) 내부 상태 초기화 ─────────────────────────────────────────────────────
        self.is_switching = False
        self.get_logger().info('[waypoint_map_switcher] Ready. Currently using: P1')


    def load_waypoints(self, yaml_path) -> dict:
        """
        YAML에서 waypoints 리스트를 읽고,
        id를 key로 하는 딕셔너리로 반환.
        """
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        wp_dict = {}
        for wp in data.get('waypoints', []):
            wp_dict[wp['id']] = {
                'x': float(wp['x']),
                'y': float(wp['y']),
                'yaw': float(wp.get('yaw', 0.0))
            }
        return wp_dict


    def amcl_callback(self, msg: PoseStamped):
        """
        AMCL에서 퍼블리시하는 현재 위치를 받아서,
        'elevator' 웨이포인트와의 거리를 계산.
        threshold 이내이면 맵 전환 트리거.
        """
        if self.is_switching:
            return

        # 현재 맵이 P1인지 P2인지에 따라 웨이포인트 dict 선택
        if self.current_map == 'P1':
            target_wp = self.waypoints_p1.get(self.trigger_id, None)
        else:
            target_wp = self.waypoints_p2.get(self.trigger_id, None)

        if target_wp is None:
            self.get_logger().warn(f"[waypoint_map_switcher] '{self.trigger_id}' not found on {self.current_map}")
            return

        # AMCL PoseStamped: position = msg.pose.position
        dx = msg.pose.position.x - target_wp['x']
        dy = msg.pose.position.y - target_wp['y']
        dist = math.hypot(dx, dy)

        if dist < self.threshold:
            self.get_logger().info(f"[waypoint_map_switcher] Reached '{self.trigger_id}' on {self.current_map} (dist: {dist:.2f}), switching map...")
            self.is_switching = True
            self.switch_map()
            # 맵 전환 및 initialpose 퍼블리시 이후
            # 다시 감지를 허용하기 전 잠깐 대기
            time.sleep(2.0)
            self.is_switching = False


    def switch_map(self):
        """
        1) 현재 localization(AMCL) deactivate → cleanup
        2) 다른 맵으로 load_map
        3) localization configure → activate
        4) 새 맵에서 same ID 웨이포인트를 initialpose로 퍼블리시
        5) current_map 업데이트
        """
        # 1) AMCL 비활성화 → Cleanup
        self.change_lifecycle_state(Transition.TRANSITION_DEACTIVATE)
        self.change_lifecycle_state(Transition.TRANSITION_CLEANUP)
        self.get_logger().info(f"[waypoint_map_switcher] '{self.current_map}' localization cleaned up")

        # 2) 맵 전환: 현재 P1이면 P2로, P2이면 P1로
        if self.current_map == 'P1':
            new_map = self.map_p2_path
            new_wp_dict = self.waypoints_p2
            next_map_label = 'P2'
        else:
            new_map = self.map_p1_path
            new_wp_dict = self.waypoints_p1
            next_map_label = 'P1'

        # LoadMap 요청
        self.load_new_map(new_map)
        self.get_logger().info(f"[waypoint_map_switcher] Loaded {next_map_label} map: {new_map}")

        # 3) AMCL 재설정: Configure → Activate
        self.change_lifecycle_state(Transition.TRANSITION_CONFIGURE)
        self.change_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        self.get_logger().info(f"[waypoint_map_switcher] '{next_map_label}' localization activated")

        # 4) 새 맵의 동일 ID 웨이포인트 좌표를 initialpose로 퍼블리시
        wp = new_wp_dict[self.trigger_id]
        self.publish_init_pose(wp)
        self.get_logger().info(f"[waypoint_map_switcher] Published initialpose for '{self.trigger_id}' on {next_map_label}: (x={wp['x']}, y={wp['y']}, yaw={wp['yaw']})")

        # 5) current_map 업데이트
        self.current_map = next_map_label


    def change_lifecycle_state(self, transition_id: int):
        """
        Lifecycle Manager에 ChangeState 요청을 보내서 AMCL을 
        deactivate, cleanup, configure, activate 시킴.
        """
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.cli_lifecycle.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"[waypoint_map_switcher] Lifecycle transition {transition_id} succeeded")
        else:
            self.get_logger().warn(f"[waypoint_map_switcher] Lifecycle transition {transition_id} failed")


    def load_new_map(self, map_path: str):
        """
        nav2_map_server의 LoadMap 서비스를 호출하여 
        새로운 맵 YAML을 로드한다.
        """
        req = LoadMap.Request()
        req.map_url = map_path
        future = self.cli_load_map.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def publish_init_pose(self, wp: dict):
        """
        주어진 웨이포인트 딕셔너리(wp: {'x':.., 'y':.., 'yaw':..})를
        PoseWithCovarianceStamped로 변환하여 /initialpose에 퍼블리시.
        """
        quat = tf_transformations.quaternion_from_euler(0, 0, wp['yaw'])

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose = Pose(
            position=Point(x=wp['x'], y=wp['y'], z=0.0),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )
        # Covariance (기본값)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.initpose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMapSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
