#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped 
from rmf_fleet_msgs.msg import Location, RobotState, RobotMode
from rmf_task_msgs.msg import TaskSummary
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from transforms3d.euler import quat2euler, euler2quat


class Turtlebot4FleetAdapter(Node):
    def __init__(self):
        super().__init__('tb4_adapter')

        # 파라미터(확장용)
        self.declare_parameter('config_file', '')
        self.declare_parameter('nav_graph_file', '')

        self.current_pose = None

        # 1) 현재 위치 구독 (/robot1/amcl_pose)
        self.create_subscription(
        PoseWithCovarianceStamped,
        '/robot1/amcl_pose',
        self.pose_callback,
        10
        )

        # 2) 로봇 상태(RobotState) 게시 (/robot_state)
        self.robot_state_pub = self.create_publisher(
            RobotState,
            '/robot_state',
            10
        )

        # 3) Nav2 NavigateToPose 액션 클라이언트 (/robot1/navigate_to_pose)
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            '/robot1/navigate_to_pose'
        )

        # 4) Task 수신 (/task_summaries)
        self.create_subscription(
            TaskSummary,
            '/task_summaries',
            self.task_callback,
            10
        )

        self.get_logger().info("TB4 Fleet Adapter Initialized.")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info("pose_callback 호출됨")

        pose = msg.pose.pose
        self.get_logger().info(f"받은 위치: x={pose.position.x}, y={pose.position.y}")

        robot_state = RobotState()
        robot_state.name = "robot1"
        robot_state.location = Location()
        robot_state.location.x = pose.position.x
        robot_state.location.y = pose.position.y

        q = pose.orientation
        try:
            yaw, _, _ = quat2euler([q.w, q.x, q.y, q.z])
            self.get_logger().info(f"계산된 yaw: {yaw}")
        except Exception as e:
            self.get_logger().error(f"Quaternion 변환 실패: {e}")
            return

        robot_state.location.yaw = yaw
        robot_state.mode.mode = RobotMode.MODE_MOVING

        self.robot_state_pub.publish(robot_state)
        self.get_logger().info("→ /robot_state 퍼블리시 완료")

    def task_callback(self, msg: TaskSummary):
        """
        RMF Core로부터 전달된 TaskSummary를 받아 Nav2로 follow path 요청을 보냅니다.
        - TaskSummary.state가 STATE_ACTIVE인 경우에만 처리
        - 여기 예제에서는 TaskSummary.description 안에 목적지 정보를
          넣었다고 가정하고, 간단히 msg.payload에서 x,y 좌표를 파싱합니다.
        """
        if msg.state != TaskSummary.STATE_ACTIVE:
            return

        self.get_logger().info(f"New task received: {msg.task_id}")

        # --- 예시: msg.requested Task payload에서 목적지 좌표 추출 ---
        try:
            import json
            payload = json.loads(msg.payload)
            tx = float(payload['x'])
            ty = float(payload['y'])
            tyaw = float(payload['yaw'])
        except Exception as e:
            self.get_logger().error(f"Failed to parse task payload: {e}")
            return

        # NavigateToPose 메시지 생성
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = tx
        goal_pose.pose.position.y = ty
        # quaternion 설정 (yaw만 사용)
        q = euler2quat(tyaw, 0.0, 0.0)  # yaw, pitch, roll
        goal_pose.pose.orientation.x = q[1]
        goal_pose.pose.orientation.y = q[2]
        goal_pose.pose.orientation.z = q[3]
        goal_pose.pose.orientation.w = q[0]

        # Nav2 액션 호출
        self.navigate_to(goal_pose)

    def navigate_to(self, pose: PoseStamped):
        """
        Nav2의 NavigateToPose 액션 서버로 goal을 전송합니다.
        """
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending NavigateToPose goal...")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        액션 서버가 Goal을 받았는지 확인하고, 이후 결과 콜백 등록
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            return

        self.get_logger().info("Nav2 goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """
        Nav2 경로 수행 결과 처리 (성공/실패)
        """
        result = future.result().result
        status = future.result().status

        if status == NavigateToPose.Result.Response.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2: Goal Succeeded")
        else:
            self.get_logger().warn(f"Nav2: Goal Failed with status {status}")


def main(args=None):
    rclpy.init(args=args)
    adapter = Turtlebot4FleetAdapter()
    rclpy.spin(adapter)
    adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()