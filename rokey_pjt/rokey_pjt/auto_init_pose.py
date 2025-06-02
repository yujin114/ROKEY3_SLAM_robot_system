#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import Undock
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
import time

def create_pose(x, y, yaw_deg, navigator):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

class InitPoseUndockNode(Node):
    def __init__(self):
        super().__init__('initpose_undock_node')

        # 1. BasicNavigator 초기화 (robot4 namespace)
        self.navigator = BasicNavigator(namespace='/robot0')

        # 2. initial pose 설정
        initial_pose = create_pose(0.169, 0.014, 0.002, self.navigator)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 위치 설정 중...')
        time.sleep(1.0)

        # 3. Nav2 활성화 대기
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 활성화 완료')

        # 4. Undock 액션 클라이언트 생성
        self.undock_client = ActionClient(self, Undock, '/robot0/undock')

        # 5. 액션 서버 연결 후 목표 전송
        self.undock_client.wait_for_server()
        self.send_undock_goal()

    def send_undock_goal(self):
        goal_msg = Undock.Goal()
        self.get_logger().info('Undocking 요청 중...')
        self.undock_future = self.undock_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self.undock_future.add_done_callback(self.undock_response_cb)

    def feedback_cb(self, feedback_msg):
        self.get_logger().info('Undocking 진행 중...')

    def undock_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undocking goal rejected!')
            rclpy.shutdown()
            return
        self.get_logger().info('Undocking goal accepted.')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.undock_result_cb)

    def undock_result_cb(self, future):
        result = future.result().result
        self.get_logger().info('Undocking 완료.')
        self.navigator.destroy_node()
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitPoseUndockNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
