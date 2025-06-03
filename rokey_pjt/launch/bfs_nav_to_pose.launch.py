from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rokey_pjt',
            executable='dock_controller',
            name='dock_controller',
            namespace='robot0',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='path_follower',
            name='path_follower',
            namespace='robot0',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='pose_publisher',
            name='pose_publisher',
            namespace='robot0',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='bfs_path_finder',
            name='bfs_path_finder',
            namespace='robot0',
            output='screen'
        ),
    ])

