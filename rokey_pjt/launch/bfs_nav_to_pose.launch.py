from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rokey_pjt',
            executable='dock_controller',
            name='dock_controller',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='path_follower',
            name='path_follower',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='pose_publisher',
            name='pose_publisher',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='bfs_path_finder',
            name='bfs_path_finder',
            output='screen'
        ),
    ])
