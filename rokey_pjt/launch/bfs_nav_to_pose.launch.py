from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot0',
        description='Robot namespace'
    )

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        ns_arg,

        Node(
            package='rokey_pjt',
            executable='dock_controller',
            name='dock_controller',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='path_follower',
            name='path_follower',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='pose_publisher',
            name='pose_publisher',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='bfs_path_finder',
            name='bfs_path_finder',
            namespace=namespace,
            output='screen'
        ),
    ])
