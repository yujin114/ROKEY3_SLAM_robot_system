from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_rokey_pjt_bringup = get_package_share_directory('rokey_pjt')
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
        Node(
            package='rokey_pjt',
            executable='map_switcher',
            name='map_switcher',
            output='screen',
            parameters=[
                {'map_p1': PathJoinSubstitution([pkg_rokey_pjt_bringup, 'maps', 'P1.yaml'])},
                {'map_p2': PathJoinSubstitution([pkg_rokey_pjt_bringup, 'maps', 'P2.yaml'])},
                {'waypoints_p1': PathJoinSubstitution([pkg_rokey_pjt_bringup, 'config', 'waypoints_P1.yaml'])},
                {'waypoints_p2': PathJoinSubstitution([pkg_rokey_pjt_bringup, 'config', 'waypoints_P2.yaml'])},
                {'trigger_id2': 'p2_0'},
                {'threshold_dist': 0.5}
            ]
        ),

    ])