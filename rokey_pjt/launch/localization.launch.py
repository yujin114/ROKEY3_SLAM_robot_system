from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/robot0',
        description='Namespace for the robot'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.getenv('HOME'), 'rokey_ws/maps/P1.yaml'),
        description='Full path to map file'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(os.getenv('HOME'), 'rokey_ws/configs/local2.yaml'),
        description='Full path to the params file'
    )

    launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot4_navigation'),
            '/launch/localization.launch.py'
        ]),
        launch_arguments={
            'namespace': namespace,
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    # remap scan → scan/downsample
    remap_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',  # dummy node to enforce remap
        name='scan_remapper',
        namespace=namespace,
        remappings=[
            ('scan', 'scan/downsample')
        ],
        parameters=[],
        condition=None  # no real transform, only for remapping
    )

    return LaunchDescription([
        declare_namespace,
        declare_map,
        declare_params_file,
        remap_node,
        launch_nav
    ])
