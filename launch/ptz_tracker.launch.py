from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ptz_tracker'),
        'config',
        'ptz_tracker.yaml'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='YAML with tracker parameters',
    )

    node = Node(
        package='ptz_tracker',
        executable='ptz_tracker_node',
        name='ptz_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
    )

    return LaunchDescription([
        config_file_arg,
        node,
    ])