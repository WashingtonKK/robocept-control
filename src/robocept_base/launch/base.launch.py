"""Launch the differential-drive base driver node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robocept_base'),
        'config', 'base.yaml',
    )

    base_driver = Node(
        package='robocept_base',
        executable='base_driver',
        name='base_driver',
        namespace='robocept',
        parameters=[config],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
        ],
        output='screen',
    )

    return LaunchDescription([
        base_driver,
    ])
