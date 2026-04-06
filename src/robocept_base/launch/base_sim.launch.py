"""Launch the Gazebo-facing base adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robocept_base'),
        'config', 'base_sim.yaml',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo /clock for the base adapter.',
    )

    base_sim_adapter = Node(
        package='robocept_base',
        executable='base_sim_adapter',
        name='base_sim_adapter',
        namespace='robocept',
        parameters=[
            config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        base_sim_adapter,
    ])
