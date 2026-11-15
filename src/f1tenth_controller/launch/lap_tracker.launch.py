"""Launch lap statistics; use ros2 run in a terminal for the live panel."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('f1tenth_controller'),
                          'config', 'lap_tracker.yaml')
    return LaunchDescription([Node(package='f1tenth_controller', executable='lap_tracker',
                                    name='lap_tracker', parameters=[config], output='screen')])
