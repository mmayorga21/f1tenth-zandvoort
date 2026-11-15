"""Launch only the controller, disabled; the simulator runs separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('f1tenth_controller'),
                          'config', 'follow_the_gap.yaml')
    return LaunchDescription([Node(package='f1tenth_controller', executable='gap_follower',
                                    name='gap_follower', parameters=[config], output='screen')])
