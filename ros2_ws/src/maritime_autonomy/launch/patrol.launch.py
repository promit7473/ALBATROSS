"""Launch MAVROS + the maritime patrol skeleton against PX4 SITL.

Usage:
  ros2 launch maritime_autonomy patrol.launch.py
  ros2 launch maritime_autonomy patrol.launch.py fcu_url:=udp://:14540@127.0.0.1:14580
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('maritime_autonomy'), 'config', 'patrol_params.yaml')
    fcu_url = LaunchConfiguration('fcu_url')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14580',
                              description='MAVROS FCU URL (SITL onboard link by default)'),
        # NOTE: do NOT set name='mavros' here. mavros_node internally spawns two
        # nodes (mavros_router + mavros/uas); a __node remap collapses both onto
        # the same name and it dies with "invalid allocator" on /mavros/mavros/status.
        # Leaving the default naming keeps topics under /mavros/* as the nodes expect.
        Node(
            package='mavros', executable='mavros_node', output='screen',
            parameters=[{'fcu_url': fcu_url}],
        ),
        Node(
            package='maritime_autonomy', executable='patrol_node', name='maritime_patrol',
            output='screen', parameters=[cfg],
        ),
    ])
