"""Full system: simulation + MAVROS + perception + follow controller + operator console.

  ros2 launch tracker_bringup tracking.launch.py [gui:=false] [console:=false] [weights:=...]
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FCU_URL = 'udp://:14541@127.0.0.1:14581'    # PX4 SITL instance 1, offboard link
FCU_SYSTEM_ID = 2                            # MAV_SYS_ID = instance + 1


def launch_setup(context):
    bringup = Path(get_package_share_directory('tracker_bringup'))
    ws = Path(os.environ.get('TRACKING_WS') or bringup.parents[3])
    params = str(bringup / 'config' / 'tracking.yaml')
    weights = LaunchConfiguration('weights').perform(context) or str(ws / 'assets' / 'weights' / 'yolo11s.pt')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('tracker_sim')) / 'launch' / 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'px4_dir': LaunchConfiguration('px4_dir')}.items())
    mavros = Node(package='mavros', executable='mavros_node', name='mavros', output='log',
                  arguments=['--ros-args', '--log-level', 'mavros:=warn'],
                  parameters=[{'fcu_url': FCU_URL, 'gcs_url': '', 'tgt_system': FCU_SYSTEM_ID,
                               'tgt_component': 1, 'fcu_protocol': 'v2.0',
                               'plugin_allowlist': ['sys_status', 'sys_time', 'command', 'param', 'imu',
                                                    'local_position', 'setpoint_raw', 'home_position']}])
    perception = Node(package='tracker_perception', executable='perception_node', name='perception',
                      output='screen', parameters=[params, {'weights': weights}])
    follow = Node(package='tracker_control', executable='follow_node', name='follow', output='screen',
                  parameters=[params])
    console = Node(package='tracker_ui', executable='operator_console', output='screen',
                   condition=IfCondition(LaunchConfiguration('console')))
    # console last, so its window opens on top of the Gazebo GUI
    return [sim, mavros, perception, follow, TimerAction(period=15.0, actions=[console])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('console', default_value='true', description='operator console window'),
        DeclareLaunchArgument('weights', default_value='', description='YOLO weights path'),
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', '~/PX4-Autopilot')),
        OpaqueFunction(function=launch_setup),
    ])
