"""Real drone: RTSP camera + Intelis bridge link + perception + follow controller + operator console.

  GCS_TOKEN=<operator key> ros2 launch tracker_bringup real.launch.py [console:=false] [weights:=...]

The flight controller is reached through the Intelis telemetry bridge next to the drone
(tracker_hw/bridge_link stands in for MAVROS); there is no simulation and no MAVROS here.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    bringup = Path(get_package_share_directory('tracker_bringup'))
    ws = Path(os.environ.get('TRACKING_WS') or bringup.parents[3])
    params = [str(bringup / 'config' / 'tracking.yaml'), str(bringup / 'config' / 'real.yaml')]
    # yolo11l: on the lab RTX 5060 Ti ~19 ms/frame at 960 px, and it keeps partly hidden people
    # that yolo11s drops (benchmark on drone frames, 2026-10-25)
    weights = LaunchConfiguration('weights').perform(context) or str(ws / 'assets' / 'weights' / 'yolo11l.pt')
    venv = ws / '.venv' / 'bin' / 'python'
    prefix = [str(venv)] if venv.exists() else None      # torch / ultralytics live in the venv

    camera = Node(package='tracker_hw', executable='rtsp_camera', name='rtsp_camera', output='screen',
                  parameters=params)
    link = Node(package='tracker_hw', executable='bridge_link', name='bridge_link', output='screen',
                parameters=params)
    perception = Node(package='tracker_perception', executable='perception_node', name='perception',
                      prefix=prefix, output='screen', parameters=params + [{'weights': weights}])
    follow = Node(package='tracker_control', executable='follow_node', name='follow', output='screen',
                  parameters=params)
    console = Node(package='tracker_ui', executable='operator_console', prefix=prefix, output='screen',
                   condition=IfCondition(LaunchConfiguration('console')))
    return [camera, link, perception, follow, TimerAction(period=5.0, actions=[console])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('console', default_value='true', description='operator console window'),
        DeclareLaunchArgument('weights', default_value='', description='YOLO weights path'),
        OpaqueFunction(function=launch_setup),
    ])
