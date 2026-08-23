"""Launch IMU-driven Gazebo simulation with real Pixhawk orientation.

Stack:
  1. MAVROS (real Pixhawk 6C over /dev/ttyACM0)
  2. Gazebo Harmonic (drone model in sim world)
  3. imu_sim_bridge (real IMU -> sim drone pose/TF)
  4. RViz2 (3D visualization)

Usage:
  ros2 launch maritime_autonomy sim_imu.launch.py
  ros2 launch maritime_autonomy sim_imu.launch.py fcu_url:=/dev/ttyACM0:921600
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('maritime_autonomy')
    cfg = os.path.join(pkg, 'config', 'patrol_params.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'sim.rviz')
    world = os.path.join(pkg, 'worlds', 'chattogram.sdf')
    model = os.path.join(pkg, 'models', 'drone', 'model.sdf')

    fcu_url = LaunchConfiguration('fcu_url')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:921600',
                              description='MAVROS FCU URL'),
        Node(
            package='mavros', executable='mavros_node', name='mavros',
            output='screen', parameters=[{'fcu_url': fcu_url}],
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', world, '-r'],
            output='screen',
        ),
        Node(
            package='maritime_autonomy', executable='imu_sim_bridge',
            name='imu_sim_bridge', output='screen', parameters=[cfg],
        ),
        Node(
            package='maritime_autonomy', executable='gazebo_pose_driver',
            name='gazebo_pose_driver', output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_cfg], output='screen',
        ),
    ])
