"""Simulation: Gazebo Harmonic city world + PX4 SITL (attached to the drone in the world)
+ camera bridge + moving traffic.

Args
  px4_dir   PX4-Autopilot checkout with a built px4_sitl_default (default: $PX4_DIR or ~/PX4-Autopilot)
  gui       start the Gazebo GUI (default true)
  follow_cam  make the GUI camera chase the drone (default true)
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PX4_INSTANCE = '1'          # instance 1: own rootfs/params/ports, never clashes with a stock SITL (0)
MODEL_NAME = 'drone'


def workspace_root():
    if os.environ.get('TRACKING_WS'):
        return Path(os.environ['TRACKING_WS'])
    # <ws>/install/tracker_sim/share/tracker_sim
    return Path(get_package_share_directory('tracker_sim')).parents[3]


def launch_setup(context):
    share = Path(get_package_share_directory('tracker_sim'))
    ws = workspace_root()
    px4 = Path(os.path.expanduser(LaunchConfiguration('px4_dir').perform(context)))
    world = share / 'worlds' / 'city.sdf'

    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join(filter(None, [
        str(ws / 'assets' / 'models'), str(share / 'models'),
        str(px4 / 'Tools' / 'simulation' / 'gz' / 'models'), env.get('GZ_SIM_RESOURCE_PATH', '')]))
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(filter(None, [
        str(px4 / 'build' / 'px4_sitl_default' / 'src' / 'modules' / 'simulation' / 'gz_plugins'),
        env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))
    env['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

    if not (ws / 'assets' / 'models' / 'people').is_dir():
        raise RuntimeError(f'assets missing in {ws}/assets -- run scripts/setup_assets.sh first')
    px4_bin = px4 / 'build' / 'px4_sitl_default' / 'bin' / 'px4'
    if not px4_bin.exists():
        raise RuntimeError(f'{px4_bin} not found -- build PX4 with "make px4_sitl" or set px4_dir:=')

    gz_server = ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', str(world)], env=env, output='screen',
                               name='gz_server')
    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], env={**env, 'QT_QPA_PLATFORM': 'xcb'},
                            output='log', name='gz_gui', condition=IfCondition(LaunchConfiguration('gui')))
    px4_env = {**env, 'PX4_SYS_AUTOSTART': '4001', 'PX4_GZ_STANDALONE': '1',
               'PX4_GZ_MODEL_NAME': MODEL_NAME, 'PX4_GZ_WORLD': 'city'}
    px4_proc = ExecuteProcess(cmd=[str(px4_bin), '-i', PX4_INSTANCE, '-d'], cwd=str(px4), env=px4_env,
                              output='log', name='px4')
    follow_cam = ExecuteProcess(
        cmd=['bash', '-c', 'for i in $(seq 20); do gz topic -t /gui/track -m gz.msgs.CameraTrack -p '
             f'\'track_mode: FOLLOW, follow_target: {{name: "{MODEL_NAME}"}}, '
             'follow_offset: {x: -8, y: 0, z: 5}, follow_pgain: 0.05\'; sleep 1; done'],
        env=env, output='log', condition=IfCondition(LaunchConfiguration('follow_cam')))
    bridge = Node(package='tracker_sim', executable='gz_camera_bridge', output='screen', additional_env=env)
    traffic = Node(package='tracker_sim', executable='traffic_node', output='screen', additional_env=env)

    return [gz_server, gz_gui, bridge, traffic,
            TimerAction(period=6.0, actions=[px4_proc]),
            TimerAction(period=15.0, actions=[follow_cam])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', '~/PX4-Autopilot')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('follow_cam', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
