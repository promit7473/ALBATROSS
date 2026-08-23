#!/usr/bin/env python3
"""#13 Buoy drop - SAR cargo release at the target.

Extends the Task-2 descend profile: fly (Task-1 exclusion-aware routing) to the
target, descend to the action altitude and hold; the instant the drop (low-hold)
waypoint is reached it
  (a) fires a servo release via MAV_CMD_DO_SET_SERVO (live command over
      /mavros/cmd/command - opens a gripper/latch on real hardware), and
  (b) spawns a visual life-buoy in Gazebo at the drone's position that falls to
      the water,
then climbs back and RTLs. The release is a live command (not a mission item),
so PX4's mission-validity check stays happy.

Params (+ DescendNode params): gz_world (default 'default'), servo_channel.
"""
import subprocess

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandLong
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from .descend_node import DescendNode

DO_SET_SERVO = 183  # MAV_CMD_DO_SET_SERVO

BUOY_SDF = (
    '<?xml version="1.0"?>'
    '<sdf version="1.8"><model name="buoy"><link name="l">'
    '<inertial><mass>1.5</mass><inertia>'
    '<ixx>0.02</ixx><iyy>0.02</iyy><izz>0.03</izz>'
    '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
    '<collision name="c"><geometry><cylinder>'
    '<radius>0.4</radius><length>0.18</length></cylinder></geometry></collision>'
    '<visual name="v"><geometry><cylinder>'
    '<radius>0.4</radius><length>0.18</length></cylinder></geometry>'
    '<material><ambient>1 0.45 0 1</ambient><diffuse>1 0.45 0 1</diffuse></material>'
    '</visual></link></model></sdf>')

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST)


class DropNode(DescendNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('gz_world', 'default')
        self.declare_parameter('servo_channel', 1)
        self.gz_world = str(self.get_parameter('gz_world').value)
        self.servo_ch = int(self.get_parameter('servo_channel').value)
        self.local = None
        self._dropped = False
        self.drop_index = None
        self.cli_cmd = self.create_client(CommandLong, '/mavros/cmd/command')
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._local_cb, SENSOR_QOS)

    def _local_cb(self, msg):
        self.local = msg.pose.position

    def build_mission(self):
        wps = super().build_mission()   # [..., descend, hold(drop), climb]
        self.drop_index = len(wps) - 2  # the low-hold waypoint == drop point
        self.get_logger().info(
            f'buoy drop armed: release servo ch{self.servo_ch} when waypoint '
            f'{self.drop_index} (low-hold) is reached')
        return wps

    def _on_reached(self, seq):
        if not self._dropped and self.drop_index is not None and seq >= self.drop_index:
            self._dropped = True
            if self.range_m is not None:
                self.get_logger().info(f'Rangefinder surface clearance: {self.range_m:.2f} m')
                if self.range_m < self.min_alt - 1.0:
                    self.get_logger().warn(
                        f'SAFETY WARNING: Measured range ({self.range_m:.2f}m) below safety floor ({self.min_alt:.1f}m)!')
            else:
                self.get_logger().info('Rangefinder topic not received; proceeding with GPS/barometric altitude hold.')
            self._fire_servo()
            self._release_buoy()

    def _fire_servo(self):
        try:
            req = CommandLong.Request()
            req.command = DO_SET_SERVO
            req.param1 = float(self.servo_ch)
            req.param2 = 2000.0   # PWM -> release
            res = self._call(self.cli_cmd, req, 'cmd/command(DO_SET_SERVO)')
            self.get_logger().info(f'servo release cmd sent (success={getattr(res,"success",None)})')
        except Exception as e:
            self.get_logger().warn(f'servo cmd failed: {e}')

    def _release_buoy(self):
        if self.local is None:
            self.get_logger().warn('BUOY RELEASE: no local position yet')
            return
        x, y, z = self.local.x, self.local.y, max(self.local.z, 1.0)
        with open('/tmp/buoy.sdf', 'w') as f:
            f.write(BUOY_SDF)
        req = (f'sdf_filename: "/tmp/buoy.sdf", '
               f'pose: {{position: {{x: {x:.2f}, y: {y:.2f}, z: {z:.2f}}}}}, '
               f'name: "buoy", allow_renaming: true')
        cmd = ['gz', 'service', '-s', f'/world/{self.gz_world}/create',
               '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '3000', '--req', req]
        try:
            r = subprocess.run(cmd, capture_output=True, timeout=6, text=True)
            ok = 'true' in (r.stdout or '').lower()
            self.get_logger().info(
                f'>>> BUOY RELEASED at local ({x:.1f},{y:.1f},{z:.1f})  gz_spawn={ok} <<<')
        except Exception as e:
            self.get_logger().warn(f'buoy spawn failed: {e}')


def main():
    rclpy.init()
    node = DropNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
