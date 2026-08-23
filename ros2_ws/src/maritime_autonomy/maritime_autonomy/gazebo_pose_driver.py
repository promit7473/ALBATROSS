#!/usr/bin/env python3
"""Moves the Gazebo drone entity to match the IMU-driven pose.

Subscribes to /drone/pose and calls gz service to reposition the drone
model in the sim world. Throttled to ~10 Hz to avoid overloading.
"""
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GazeboPoseDriver(Node):
    def __init__(self):
        super().__init__('gazebo_pose_driver')
        self._last_sec = 0
        self.create_subscription(PoseStamped, '/drone/pose', self._pose_cb, 10)
        self.get_logger().info('Gazebo pose driver active - moving drone entity')

    def _pose_cb(self, msg: PoseStamped):
        now_sec = msg.header.stamp.sec
        if now_sec == self._last_sec:
            return
        self._last_sec = now_sec

        p = msg.pose.position
        q = msg.pose.orientation
        cmd = [
            'gz', 'service', '-s', '/world/chattogram_sim/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req',
            f"name: 'drone', position: {{x: {p.x}, y: {p.y}, z: {p.z}}}, "
            f"orientation: {{x: {q.x}, y: {q.y}, z: {q.z}, w: {q.w}}}"
        ]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    rclpy.init()
    node = GazeboPoseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
