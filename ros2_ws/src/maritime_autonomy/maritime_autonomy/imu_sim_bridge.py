#!/usr/bin/env python3
"""Bridges real Pixhawk IMU orientation into the Gazebo simulation.

Single efficient node that:
  - Reads /mavros/imu/data at native rate
  - Publishes TF + /drone/pose + /drone/imu at ~5Hz
  - Pushes pose to Gazebo via set_pose service (gravity-free model)
"""
import math
import os

os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation as R

from gz.transport13 import Node as GzNode
from gz.msgs10 import pose_pb2, boolean_pb2


class IMUSimBridge(Node):
    def __init__(self):
        super().__init__('imu_sim_bridge')
        self.declare_parameter('hover_height', 2.0)

        self.hover_h = float(self.get_parameter('hover_height').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 5)
        self.imu_pub = self.create_publisher(Imu, '/drone/imu', 5)

        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Imu, '/mavros/imu/data', self._imu_cb, sensor_qos)

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.yaw_from_gyro = 0.0
        self.last_time = None

        self._pub_count = 0
        self._publish_every = 20

        self._gz_node = GzNode()
        self._gz_pose_msg = pose_pb2.Pose()

        self.get_logger().info(
            f'IMU->Sim bridge active  hover={self.hover_h}m  '
            f'gz set_pose service @~5Hz')

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        now = self.get_clock().now()

        if self.last_time is not None:
            dt = min((now - self.last_time).nanoseconds * 1e-9, 0.1)
            self.yaw_from_gyro += msg.angular_velocity.z * dt

            roll, pitch, _ = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            speed = 0.0  # pure orientation mirror: tilt board -> drone tilts in place
            tilt_fwd = -pitch
            tilt_right = roll
            self.drone_x += (tilt_fwd * math.cos(self.yaw_from_gyro)
                             - tilt_right * math.sin(self.yaw_from_gyro)) * speed * dt
            self.drone_y += (tilt_fwd * math.sin(self.yaw_from_gyro)
                             + tilt_right * math.cos(self.yaw_from_gyro)) * speed * dt

        self.last_time = now
        self._pub_count += 1

        if self._pub_count % self._publish_every != 0:
            return

        dz = self.hover_h
        if self.last_time is not None:
            dz = self.hover_h

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'drone_base_link'
        t.transform.translation = Vector3(x=self.drone_x, y=self.drone_y, z=dz)
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = self.drone_x
        ps.pose.position.y = self.drone_y
        ps.pose.position.z = dz
        ps.pose.orientation = q
        self.pose_pub.publish(ps)
        self.imu_pub.publish(msg)

        try:
            self._gz_pose_msg.header.stamp.sec = now.nanoseconds // 10**9
            self._gz_pose_msg.header.stamp.nsec = now.nanoseconds % 10**9
            self._gz_pose_msg.name = 'drone'
            self._gz_pose_msg.position.x = self.drone_x
            self._gz_pose_msg.position.y = self.drone_y
            self._gz_pose_msg.position.z = dz
            self._gz_pose_msg.orientation.x = q.x
            self._gz_pose_msg.orientation.y = q.y
            self._gz_pose_msg.orientation.z = q.z
            self._gz_pose_msg.orientation.w = q.w
            self._gz_node.request(
                '/world/chattogram_sim/set_pose',
                self._gz_pose_msg,
                pose_pb2.Pose,
                boolean_pb2.Boolean,
                50)
        except Exception:
            pass


def main():
    rclpy.init()
    node = IMUSimBridge()
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
