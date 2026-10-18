"""Gazebo camera -> ROS 2 bridge.

The Humble `ros_gz_bridge` binaries are built against Gazebo Fortress and cannot
talk to Gazebo Harmonic, so this node subscribes to the camera on gz-transport
directly (python bindings) and republishes it as sensor_msgs/Image + CameraInfo.

Frames are stamped with the ROS clock at arrival: every downstream consumer
(tracker, controller) works on that one clock, which keeps latency compensation
consistent with MAVROS pose stamps.
"""
import array
import os

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import gz.transport13 as gzt  # noqa: E402
import rclpy  # noqa: E402
from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo  # noqa: E402
from gz.msgs10.image_pb2 import Image as GzImage  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402


class GzCameraBridge(Node):
    def __init__(self):
        super().__init__('gz_camera_bridge')
        self.declare_parameter('gz_image_topic', '/tracker/camera/image')
        self.declare_parameter('gz_info_topic', '/tracker/camera/camera_info')
        self.declare_parameter('frame_id', 'camera_optical')
        gz_image = self.get_parameter('gz_image_topic').value
        gz_info = self.get_parameter('gz_info_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_image = self.create_publisher(Image, 'camera/image_raw', 2)
        self.pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', 2)
        self.info = None

        self.gz = gzt.Node()
        self.gz.subscribe(GzImage, gz_image, self.on_image)
        self.gz.subscribe(GzCameraInfo, gz_info, self.on_info)
        self.topics = (gz_image, gz_info)
        self.get_logger().info(f'bridging gz {gz_image} -> {self.pub_image.topic_name}')

    def on_info(self, m):
        if self.info is not None:
            return
        info = CameraInfo()
        info.width, info.height = m.width, m.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = list(m.intrinsics.k) if len(m.intrinsics.k) == 9 else [0.0] * 9
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = list(m.projection.p) if len(m.projection.p) == 12 else \
            [info.k[0], 0.0, info.k[2], 0.0, 0.0, info.k[4], info.k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info = info

    def on_image(self, m):
        if m.width == 0 or m.height == 0:
            return
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.width, img.height = m.width, m.height
        channels = len(m.data) // (m.width * m.height)
        img.encoding = {1: 'mono8', 3: 'rgb8', 4: 'rgba8'}.get(channels, 'rgb8')
        img.step = m.width * channels
        img.data = array.array('B', m.data)    # assigning bytes directly costs ~150 ms/frame in rclpy
        self.pub_image.publish(img)
        if self.info is not None:
            self.info.header.stamp = stamp
            self.info.header.frame_id = self.frame_id
            self.pub_info.publish(self.info)

    def close(self):
        for t in self.topics:
            self.gz.unsubscribe(t)


def main():
    rclpy.init()
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()            # gz callbacks must stop before the interpreter tears down
        node.destroy_node()
        rclpy.try_shutdown()
