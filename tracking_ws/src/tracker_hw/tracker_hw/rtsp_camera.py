"""RTSP camera -> ROS 2 (camera/image_raw + camera/camera_info), in place of the Gazebo camera bridge.

ffmpeg decodes the stream (H.264 from the drone, relayed by MediaMTX) into raw BGR frames on a
pipe; a reader thread publishes each one as it arrives, so the tracker always sees the newest
frame. If the stream stalls or ffmpeg exits, it is restarted.

Frames are stamped with the ROS clock at arrival minus `latency` (glass-to-pipe delay of the
video link), so the follow controller geolocates each detection with the pose at capture time.
CameraInfo is a pinhole model from the horizontal field of view.
"""
import array
import math
import subprocess
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RtspCamera(Node):
    def __init__(self):
        super().__init__('rtsp_camera')
        p = self.declare_parameters('', [
            ('url', 'rtsp://127.0.0.1:8554/cam_web'),
            ('width', 1280), ('height', 720),       # frames are scaled to this size
            ('hfov_deg', 81.0),
            ('latency', 0.25),                      # [s] capture -> arrival
            ('stall_timeout', 3.0),                 # [s] no frame -> restart ffmpeg
            ('frame_id', 'camera_optical'),
        ])
        self.prm = {x.name: x.value for x in p}
        self.w, self.h = self.prm['width'], self.prm['height']
        self.frame_bytes = self.w * self.h * 3
        self.latency = Duration(seconds=self.prm['latency'])

        self.pub_image = self.create_publisher(Image, 'camera/image_raw', 2)
        self.pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', 2)
        self.info = self._camera_info()

        self.proc = None
        self.last_frame = time.monotonic()
        self.frames = 0
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.create_timer(1.0, self._watchdog)
        self.create_timer(10.0, self._report)
        self.get_logger().info(f'reading {self.prm["url"]} -> {self.pub_image.topic_name} ({self.w}x{self.h})')

    def _camera_info(self):
        fx = (self.w / 2) / math.tan(math.radians(self.prm['hfov_deg']) / 2)
        cx, cy = self.w / 2, self.h / 2
        info = CameraInfo()
        info.width, info.height = self.w, self.h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.header.frame_id = self.prm['frame_id']
        return info

    def _ffmpeg(self):
        url = self.prm['url']
        cmd = ['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
               '-fflags', 'nobuffer', '-flags', 'low_delay']
        if url.startswith('rtsp'):
            cmd += ['-rtsp_transport', 'tcp', '-timeout', '5000000']
        cmd += ['-i', url, '-an', '-vf', f'scale={self.w}:{self.h}',
                '-pix_fmt', 'bgr24', '-f', 'rawvideo', 'pipe:1']
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)

    def _run(self):
        while self.running:
            self.proc = self._ffmpeg()
            self.last_frame = time.monotonic()
            out = self.proc.stdout
            buf = bytearray(self.frame_bytes)
            view = memoryview(buf)
            while self.running:
                n = 0
                while n < self.frame_bytes:
                    got = out.readinto(view[n:])
                    if not got:
                        break
                    n += got
                if n < self.frame_bytes:
                    break                           # ffmpeg exited / was killed
                self._publish(buf)
            self.proc.kill()
            self.proc.wait()
            if self.running:
                self.get_logger().warn('stream ended, reconnecting in 1 s')
                time.sleep(1.0)

    def _publish(self, buf):
        self.last_frame = time.monotonic()
        self.frames += 1
        stamp = (self.get_clock().now() - self.latency).to_msg()
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.prm['frame_id']
        img.width, img.height = self.w, self.h
        img.encoding, img.step = 'bgr8', self.w * 3
        img.data = array.array('B', buf)        # assigning bytes directly is very slow in rclpy
        self.pub_image.publish(img)
        self.info.header.stamp = stamp
        self.pub_info.publish(self.info)

    def _watchdog(self):
        if self.proc is not None and time.monotonic() - self.last_frame > self.prm['stall_timeout']:
            self.get_logger().warn(f'no frame for {self.prm["stall_timeout"]:.0f} s, restarting the stream')
            self.last_frame = time.monotonic()
            self.proc.kill()

    def _report(self):
        self.get_logger().info(f'{self.frames / 10.0:.1f} fps')
        self.frames = 0

    def close(self):
        self.running = False
        if self.proc is not None:
            self.proc.kill()


def main():
    rclpy.init()
    node = RtspCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
