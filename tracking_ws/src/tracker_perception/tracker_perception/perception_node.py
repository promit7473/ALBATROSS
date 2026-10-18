"""Perception node: detect -> track -> target lock, on the drone camera stream.

Subscribes  camera/image_raw            sensor_msgs/Image
Publishes   tracking/tracks             tracker_interfaces/TrackArray
            tracking/target             tracker_interfaces/TargetState
            tracking/image_annotated    sensor_msgs/Image (boxes, ids, lock)
Services    tracking/lock               tracker_interfaces/LockTarget

Pipeline per frame: YOLO detection -> camera-motion estimate (optical flow) ->
ByteTrack-style association -> target lock / re-identification.
Inference runs in a worker thread on the *latest* frame only: when the
detector is slower than the camera, stale frames are dropped instead of
queueing up latency.
"""
import array
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tracker_interfaces.msg import TargetState, Track, TrackArray
from tracker_interfaces.srv import LockTarget

from tracker_perception.byte_tracker import ByteTracker
from tracker_perception.camera_motion import CameraMotion
from tracker_perception.detector import YoloDetector
from tracker_perception.target_lock import TargetLock

CLASS_COLOURS = {'person': (60, 220, 60), 'car': (255, 160, 40), 'truck': (255, 100, 0),
                 'bus': (255, 60, 160), 'motorcycle': (0, 200, 255), 'bicycle': (0, 255, 255)}
LOCK_COLOUR = (0, 0, 255)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        p = self.declare_parameters('', [
            ('weights', ''),
            ('classes', ['person', 'car', 'truck', 'bus', 'motorcycle', 'bicycle']),
            ('lockable_classes', ['person']),
            ('imgsz', 640),
            ('device', 'auto'),
            ('threads', 4),
            ('det_conf', 0.1),
            ('high_thresh', 0.35),
            ('new_track_thresh', 0.40),
            ('min_hits', 2),
            ('max_lost_frames', 45),
            ('reid_max_distance', 0.35),
            ('reid_max_lost_time', 20.0),
            ('annotated_max_hz', 15.0),
        ])
        prm = {x.name: x.value for x in p}
        weights = prm['weights'] or os.path.join(os.environ.get('TRACKING_WS', ''), 'assets/weights/yolo11s.pt')
        self.detector = YoloDetector(weights, classes=prm['classes'], conf=prm['det_conf'],
                                     imgsz=prm['imgsz'], device=prm['device'], threads=prm['threads'])
        self.get_logger().info(f'detector {weights} on {self.detector.device}, imgsz {prm["imgsz"]}')
        self.names = self.detector.names
        self.lockable = set(prm['lockable_classes'])
        self.tracker = ByteTracker(high_thresh=prm['high_thresh'], new_track_thresh=prm['new_track_thresh'],
                                   min_hits=prm['min_hits'],
                                   max_lost=prm['max_lost_frames'])
        self.gmc = CameraMotion()
        self.lock = TargetLock(max_app_dist=prm['reid_max_distance'], max_lost_time=prm['reid_max_lost_time'])
        self.annotated_period = 1.0 / prm['annotated_max_hz']

        self.pub_tracks = self.create_publisher(TrackArray, 'tracking/tracks', 5)
        self.pub_target = self.create_publisher(TargetState, 'tracking/target', 5)
        self.pub_annot = self.create_publisher(Image, 'tracking/image_annotated', 2)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, 2)
        self.create_service(LockTarget, 'tracking/lock', self.on_lock)

        self._frame_lock = threading.Lock()
        self._latest = None
        self._state_lock = threading.Lock()     # guards tracker / lock between worker and service
        self._tracks, self._frame = [], None
        self._last_annot = 0.0
        self._fps = 0.0
        self._running = True
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    # ------------------------------------------------------------------ ROS callbacks

    def on_image(self, msg):
        with self._frame_lock:
            self._latest = msg

    def on_lock(self, req, res):
        with self._state_lock:
            if req.track_id < 0:
                self.lock.release()
                res.success, res.message = True, 'lock released'
                return res
            t = next((t for t in self._tracks if t.id == req.track_id), None)
            if t is None:
                res.success, res.message = False, f'track {req.track_id} not found'
            elif self.names[t.cls] not in self.lockable:
                res.success, res.message = False, f'class {self.names[t.cls]} is not lockable'
            else:
                self.lock.lock(t, self._frame, time.monotonic())
                res.success, res.message = True, f'locked on track {t.id}'
        self.get_logger().info(res.message)
        return res

    # ------------------------------------------------------------------ worker

    def _loop(self):
        while self._running and rclpy.ok():
            with self._frame_lock:
                msg, self._latest = self._latest, None
            if msg is None:
                time.sleep(0.005)
                continue
            t0 = time.monotonic()
            frame = self._to_bgr(msg)
            dets = self.detector(frame)
            motion = self.gmc.estimate(frame, dets.boxes)
            with self._state_lock:
                tracks = self.tracker.update(dets, motion)
                target = self.lock.update(tracks, frame, time.monotonic())
                self._tracks, self._frame = tracks, frame
                status, lost_for, locked_id = self.lock.status, self.lock.lost_for, self.lock.track_id
                last_box = self.lock.last_box
                if status == 2 and self.lock.debug:
                    self.get_logger().info(f're-id candidates: {self.lock.debug}', throttle_duration_sec=1.0)
            dt = time.monotonic() - t0
            self._fps = 0.9 * self._fps + 0.1 / max(dt, 1e-3) if self._fps else 1.0 / max(dt, 1e-3)
            self._publish(msg, frame, tracks, target, status, lost_for, locked_id, last_box)

    @staticmethod
    def _to_bgr(msg):
        ch = len(msg.data) // (msg.width * msg.height)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, ch)
        if msg.encoding.startswith('rgb'):
            return cv2.cvtColor(img[..., :3], cv2.COLOR_RGB2BGR)
        if ch == 1:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return np.ascontiguousarray(img[..., :3])

    def _track_msg(self, t):
        m = Track()
        m.id, m.class_name, m.score = t.id, self.names[t.cls], t.score
        m.x1, m.y1, m.x2, m.y2 = (float(v) for v in t.box)
        m.state = t.state
        return m

    def _publish(self, msg, frame, tracks, target, status, lost_for, locked_id, last_box):
        ta = TrackArray()
        ta.header = msg.header
        ta.image_width, ta.image_height = msg.width, msg.height
        ta.tracks = [self._track_msg(t) for t in tracks]
        self.pub_tracks.publish(ta)

        ts = TargetState()
        ts.header = msg.header
        ts.status = status
        ts.track_id = locked_id
        ts.lost_for = float(lost_for)
        ts.image_width, ts.image_height = msg.width, msg.height
        if target is not None:
            ts.track = self._track_msg(target)
        elif last_box is not None:
            ts.track.x1, ts.track.y1, ts.track.x2, ts.track.y2 = (float(v) for v in last_box)
        self.pub_target.publish(ts)

        now = time.monotonic()
        if self.pub_annot.get_subscription_count() and now - self._last_annot >= self.annotated_period:
            self._last_annot = now
            self.pub_annot.publish(self._annotate(msg, frame, tracks, status, locked_id, last_box))

    def _annotate(self, msg, frame, tracks, status, locked_id, last_box):
        img = frame.copy()
        for t in tracks:
            x1, y1, x2, y2 = (int(v) for v in t.box)
            name = self.names[t.cls]
            locked = t.id == locked_id and status == 1
            col = LOCK_COLOUR if locked else CLASS_COLOURS.get(name, (200, 200, 200))
            thick = 3 if locked else 1
            if t.state == 2:                      # coasting: dashed look via thin grey
                col, thick = (150, 150, 150), 1
            cv2.rectangle(img, (x1, y1), (x2, y2), col, thick)
            label = f'{"LOCK " if locked else ""}{name} {t.id}'
            cv2.putText(img, label, (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        if status == 2 and last_box is not None:
            x1, y1, x2, y2 = (int(v) for v in last_box)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 165, 255), 1)
            cv2.putText(img, 'SEARCHING', (x1, y2 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
        cv2.putText(img, f'det+track {self._fps:4.1f} Hz', (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        out = Image()
        out.header = msg.header
        out.height, out.width = img.shape[:2]
        out.encoding, out.step = 'bgr8', img.shape[1] * 3
        out.data = array.array('B', img.tobytes())    # bytes assignment is very slow in rclpy
        return out

    def destroy_node(self):
        self._running = False
        self._worker.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
