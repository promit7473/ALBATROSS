"""Operator console (OpenCV window).

  left click on a person   lock / track that person
  right click              release the lock
  t / l / h                takeoff / land / hover here (release lock, stop following)
  w / a / s / d            (hovering) move 5 m forward / left / back / right
  q / e                    (hovering) yaw 30 deg left / right
  m                        cycle follow mode: behind -> hold_bearing -> orbit
  + / -                    follow distance +/- 2 m        0   distance back to auto
  r / f                    altitude +/- 2 m
  Esc                      quit the console (the drone keeps its current behaviour)
"""
import math
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tracker_interfaces.msg import FollowStatus, TargetState, TrackArray
from tracker_interfaces.srv import Command, LockTarget

WINDOW = 'Drone Tracker'
MODES = ['behind', 'hold_bearing', 'orbit']
MOVES = {'w': '5 0', 's': '-5 0', 'a': '0 5', 'd': '0 -5'}      # forward left [m]
MAP_SIZE, MAP_RANGE = 180, 40.0          # mini-map: pixels, metres from centre to edge
STATUS_NAMES = {TargetState.NONE: 'none', TargetState.TRACKING: 'TRACKING', TargetState.LOST: 'LOST'}


class Console(Node):
    def __init__(self):
        super().__init__('operator_console')
        self.lock = threading.Lock()
        self.frame, self.tracks, self.status, self.target = None, None, None, None
        self.create_subscription(Image, 'tracking/image_annotated', self.on_image, 2)
        self.create_subscription(TrackArray, 'tracking/tracks', self.on_tracks, 5)
        self.create_subscription(FollowStatus, 'follow/status', self.on_status, 5)
        self.create_subscription(TargetState, 'tracking/target', self.on_target, 5)
        self.lock_cli = self.create_client(LockTarget, 'tracking/lock')
        self.cmd_cli = self.create_client(Command, 'follow/command')
        self.message = 'click a person to track'

    def on_image(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[..., :3]
        if m.encoding.startswith('rgb'):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.frame = img.copy()

    def on_tracks(self, m):
        self.tracks = m

    def on_status(self, m):
        self.status = m

    def on_target(self, m):
        self.target = m

    # ------------------------------------------------------------------ actions

    def _done(self, future):
        try:
            r = future.result()
            self.message = r.message
        except Exception as e:  # noqa: BLE001 - surface any service failure to the operator
            self.message = f'error: {e}'

    def lock_at(self, x, y):
        tr = self.tracks
        if tr is None:
            return
        hits = [t for t in tr.tracks if t.x1 <= x <= t.x2 and t.y1 <= y <= t.y2]
        if not hits:
            self.message = 'nothing under the cursor'
            return
        t = min(hits, key=lambda t: (t.x2 - t.x1) * (t.y2 - t.y1))      # innermost box
        self.call_lock(t.id)

    def call_lock(self, track_id):
        if not self.lock_cli.service_is_ready():
            self.message = 'perception not running'
            return
        self.lock_cli.call_async(LockTarget.Request(track_id=track_id)).add_done_callback(self._done)

    def command(self, cmd, value=''):
        if not self.cmd_cli.service_is_ready():
            self.message = 'follow controller not running'
            return
        self.cmd_cli.call_async(Command.Request(command=cmd, value=str(value))).add_done_callback(self._done)

    # ------------------------------------------------------------------ drawing

    def render(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            frame = np.full((480, 640, 3), 30, np.uint8)
            cv2.putText(frame, 'waiting for camera...', (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (200, 200, 200), 1, cv2.LINE_AA)
        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 14, 1)
        self._minimap(frame)
        panel = np.full((70, w, 3), 25, np.uint8)
        s, t = self.status, self.target
        line1 = 'no flight controller'
        if s is not None:
            line1 = (f'{s.flight_state:<9} {"ARMED" if s.armed else "disarmed"} {"OFFB" if s.offboard else ""}'
                     f'  alt {s.drone_position.z:5.1f}/{s.desired_altitude:.0f} m'
                     f'  dist {s.target_distance:5.1f}/{s.desired_distance:.1f} m  mode {s.follow_mode}')
        line2 = f'target: {STATUS_NAMES.get(t.status, "?") if t else "none"}'
        if t is not None and t.status != TargetState.NONE:
            line2 += f' id {t.track_id}'
            if t.status == TargetState.LOST:
                line2 += f' lost {t.lost_for:4.1f}s'
        if s is not None and s.target_valid:
            line2 += f'  speed {math.hypot(s.target_velocity.x, s.target_velocity.y):3.1f} m/s'
        line2 += f'   | {self.message}'
        cv2.putText(panel, line1, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(panel, line2, (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(panel, 'click=lock rclick=unlock t/l/h=takeoff/land/hover wasd/qe=move/yaw m=mode +/-/0=dist r/f=alt Esc=quit',
                    (8, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (170, 170, 170), 1, cv2.LINE_AA)
        return np.vstack([frame, panel])

    def _minimap(self, frame):
        """Top-down view centred on the drone, north up: drone heading + target estimate."""
        s = self.status
        if s is None:
            return
        m = np.full((MAP_SIZE, MAP_SIZE, 3), 20, np.uint8)
        c = MAP_SIZE // 2
        k = c / MAP_RANGE
        for r in (10, 20, 30, 40):
            cv2.circle(m, (c, c), int(r * k), (60, 60, 60), 1)
        yaw = s.drone_yaw
        tip = (int(c + 12 * math.cos(yaw)), int(c - 12 * math.sin(yaw)))
        cv2.arrowedLine(m, (c, c), tip, (255, 255, 255), 2, tipLength=0.4)
        if s.target_valid:
            dx = s.target_position.x - s.drone_position.x
            dy = s.target_position.y - s.drone_position.y
            pt = (int(c + dx * k), int(c - dy * k))
            cv2.circle(m, pt, 5, (0, 0, 255), -1)
            v = (int(pt[0] + 3 * s.target_velocity.x * k), int(pt[1] - 3 * s.target_velocity.y * k))
            cv2.line(m, pt, v, (0, 0, 255), 1)
        cv2.putText(m, 'N', (c - 4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        h, w = frame.shape[:2]
        frame[8:8 + MAP_SIZE, w - MAP_SIZE - 8:w - 8] = cv2.addWeighted(
            frame[8:8 + MAP_SIZE, w - MAP_SIZE - 8:w - 8], 0.3, m, 0.7, 0)

    def handle_key(self, key):
        s = self.status
        if key == ord('t'):
            self.command('takeoff')
        elif key == ord('l'):
            self.command('land')
        elif key == ord('h'):
            self.call_lock(-1)
            self.command('hover')
        elif key == ord('m') and s is not None:
            self.command('follow_mode', MODES[(MODES.index(s.follow_mode) + 1) % len(MODES)])
        elif key in (ord('+'), ord('=')) and s is not None:
            self.command('distance', f'{s.desired_distance + 2:.1f}')
        elif key == ord('-') and s is not None:
            self.command('distance', f'{max(3.0, s.desired_distance - 2):.1f}')
        elif key == ord('0'):
            self.command('distance', 'auto')
        elif key == ord('r') and s is not None:
            self.command('altitude', f'{s.desired_altitude + 2:.1f}')
        elif key == ord('f') and s is not None:
            self.command('altitude', f'{s.desired_altitude - 2:.1f}')
        elif chr(key) in MOVES:
            self.command('move', MOVES[chr(key)])
        elif key == ord('q'):
            self.command('yaw', '30')
        elif key == ord('e'):
            self.command('yaw', '-30')


def main():
    rclpy.init()
    node = Console()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            node.lock_at(x, y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            node.call_lock(-1)

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse)
    try:
        while rclpy.ok():
            cv2.imshow(WINDOW, node.render())
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key != 255:
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
