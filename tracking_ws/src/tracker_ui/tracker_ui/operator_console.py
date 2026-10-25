"""Operator console (OpenCV window): live annotated camera, buttons, click-to-track.

Everything can be done with the mouse:
  * buttons under the video: TAKE OFF, LAND, HOVER, move / yaw, distance, altitude,
    follow mode, UNLOCK
  * left click on (or near) a person in the video   -> lock and follow that person
  * right click in the video                         -> release the lock

Keyboard shortcuts (the window must have focus):
  t / l / h   takeoff / land / hover (release lock)
  w a s d     move 5 m forward / left / back / right (while hovering)
  q / e       yaw 30 deg left / right (while hovering)
  m           cycle follow mode      + / - / 0   distance +2 / -2 / auto
  r / f       altitude +2 / -2       z            full screen on / off
  Esc         close the console

The window can be resized (aspect ratio kept); it starts at a size that fits the screen.
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
VIDEO_W = 1280                               # video is drawn this wide, whatever the camera resolution
FIT_H = 900                                  # start no taller than this (1080p screen minus bars / dock)
MODES = ['behind', 'hold_bearing', 'orbit']
MOVES = {'w': '5 0', 's': '-5 0', 'a': '0 5', 'd': '0 -5'}      # forward left [m]
MAP_SIZE, MAP_RANGE = 200, 40.0             # mini-map: pixels, metres from centre to edge
PICK_RADIUS = 40                             # [video px] a click this close to a box selects it
STATUS_NAMES = {TargetState.NONE: 'none', TargetState.TRACKING: 'TRACKING', TargetState.LOST: 'LOST - searching'}

BTN_H, BTN_GAP = 38, 6
INFO_H = 56
# (label, action) rows; action is a key handled by Console.handle_key
BUTTONS = [
    [('TAKE OFF', 't'), ('LAND', 'l'), ('HOVER', 'h'), ('MODE', 'm'),
     ('DIST -', '-'), ('DIST +', '+'), ('DIST AUTO', '0'), ('ALT -', 'f'), ('ALT +', 'r')],
    [('YAW LEFT', 'q'), ('FORWARD', 'w'), ('YAW RIGHT', 'e'), ('LEFT', 'a'), ('BACK', 's'),
     ('RIGHT', 'd'), ('FULL SCREEN', 'z')],
]


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
        self.message = 'press TAKE OFF, then click a person'
        self.buttons = []                  # (x1, y1, x2, y2, key) in window pixels, rebuilt each render
        self.scale = 1.0                   # window pixels per video pixel, set in render()
        self.video_h = 720
        self.fullscreen = False

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
            self.message = future.result().message
        except Exception as e:  # noqa: BLE001 - surface any service failure to the operator
            self.message = f'error: {e}'

    def click(self, x, y, right=False):
        """Mouse click in window pixels: a button, or a person in the video."""
        for x1, y1, x2, y2, key in self.buttons:
            if x1 <= x <= x2 and y1 <= y <= y2:
                self.handle_key(ord(key))
                return
        if y >= self.video_h:
            return
        if right:
            self.call_lock(-1)
            return
        self.lock_at(x / self.scale, y / self.scale)

    def lock_at(self, x, y):
        """Lock the person under (or nearest to) video pixel (x, y)."""
        tr = self.tracks
        people = [t for t in tr.tracks if t.class_name == 'person'] if tr else []
        if not people:
            self.message = 'no people detected yet - fly closer (FORWARD / YAW)'
            return

        def gap(t):                          # 0 inside the box, else distance to its edge
            return math.hypot(max(t.x1 - x, 0, x - t.x2), max(t.y1 - y, 0, y - t.y2))

        best = min(people, key=lambda t: (gap(t), (t.x2 - t.x1) * (t.y2 - t.y1)))
        if gap(best) > PICK_RADIUS:
            self.message = 'click on (or next to) a box around a person'
            return
        self.call_lock(best.id)

    def call_lock(self, track_id):
        if not self.lock_cli.service_is_ready():
            self.message = 'perception node not running'
            return
        self.lock_cli.call_async(LockTarget.Request(track_id=track_id)).add_done_callback(self._done)

    def command(self, cmd, value=''):
        if not self.cmd_cli.service_is_ready():
            self.message = 'follow controller not running'
            return
        self.message = f'sent: {cmd} {value}'.strip()
        self.cmd_cli.call_async(Command.Request(command=cmd, value=str(value))).add_done_callback(self._done)

    def handle_key(self, key):
        s = self.status
        c = chr(key) if key < 256 else ''
        if c == 't':
            self.command('takeoff')
        elif c == 'l':
            self.command('land')
        elif c == 'h':
            self.call_lock(-1)
            self.command('hover')
        elif c == 'm' and s is not None:
            self.command('follow_mode', MODES[(MODES.index(s.follow_mode) + 1) % len(MODES)])
        elif c in ('+', '=') and s is not None:
            self.command('distance', f'{s.desired_distance + 2:.1f}')
        elif c == '-' and s is not None:
            self.command('distance', f'{max(3.0, s.desired_distance - 2):.1f}')
        elif c == '0':
            self.command('distance', 'auto')
        elif c == 'r' and s is not None:
            self.command('altitude', f'{s.desired_altitude + 2:.1f}')
        elif c == 'f' and s is not None:
            self.command('altitude', f'{s.desired_altitude - 2:.1f}')
        elif c in MOVES:
            self.command('move', MOVES[c])
        elif c == 'q':
            self.command('yaw', '30')
        elif c == 'e':
            self.command('yaw', '-30')
        elif c == 'z':
            self.fullscreen = not self.fullscreen
            cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL)

    # ------------------------------------------------------------------ drawing

    def render(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            frame = np.full((720, 1280, 3), 30, np.uint8)
            cv2.putText(frame, 'waiting for camera...', (500, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (200, 200, 200), 2, cv2.LINE_AA)
        self.scale = VIDEO_W / frame.shape[1]
        video = cv2.resize(frame, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        h, w = video.shape[:2]
        self.video_h = h
        cv2.drawMarker(video, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)
        self._hint(video)
        self._minimap(video)
        info = self._info_panel(w)
        buttons = self._button_panel(w, h + INFO_H)
        return np.vstack([video, info, buttons])

    def _hint(self, video):
        """Big centred instruction for what to do next."""
        s, t, tr = self.status, self.target, self.tracks
        n_people = sum(1 for x in tr.tracks if x.class_name == 'person') if tr else 0
        if s is None or not s.connected:
            text = 'waiting for the drone link ...'
        elif s.flight_state == 'LANDED':
            text = 'press TAKE OFF'
        elif s.flight_state == 'TAKEOFF':
            text = 'taking off ...'
        elif t is not None and t.status != TargetState.NONE:
            return
        elif n_people == 0:
            text = 'no people in view - use FORWARD / YAW to look around'
        else:
            text = f'CLICK a person to track ({n_people} detected)'
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        x, y = (video.shape[1] - tw) // 2, video.shape[0] - 24
        cv2.rectangle(video, (x - 10, y - th - 10), (x + tw + 10, y + 10), (0, 0, 0), -1)
        cv2.putText(video, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

    def _info_panel(self, w):
        panel = np.full((INFO_H, w, 3), 25, np.uint8)
        s, t = self.status, self.target
        line1 = 'no flight controller'
        if s is not None:
            line1 = (f'{s.flight_state:<9} {"ARMED" if s.armed else "disarmed"} {"OFFBOARD" if s.offboard else ""}'
                     f'   alt {s.drone_position.z:5.1f} / {s.desired_altitude:.0f} m'
                     f'   dist {s.target_distance:5.1f} / {s.desired_distance:.1f} m   mode {s.follow_mode}')
        line2 = f'target: {STATUS_NAMES.get(t.status, "?") if t else "none"}'
        if t is not None and t.status != TargetState.NONE:
            line2 += f' (id {t.track_id})'
            if t.status == TargetState.LOST:
                line2 += f' {t.lost_for:4.1f}s'
        if s is not None and s.target_valid:
            line2 += f'  speed {math.hypot(s.target_velocity.x, s.target_velocity.y):3.1f} m/s'
        line2 += f'    | {self.message}'
        cv2.putText(panel, line1, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(panel, line2, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 255, 255), 1, cv2.LINE_AA)
        return panel

    def _button_panel(self, w, y0):
        rows = len(BUTTONS)
        panel = np.full((rows * (BTN_H + BTN_GAP) + BTN_GAP, w, 3), 40, np.uint8)
        self.buttons = []
        s = self.status
        for r, row in enumerate(BUTTONS):
            bw = (w - BTN_GAP * (len(row) + 1)) // len(row)
            for i, (label, key) in enumerate(row):
                x1 = BTN_GAP + i * (bw + BTN_GAP)
                y1 = BTN_GAP + r * (BTN_H + BTN_GAP)
                x2, y2 = x1 + bw, y1 + BTN_H
                col = (70, 70, 70)
                if key == 't':
                    col = (40, 120, 40)
                elif key == 'l':
                    col = (40, 40, 140)
                elif key == 'm' and s is not None:
                    label = f'MODE {s.follow_mode}'
                cv2.rectangle(panel, (x1, y1), (x2, y2), col, -1)
                cv2.rectangle(panel, (x1, y1), (x2, y2), (150, 150, 150), 1)
                scale = 0.5
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
                while tw > bw - 8 and scale > 0.3:          # shrink the label to fit the button
                    scale -= 0.02
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
                cv2.putText(panel, label, (x1 + (bw - tw) // 2, y1 + (BTN_H + th) // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
                self.buttons.append((x1, y0 + y1, x2, y0 + y2, key))
        return panel

    def _minimap(self, video):
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
        tip = (int(c + 14 * math.cos(yaw)), int(c - 14 * math.sin(yaw)))
        cv2.arrowedLine(m, (c, c), tip, (255, 255, 255), 2, tipLength=0.4)
        if s.target_valid:
            dx = s.target_position.x - s.drone_position.x
            dy = s.target_position.y - s.drone_position.y
            pt = (int(c + dx * k), int(c - dy * k))
            cv2.circle(m, pt, 5, (0, 0, 255), -1)
            v = (int(pt[0] + 3 * s.target_velocity.x * k), int(pt[1] - 3 * s.target_velocity.y * k))
            cv2.line(m, pt, v, (0, 0, 255), 1)
        cv2.putText(m, 'N', (c - 4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        h, w = video.shape[:2]
        roi = video[8:8 + MAP_SIZE, w - MAP_SIZE - 8:w - 8]
        video[8:8 + MAP_SIZE, w - MAP_SIZE - 8:w - 8] = cv2.addWeighted(roi, 0.3, m, 0.7, 0)


def main():
    rclpy.init()
    node = Console()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            node.click(x, y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            node.click(x, y, right=True)

    # NORMAL: resizable, mouse coordinates stay in image pixels; GUI_NORMAL: no Qt toolbar
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
    cv2.setMouseCallback(WINDOW, on_mouse)
    img = node.render()
    k = min(1.0, FIT_H / img.shape[0])
    cv2.resizeWindow(WINDOW, int(img.shape[1] * k), int(img.shape[0] * k))
    cv2.imshow(WINDOW, img)
    cv2.moveWindow(WINDOW, 10000, 0)             # window managers clamp this to the top-right corner
    cv2.setWindowProperty(WINDOW, cv2.WND_PROP_TOPMOST, 1)
    try:
        while rclpy.ok():
            cv2.imshow(WINDOW, node.render())
            key = cv2.waitKey(30)
            key = key & 0xFF if key != -1 else -1
            if key == 27 or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if 0 <= key < 256:
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
