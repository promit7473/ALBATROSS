"""Follow node: geolocates the locked target, estimates its motion and flies the drone.

Flight state machine (PX4 offboard via MAVROS)

  DISCONNECTED --fcu connected--> LANDED --takeoff--> TAKEOFF --at altitude--> HOVER
  HOVER  --target locked & located--> FOLLOW
  FOLLOW --target lost--> (coast on prediction for `coast_time`) --> SEARCH
  SEARCH --target re-acquired--> FOLLOW ;  --lock released / timeout--> HOVER
  any airborne state --land--> LANDING --disarmed--> LANDED
  any airborne state --pilot leaves OFFBOARD--> PILOT (we stop commanding)
  FOLLOW / SEARCH --no camera frame for `video_timeout`--> HOVER (real video link dropped)

Keep-out: every airborne velocity command passes keep_out() while the target position is
known: inside `keepout_soft` no motion toward the person, inside `keepout_hard` back away and climb.

Subscribes  /mavros/state, /mavros/local_position/pose, tracking/target, camera/camera_info
Publishes   /mavros/setpoint_raw/local (velocity + yaw-rate setpoints, 20 Hz)
            follow/status (FollowStatus), follow/target_pose (PoseStamped, ENU)
Services    follow/command (Command): takeoff | land | hover | follow_mode <m> |
            distance <m|auto> | altitude <m> | orbit_rate <deg/s> |
            move "<forward> <left>" [m, body frame] | yaw <deg>   (move/yaw: HOVER only)
"""
import bisect
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, ParamSetV2, SetMode
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tracker_interfaces.msg import FollowStatus, TargetState
from tracker_interfaces.srv import Command

from tracker_control.follow_controller import FollowController, FollowParams, keep_out
from tracker_control.geometry import CameraModel, quat_to_rot, wrap, yaw_of
from tracker_control.target_estimator import TargetEstimator

# velocity + yaw-rate setpoint: ignore position, acceleration and yaw
TYPE_MASK_VEL_YAWRATE = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                         PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW)
AIRBORNE = ('TAKEOFF', 'HOVER', 'FOLLOW', 'SEARCH')


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PoseBuffer:
    """Recent drone poses, so a detection is geolocated with the pose at its capture time."""

    def __init__(self, horizon=2.0):
        self.horizon = horizon
        self.t, self.data = deque(), deque()

    def add(self, t, p, R):
        self.t.append(t)
        self.data.append((p, R))
        while self.t and self.t[0] < t - self.horizon:
            self.t.popleft()
            self.data.popleft()

    def at(self, t):
        if not self.t:
            return None
        ts = list(self.t)
        i = bisect.bisect_left(ts, t)
        if i == 0:
            return self.data[0]
        if i >= len(ts):
            return self.data[-1]
        return self.data[i] if ts[i] - t < t - ts[i - 1] else self.data[i - 1]


class FollowNode(Node):
    def __init__(self):
        super().__init__('follow')
        d = FollowParams()
        decl = self.declare_parameters('', [
            ('altitude', d.altitude), ('distance', d.distance), ('mode', d.mode),
            ('orbit_rate_deg', math.degrees(d.orbit_rate)),
            ('bearing_rate_deg', math.degrees(d.bearing_rate_max)),
            ('camera_pitch_deg', 40.0), ('camera_hfov', 1.2), ('camera_offset', [0.12, 0.0, 0.20]),
            ('target_centre_height', d.target_centre_height),
            ('kp_xy', d.kp_xy), ('ki_xy', d.ki_xy), ('kp_z', d.kp_z), ('kp_yaw', d.kp_yaw),
            ('v_max_xy', d.v_max_xy), ('v_max_z', d.v_max_z), ('a_max_xy', d.a_max_xy),
            ('yaw_rate_max_deg', math.degrees(d.yaw_rate_max)),
            ('min_altitude', d.min_altitude), ('geofence_radius', d.geofence_radius),
            ('coast_time', 3.0), ('search_yaw_rate_deg', 25.0),
            ('target_accel_std', 1.5), ('ground_z', 0.0),
            ('setpoint_rate', 20.0),
            ('video_timeout', 0.0),          # [s] 0 = off (simulation)
            ('keepout_soft', 4.0),           # [m] no motion toward the person inside this
            ('keepout_hard', 2.5),           # [m] back away + climb inside this
            ('px4_params', ['']),
        ])
        prm = {x.name: x.value for x in decl}
        self.prm = prm
        self.fp = FollowParams(
            altitude=prm['altitude'], distance=prm['distance'], mode=prm['mode'],
            orbit_rate=math.radians(prm['orbit_rate_deg']),
            bearing_rate_max=math.radians(prm['bearing_rate_deg']), camera_pitch=math.radians(prm['camera_pitch_deg']),
            target_centre_height=prm['target_centre_height'],
            kp_xy=prm['kp_xy'], ki_xy=prm['ki_xy'], kp_z=prm['kp_z'], kp_yaw=prm['kp_yaw'],
            v_max_xy=prm['v_max_xy'], v_max_z=prm['v_max_z'], a_max_xy=prm['a_max_xy'],
            yaw_rate_max=math.radians(prm['yaw_rate_max_deg']),
            min_altitude=prm['min_altitude'], geofence_radius=prm['geofence_radius'])
        self.controller = FollowController(self.fp)
        self.estimator = TargetEstimator(accel_std=prm['target_accel_std'])
        self.camera = None
        self.poses = PoseBuffer()

        self.fsm = 'DISCONNECTED'
        self.mav_state = State()
        self.pose = None                     # (p, R, yaw)
        self.hold_xy, self.hold_yaw = None, 0.0
        self.target_status = TargetState.NONE
        self.last_target_seen = 0.0
        self.image_yaw_err, self.image_yaw_err_t = None, 0.0
        self.params_sent = False
        self.last_request = 0.0
        self.search_dir = 1.0
        self.last_frame_t = None
        self.keepout_zone = 'clear'

        self.sp_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.status_pub = self.create_publisher(FollowStatus, 'follow/status', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, 'follow/target_pose', 10)
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(TargetState, 'tracking/target', self.on_target, 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_camera_info, 2)
        self.create_service(Command, 'follow/command', self.on_command)
        self.arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set = self.create_client(ParamSetV2, '/mavros/param/set')

        self.dt = 1.0 / prm['setpoint_rate']
        self.last_tick = time.monotonic()
        self.create_timer(self.dt, self.tick)

    # ------------------------------------------------------------------ inputs

    def on_state(self, msg):
        self.mav_state = msg

    def on_pose(self, msg):
        q = msg.pose.orientation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.pose = (p, R, yaw_of(R))
        self.poses.add(self.now(), p, R)     # receive time, same clock as the camera frames

    def on_camera_info(self, msg):
        self.last_frame_t = self.now()       # one CameraInfo per frame: doubles as a video heartbeat
        if self.camera is None and msg.k[0] > 0:
            off = self.prm['camera_offset']
            self.camera = CameraModel(msg.k[0], msg.k[4], msg.k[2], msg.k[5],
                                      math.radians(self.prm['camera_pitch_deg']), off)
            self.get_logger().info(f'camera fx={msg.k[0]:.1f} cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')

    def on_target(self, msg):
        self.target_status = msg.status
        if msg.status == TargetState.NONE:
            self.estimator.reset()
            return
        if msg.status != TargetState.TRACKING:
            return
        if self.camera is None:
            self.camera = CameraModel.from_fov(msg.image_width, msg.image_height, self.prm['camera_hfov'],
                                               math.radians(self.prm['camera_pitch_deg']),
                                               self.prm['camera_offset'])
        t = stamp_to_sec(msg.header.stamp)
        pose = self.poses.at(t)
        if pose is None:
            return
        tr = msg.track
        u, v = (tr.x1 + tr.x2) / 2, tr.y2                    # feet: bottom centre of the box
        hit = self.camera.ground_point(u, v, pose[0], pose[1], self.prm['ground_z'])
        if hit is None:
            return
        self.estimator.update(hit[0], hit[1], t)
        self.last_target_seen = t
        self.image_yaw_err = self.camera.bearing_error((tr.x1 + tr.x2) / 2)
        self.image_yaw_err_t = t

    def on_command(self, req, res):
        cmd, val = req.command.strip().lower(), req.value.strip()
        res.success, res.message = True, ''
        try:
            if cmd == 'takeoff':
                if self.fsm not in ('LANDED', 'PILOT'):
                    raise ValueError(f'cannot take off from {self.fsm}')
                self._enter('TAKEOFF')
            elif cmd == 'land':
                self._enter('LANDING')
            elif cmd == 'hover':
                if not self.mav_state.armed:
                    raise ValueError('not armed')
                self._enter('HOVER')
            elif cmd == 'follow_mode':
                if val not in ('behind', 'hold_bearing', 'orbit'):
                    raise ValueError('mode must be behind | hold_bearing | orbit')
                self.fp.mode = val
                self.controller.reset(self.controller.bearing)
            elif cmd == 'distance':
                self.fp.distance = 0.0 if val == 'auto' else float(val)
            elif cmd == 'altitude':
                self.fp.altitude = max(self.fp.min_altitude, float(val))
            elif cmd in ('move', 'yaw'):
                if self.fsm != 'HOVER' or self.hold_xy is None:
                    raise ValueError('move/yaw only while hovering (press h to stop following)')
                if cmd == 'move':
                    fwd, left = (float(x) for x in val.split())
                    c, s_ = math.cos(self.hold_yaw), math.sin(self.hold_yaw)
                    self.hold_xy = self.hold_xy + np.array([c * fwd - s_ * left, s_ * fwd + c * left])
                else:
                    self.hold_yaw = wrap(self.hold_yaw + math.radians(float(val)))
            elif cmd == 'orbit_rate':
                self.fp.orbit_rate = math.radians(float(val))
            else:
                raise ValueError(f'unknown command "{cmd}"')
            res.message = f'{cmd} {val}'.strip()
        except ValueError as e:
            res.success, res.message = False, str(e)
        self.get_logger().info(('ok: ' if res.success else 'rejected: ') + res.message)
        return res

    # ------------------------------------------------------------------ state machine

    def _enter(self, state):
        if state == self.fsm:
            return
        self.get_logger().info(f'{self.fsm} -> {state}')
        if state in ('HOVER', 'TAKEOFF', 'SEARCH') and self.pose is not None:
            self.hold_xy, self.hold_yaw = self.pose[0][:2].copy(), self.pose[2]
        if state == 'FOLLOW':
            self.controller.reset()
        if state == 'LANDING':
            self._call(self.set_mode, SetMode.Request(custom_mode='AUTO.LAND'))
        self.fsm = state

    def tick(self):
        now_m = time.monotonic()
        dt, self.last_tick = now_m - self.last_tick, now_m
        st = self.mav_state

        if not st.connected:
            self.fsm = 'DISCONNECTED'
            self._publish_status()
            return
        if self.fsm == 'DISCONNECTED':
            self._send_px4_params()
            self._enter('HOVER' if st.armed and st.mode == 'OFFBOARD' else 'LANDED')
        if self.pose is None:
            self._publish_status()
            return

        if self.fsm in AIRBORNE and st.armed and st.mode != 'OFFBOARD' and self.fsm != 'TAKEOFF':
            self.get_logger().warn(f'pilot switched to {st.mode}: releasing control')
            self._enter('PILOT')
        if self.fsm == 'LANDING' and not st.armed:
            self._enter('LANDED')

        t = self.now()
        target_fresh = self.estimator.valid and t - self.last_target_seen < self.prm['coast_time']
        vt = self.prm['video_timeout']
        video_ok = vt <= 0 or (self.last_frame_t is not None and t - self.last_frame_t < vt)
        if not video_ok and self.fsm in ('FOLLOW', 'SEARCH'):
            self.get_logger().warn('camera video lost: holding position')
            self._enter('HOVER')

        if self.fsm == 'TAKEOFF':
            self._ensure_offboard_and_armed()
            self._hold(dt, climb=True)
            if abs(self.pose[0][2] - self.fp.altitude) < 0.5:
                self._enter('HOVER')
        elif self.fsm == 'HOVER':
            self._hold(dt)
            if self.target_status == TargetState.TRACKING and target_fresh and video_ok:
                self._enter('FOLLOW')
        elif self.fsm == 'FOLLOW':
            if self.target_status == TargetState.NONE:
                self._enter('HOVER')
                self._hold(dt)
            elif not target_fresh:
                self._enter('SEARCH')
                self._search(dt)
            else:
                self._follow(dt, t)
        elif self.fsm == 'SEARCH':
            if self.target_status == TargetState.NONE:
                self._enter('HOVER')
                self._hold(dt)
            elif self.target_status == TargetState.TRACKING and target_fresh:
                self._enter('FOLLOW')
                self._follow(dt, t)
            else:
                self._search(dt)
        else:
            # LANDED / LANDING / PILOT: keep a neutral setpoint stream so offboard can engage later
            self._send(0.0, 0.0, 0.0, 0.0)
        self._publish_status()

    # ------------------------------------------------------------------ behaviours

    def _hold(self, dt, climb=False):
        """Hold hover point (+ climb during takeoff) with a P loop on position."""
        p, _, yaw = self.pose
        if self.hold_xy is None:
            self.hold_xy, self.hold_yaw = p[:2].copy(), yaw
        e = self.hold_xy - p[:2]
        v = np.clip(0.8 * e, -2.0, 2.0)
        z_des = self.fp.altitude
        vz = float(np.clip(0.8 * (z_des - p[2]), -self.fp.v_max_z, 1.5 if climb else self.fp.v_max_z))
        yaw_rate = float(np.clip(1.0 * wrap(self.hold_yaw - yaw), -0.5, 0.5))
        self._send(v[0], v[1], vz, yaw_rate)

    def _follow(self, dt, t):
        p, _, yaw = self.pose
        tp, tv = self.estimator.state_at(t)
        img_err = self.image_yaw_err if t - self.image_yaw_err_t < 0.3 else None
        vx, vy, vz, yaw_rate, _ = self.controller.compute(dt, p, yaw, tp, tv, img_err)
        self._send(vx, vy, vz, yaw_rate)

    def _search(self, dt):
        """Hover and turn toward where the target was predicted to go, then sweep."""
        p, _, yaw = self.pose
        e = self.hold_xy - p[:2]
        v = np.clip(0.8 * e, -2.0, 2.0)
        vz = float(np.clip(0.8 * (self.fp.altitude - p[2]), -self.fp.v_max_z, self.fp.v_max_z))
        rate = math.radians(self.prm['search_yaw_rate_deg'])
        if self.estimator.valid:
            tp, _ = self.estimator.state_at(min(self.now(), self.last_target_seen + 5.0))
            err = wrap(math.atan2(tp[1] - p[1], tp[0] - p[0]) - yaw)
            yaw_rate = float(np.clip(1.5 * err, -rate, rate)) if abs(err) > 0.2 else self.search_dir * rate * 0.5
        else:
            yaw_rate = self.search_dir * rate
        self._send(v[0], v[1], vz, yaw_rate)

    # ------------------------------------------------------------------ MAVROS helpers

    def _send(self, vx, vy, vz, yaw_rate):
        if self.fsm in AIRBORNE and self.pose is not None and self.estimator.valid:
            t = self.now()
            tp, _ = self.estimator.state_at(min(t, self.last_target_seen + self.prm['coast_time']))
            v, zone = keep_out((vx, vy), self.pose[0][:2], tp, self.prm['keepout_soft'],
                               self.prm['keepout_hard'], self.fp.v_max_xy)
            if zone != self.keepout_zone:
                log = self.get_logger().warn if zone != 'clear' else self.get_logger().info
                log(f'keep-out: {zone}')
                self.keepout_zone = zone
            vx, vy = v
            if zone == 'hard':
                vz = max(vz, 0.5)
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED      # MAVROS converts from ENU
        sp.type_mask = TYPE_MASK_VEL_YAWRATE
        sp.velocity.x, sp.velocity.y, sp.velocity.z = float(vx), float(vy), float(vz)
        sp.yaw_rate = float(yaw_rate)
        self.sp_pub.publish(sp)

    def _call(self, client, req):
        if client.service_is_ready():
            client.call_async(req)
        else:
            self.get_logger().warn(f'service {client.srv_name} not available')

    def _ensure_offboard_and_armed(self):
        now = time.monotonic()
        if now - self.last_request < 1.0:
            return
        self.last_request = now
        if self.mav_state.mode != 'OFFBOARD':
            self._call(self.set_mode, SetMode.Request(custom_mode='OFFBOARD'))
        elif not self.mav_state.armed:
            self._call(self.arming, CommandBool.Request(value=True))

    def _send_px4_params(self):
        """Apply simulation-friendly PX4 parameters ("NAME=VALUE" list) once per connection."""
        if self.params_sent:
            return
        self.params_sent = True
        if not any('=' in item for item in self.prm['px4_params']):
            return
        if not self.param_set.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/mavros/param/set unavailable: PX4 parameters not applied')
            return
        for item in self.prm['px4_params']:
            if '=' not in item:
                continue
            name, val = (s.strip() for s in item.split('=', 1))
            pv = ParameterValue()
            if '.' in val:
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(val)
            else:
                pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, int(val)
            self.param_set.call_async(ParamSetV2.Request(force_set=True, param_id=name, value=pv))
            self.get_logger().info(f'PX4 param {name} = {val}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self):
        s = FollowStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'map'
        s.flight_state = self.fsm
        s.follow_mode = self.fp.mode
        s.connected = self.mav_state.connected
        s.armed = self.mav_state.armed
        s.offboard = self.mav_state.mode == 'OFFBOARD'
        s.desired_altitude = float(self.fp.altitude)
        s.desired_distance = float(self.controller.standoff_distance())
        if self.pose is not None:
            p, _, yaw = self.pose
            s.drone_position.x, s.drone_position.y, s.drone_position.z = (float(v) for v in p)
            s.drone_yaw = float(yaw)
        if self.estimator.valid:
            # do not extrapolate a lost target further than the coast horizon
            tp, tv = self.estimator.state_at(min(self.now(), self.last_target_seen + self.prm['coast_time']))
            s.target_valid = True
            s.target_position.x, s.target_position.y = float(tp[0]), float(tp[1])
            s.target_velocity.x, s.target_velocity.y = float(tv[0]), float(tv[1])
            if self.pose is not None:
                s.target_distance = float(np.hypot(*(tp - self.pose[0][:2])))
            ps = PoseStamped()
            ps.header = s.header
            ps.pose.position.x, ps.pose.position.y = float(tp[0]), float(tp[1])
            ps.pose.orientation.w = 1.0
            self.target_pose_pub.publish(ps)
        self.status_pub.publish(s)


def main():
    rclpy.init()
    node = FollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
