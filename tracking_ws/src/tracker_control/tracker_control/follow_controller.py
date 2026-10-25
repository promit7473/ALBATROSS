"""Follow guidance: where the drone should be relative to the target, and how to get there.

Geometry
  The drone stands off at horizontal distance D and altitude H from the target,
  on bearing b (angle of the target->drone vector):
      p_des = p_target + D [cos b, sin b],   z_des = H
  With a fixed camera pitched down by theta, the target is centred vertically
  when (H - h_c) / D = tan(theta)  (h_c = height of the person's centre), so by
  default D is derived from H:  D_auto = (H - h_c) / tan(theta).
  A (near-)horizontal camera has no such relation: set `distance` explicitly; with
  distance = 0 the closest allowed distance is used.

Bearing (follow modes)
  behind        when the target walks (|v| > v_min), swing round to stand behind
                it (b -> heading + 180 deg) at a limited angular rate; when it
                stops, keep the current bearing (no needless circling)
  hold_bearing  keep the bearing captured when following started
  orbit         rotate the bearing at a constant rate (circle the target)

Translation (velocity command, ENU)
  v = v_target + d(p_des)/dt|bearing + Kp e + Ki ∫e,   e = p_des - p
  The target-velocity feed-forward is what lets the drone keep up with a moving
  person without a steady-state lag; the PI term removes the residual error.
  The command is saturated in speed and rate-limited in acceleration.

Heading
  yaw_rate = K_psi * e_psi + LOS rate feed-forward, where e_psi comes from the
  image (pixel offset of the target from the image centre) when a fresh
  detection exists -- lowest latency -- otherwise from the estimated geometry.
"""
import math
from dataclasses import dataclass

import numpy as np

from tracker_control.geometry import wrap

MIN_AUTO_PITCH = math.radians(5.0)      # below this the automatic stand-off distance is undefined


@dataclass
class FollowParams:
    altitude: float = 10.0
    distance: float = 0.0              # <= 0: automatic from altitude & camera pitch
    min_distance: float = 5.0
    max_distance: float = 40.0
    camera_pitch: float = math.radians(40.0)
    target_centre_height: float = 0.9
    mode: str = 'behind'               # behind | hold_bearing | orbit
    orbit_rate: float = math.radians(10.0)
    behind_min_speed: float = 0.6
    bearing_rate_max: float = math.radians(15.0)
    kp_xy: float = 0.8
    ki_xy: float = 0.05
    i_max: float = 2.0
    kp_z: float = 0.8
    kp_yaw: float = 1.5
    v_max_xy: float = 10.0
    v_max_z: float = 2.0
    a_max_xy: float = 3.0
    yaw_rate_max: float = math.radians(60.0)
    min_altitude: float = 3.0
    geofence_radius: float = 160.0


class FollowController:
    def __init__(self, params: FollowParams):
        self.p = params
        self.reset()

    def reset(self, bearing=None):
        self.bearing = bearing
        self.integral = np.zeros(2)
        self.v_cmd = np.zeros(2)

    def standoff_distance(self):
        p = self.p
        if p.distance > 0:
            d = p.distance
        elif p.camera_pitch > MIN_AUTO_PITCH:
            d = (p.altitude - p.target_centre_height) / math.tan(p.camera_pitch)
        else:
            d = p.min_distance
        return float(np.clip(d, p.min_distance, p.max_distance))

    def _update_bearing(self, dt, drone_xy, target_xy, target_v):
        p = self.p
        current = math.atan2(drone_xy[1] - target_xy[1], drone_xy[0] - target_xy[0])
        if self.bearing is None:
            self.bearing = current
        prev = self.bearing
        if p.mode == 'orbit':
            self.bearing = wrap(self.bearing + p.orbit_rate * dt)
        elif p.mode == 'behind':
            speed = float(np.hypot(*target_v))
            if speed > p.behind_min_speed:
                desired = math.atan2(-target_v[1], -target_v[0])
                step = np.clip(wrap(desired - self.bearing), -p.bearing_rate_max * dt, p.bearing_rate_max * dt)
                self.bearing = wrap(self.bearing + step)
        # hold_bearing: unchanged
        return wrap(self.bearing - prev) / dt if dt > 0 else 0.0

    def compute(self, dt, drone_xyz, drone_yaw, target_xy, target_v, image_yaw_error=None):
        """Returns (vx, vy, vz, yaw_rate) in ENU and a dict of internals for logging."""
        p = self.p
        drone_xy = np.asarray(drone_xyz[:2], float)
        target_xy = np.asarray(target_xy, float)
        target_v = np.asarray(target_v, float)
        D = self.standoff_distance()

        bearing_rate = self._update_bearing(dt, drone_xy, target_xy, target_v)
        u = np.array([math.cos(self.bearing), math.sin(self.bearing)])
        p_des = target_xy + D * u
        # geofence: never chase the target outside the allowed radius around home
        r = np.linalg.norm(p_des)
        if r > p.geofence_radius:
            p_des *= p.geofence_radius / r

        # translation: feed-forward + PI on position error
        e = p_des - drone_xy
        self.integral = np.clip(self.integral + e * dt, -p.i_max, p.i_max)
        ff = target_v + D * bearing_rate * np.array([-u[1], u[0]])
        v = ff + p.kp_xy * e + p.ki_xy * self.integral
        n = np.linalg.norm(v)
        if n > p.v_max_xy:
            v *= p.v_max_xy / n
        dv = v - self.v_cmd
        n = np.linalg.norm(dv)
        if n > p.a_max_xy * dt:
            dv *= p.a_max_xy * dt / n
        self.v_cmd = self.v_cmd + dv

        # altitude
        z_des = max(p.altitude, p.min_altitude)
        vz = float(np.clip(p.kp_z * (z_des - drone_xyz[2]), -p.v_max_z, p.v_max_z))

        # heading: look at the target
        rel = target_xy - drone_xy
        los = math.atan2(rel[1], rel[0])
        e_psi = image_yaw_error if image_yaw_error is not None else wrap(los - drone_yaw)
        rel_v = target_v - self.v_cmd
        los_rate = (rel[0] * rel_v[1] - rel[1] * rel_v[0]) / max(rel @ rel, 1.0)
        yaw_rate = float(np.clip(p.kp_yaw * e_psi + los_rate, -p.yaw_rate_max, p.yaw_rate_max))

        info = {'p_des': p_des, 'distance': float(np.linalg.norm(rel)), 'standoff': D,
                'bearing': self.bearing, 'error': float(np.linalg.norm(e))}
        return self.v_cmd[0], self.v_cmd[1], vz, yaw_rate, info
