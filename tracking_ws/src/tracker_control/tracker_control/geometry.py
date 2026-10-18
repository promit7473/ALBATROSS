"""Camera geometry: pixel -> world ray -> point on the ground.

Frames
  world  ENU (MAVROS local frame: x east, y north, z up; z = 0 at the home/ground)
  body   FLU (x forward, y left, z up), as reported by MAVROS local_position/pose
  camera FLU body of the camera, mounted pitched down by `pitch` about body y
  optical  x right, y down, z forward (pinhole / image convention)

A pixel (u, v) defines the optical ray d_o = [(u - cx)/fx, (v - cy)/fy, 1].
It is rotated into the world, d_w = R_wb · R_bc · R_co · d_o, and intersected
with the ground plane z = ground_z from the camera centre. For a person the
pixel used is the bottom-centre of the box (the feet), which lies on the ground.
"""
import math

import numpy as np

# optical -> camera-FLU:  x_c = z_o,  y_c = -x_o,  z_c = -y_o
R_CO = np.array([[0.0, 0.0, 1.0],
                 [-1.0, 0.0, 0.0],
                 [0.0, -1.0, 0.0]])


def quat_to_rot(x, y, z, w):
    """Rotation matrix from a unit quaternion (x, y, z, w)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def wrap(a):
    return math.remainder(a, 2 * math.pi)


class CameraModel:
    def __init__(self, fx, fy, cx, cy, pitch_rad, offset=(0.0, 0.0, 0.0)):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.R_bo = rot_y(pitch_rad) @ R_CO          # optical -> body
        self.offset = np.asarray(offset, float)       # camera position in body frame

    @classmethod
    def from_fov(cls, width, height, hfov, pitch_rad, offset=(0.0, 0.0, 0.0)):
        f = width / (2 * math.tan(hfov / 2))
        return cls(f, f, width / 2, height / 2, pitch_rad, offset)

    def ray_world(self, u, v, R_wb):
        d = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        d = R_wb @ (self.R_bo @ d)
        return d / np.linalg.norm(d)

    def ground_point(self, u, v, p_wb, R_wb, ground_z=0.0, max_range=120.0):
        """World (x, y) where pixel (u, v) hits the ground, and the slant range; None if no hit."""
        c = np.asarray(p_wb, float) + R_wb @ self.offset
        d = self.ray_world(u, v, R_wb)
        if d[2] > -0.05:                               # ray (almost) parallel to / away from ground
            return None
        s = (ground_z - c[2]) / d[2]
        if s <= 0 or s > max_range:
            return None
        hit = c + s * d
        return hit[:2], s

    def bearing_error(self, u):
        """Yaw angle to rotate so pixel column u is centred (+ = target is left, turn left)."""
        return -math.atan2(u - self.cx, self.fx)
