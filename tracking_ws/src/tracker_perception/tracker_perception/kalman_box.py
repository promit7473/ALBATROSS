"""Constant-velocity Kalman filter for an image bounding box.

State  x = [cx, cy, a, h, vcx, vcy, va, vh]   (centre, aspect w/h, height + rates)
Meas.  z = [cx, cy, a, h]
Noise is scaled with box height (as in SORT / DeepSORT / ByteTrack), so a far
small person and a near large one get comparable relative uncertainty.
"""
import numpy as np

STD_POS = 1.0 / 20      # position noise as a fraction of box height
STD_VEL = 1.0 / 160


def xyxy_to_xyah(b):
    w, h = b[2] - b[0], b[3] - b[1]
    return np.array([b[0] + w / 2, b[1] + h / 2, w / max(h, 1e-6), h], dtype=np.float64)


def xyah_to_xyxy(s):
    cx, cy, a, h = s[:4]
    w = a * h
    return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], dtype=np.float32)


class KalmanBox:
    _F = np.eye(8)
    _F[:4, 4:] = np.eye(4)
    _H = np.eye(4, 8)

    def __init__(self, box_xyxy):
        z = xyxy_to_xyah(box_xyxy)
        self.x = np.r_[z, np.zeros(4)]
        h = z[3]
        std = [2 * STD_POS * h, 2 * STD_POS * h, 1e-2, 2 * STD_POS * h,
               10 * STD_VEL * h, 10 * STD_VEL * h, 1e-5, 10 * STD_VEL * h]
        self.P = np.diag(np.square(std))

    def predict(self):
        h = self.x[3]
        q = [STD_POS * h, STD_POS * h, 1e-2, STD_POS * h, STD_VEL * h, STD_VEL * h, 1e-5, STD_VEL * h]
        self.x = self._F @ self.x
        self.P = self._F @ self.P @ self._F.T + np.diag(np.square(q))
        self.x[3] = max(self.x[3], 1.0)

    def update(self, box_xyxy):
        z = xyxy_to_xyah(box_xyxy)
        h = self.x[3]
        R = np.diag(np.square([STD_POS * h, STD_POS * h, 1e-1, STD_POS * h]))
        S = self._H @ self.P @ self._H.T + R
        K = np.linalg.solve(S, (self.P @ self._H.T).T).T
        self.x = self.x + K @ (z - self._H @ self.x)
        self.P = (np.eye(8) - K @ self._H) @ self.P

    def warp(self, A):
        """Apply a 2x3 similarity transform (camera motion) to the state."""
        R, t = A[:, :2], A[:, 2]
        scale = float(np.sqrt(abs(np.linalg.det(R))))
        self.x[:2] = R @ self.x[:2] + t
        self.x[4:6] = R @ self.x[4:6]
        self.x[3] *= scale
        self.x[7] *= scale
        T = np.eye(8)
        T[:2, :2] = R
        T[4:6, 4:6] = R
        self.P = T @ self.P @ T.T

    @property
    def box(self):
        return xyah_to_xyxy(self.x)
