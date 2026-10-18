"""Target state estimator in the world frame (constant-velocity Kalman filter).

State x = [px, py, vx, vy]. Process noise: white acceleration with standard
deviation `accel_std` (a walking person changes velocity slowly, a jogger more).
Measurements are geolocated ground points whose noise grows with range
(pixel error x range, plus pose/attitude error). Outliers -- e.g. a box that
momentarily jumped to a neighbour -- are rejected by a chi-square gate on the
Mahalanobis distance of the innovation.
"""
import numpy as np

CHI2_2DOF_99 = 9.21


class TargetEstimator:
    def __init__(self, accel_std=1.5, meas_std_base=0.3, meas_std_per_m=0.03, max_speed=8.0):
        self.accel_std = accel_std
        self.meas_std_base, self.meas_std_per_m = meas_std_base, meas_std_per_m
        self.max_speed = max_speed
        self.reset()

    def reset(self):
        self.x = None
        self.P = None
        self.t = None
        self.rejected = 0

    @property
    def valid(self):
        return self.x is not None

    def predict(self, t):
        if self.x is None or t <= self.t:
            return
        dt = t - self.t
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.accel_std ** 2
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + q * (G @ G.T)
        self.t = t

    def update(self, z, slant_range, t):
        """Fuse a ground-point measurement z = (x, y) taken at time t. Returns False if gated out."""
        z = np.asarray(z, float)
        std = self.meas_std_base + self.meas_std_per_m * slant_range
        R = np.eye(2) * std ** 2
        if self.x is None:
            self.x = np.r_[z, 0.0, 0.0]
            self.P = np.diag([std ** 2, std ** 2, 4.0, 4.0])
            self.t = t
            return True
        self.predict(t)
        H = np.eye(2, 4)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        if y @ np.linalg.solve(S, y) > CHI2_2DOF_99:
            self.rejected += 1
            if self.rejected >= 5:              # persistent disagreement: the filter is wrong, restart
                self.reset()
                return self.update(z, slant_range, t)
            return False
        self.rejected = 0
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        v = np.hypot(*self.x[2:])
        if v > self.max_speed:
            self.x[2:] *= self.max_speed / v
        return True

    def state_at(self, t):
        """(position, velocity) extrapolated to time t without modifying the filter."""
        dt = max(0.0, t - self.t)
        return self.x[:2] + self.x[2:] * dt, self.x[2:].copy()

    @property
    def position_std(self):
        return float(np.sqrt(max(self.P[0, 0], self.P[1, 1])))
