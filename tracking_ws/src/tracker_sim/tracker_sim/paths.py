"""Planar path geometry shared by the world builder and the traffic node.

Pure python (no ROS) so tools/build_world.py can import it standalone.
"""
import bisect
import math


def _unit(dx, dy):
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n > 1e-9 else (0.0, 0.0)


def rounded_loop(corners, radius, arc_step_deg=10.0):
    """Closed polygon -> point list with a circular fillet of `radius` at every corner."""
    pts = []
    n = len(corners)
    for i in range(n):
        p0, p1, p2 = corners[i - 1], corners[i], corners[(i + 1) % n]
        d1 = _unit(p1[0] - p0[0], p1[1] - p0[1])
        d2 = _unit(p2[0] - p1[0], p2[1] - p1[1])
        turn = math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1[0] * d2[0] + d1[1] * d2[1])
        if abs(turn) < 1e-3:
            pts.append(tuple(p1))
            continue
        t = radius * math.tan(abs(turn) / 2.0)          # tangent length from the corner
        a = (p1[0] - d1[0] * t, p1[1] - d1[1] * t)
        side = 1.0 if turn > 0 else -1.0                 # +1 = left turn
        c = (a[0] - side * d1[1] * radius, a[1] + side * d1[0] * radius)
        phi0 = math.atan2(a[1] - c[1], a[0] - c[0])
        steps = max(2, int(math.degrees(abs(turn)) / arc_step_deg) + 1)
        for k in range(steps + 1):
            phi = phi0 + turn * k / steps
            pts.append((c[0] + radius * math.cos(phi), c[1] + radius * math.sin(phi)))
    return pts


class Route:
    """Polyline parameterised by arc length; pose_at(s) returns (x, y, yaw)."""

    def __init__(self, points, closed=True):
        self.points = [tuple(p) for p in points]
        if closed:
            self.points.append(self.points[0])
        self.closed = closed
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.cum[-1]

    def pose_at(self, s):
        s = s % self.length if self.closed else min(max(s, 0.0), self.length)
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.points) - 2)
        (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
        seg = self.cum[i + 1] - self.cum[i]
        f = (s - self.cum[i]) / seg if seg > 1e-9 else 0.0
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.atan2(y1 - y0, x1 - x0)


def walking_waypoints(path, speed, mode, turn_rate=math.pi, start_index=0):
    """Timed (t, x, y, yaw) waypoints for an actor walking a polyline.

    mode 'loop' closes the polygon, 'pingpong' walks to the end and back.
    At each corner the actor stops and turns in place (at turn_rate rad/s), so
    the walk animation never slides sideways. Yaw is unwrapped (continuous) so
    the interpolator never spins the long way round.
    """
    pts = [tuple(p) for p in path]
    if mode == 'pingpong':
        pts = pts + pts[-2::-1]
    else:
        pts = pts[start_index:] + pts[:start_index] + [pts[start_index]]

    wps, t, yaw_prev = [], 0.0, None
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        yaw = math.atan2(y1 - y0, x1 - x0)
        if yaw_prev is not None:
            yaw = yaw_prev + math.remainder(yaw - yaw_prev, 2 * math.pi)
            t += max(0.3, abs(yaw - yaw_prev) / turn_rate)
        wps.append((t, x0, y0, yaw))                    # (turned) at the segment start
        t += math.hypot(x1 - x0, y1 - y0) / speed
        wps.append((t, x1, y1, yaw))                    # arrived at the segment end
        yaw_prev = yaw
    # final turn back to the initial heading so the loop restarts seamlessly
    yaw0 = wps[0][3]
    yaw_end = yaw_prev + math.remainder(yaw0 - yaw_prev, 2 * math.pi)
    t += max(0.3, abs(yaw_end - yaw_prev) / turn_rate)
    wps.append((t, pts[-1][0], pts[-1][1], yaw_end))
    return wps
