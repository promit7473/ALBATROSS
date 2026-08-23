"""Task 1 - autonomous path planning for the maritime mission.

Given start + goal (+ optional via-points), plan a flyable waypoint path that
stays inside the KML operating area and avoids its exclusion zones. Builds an
occupancy grid once (with obstacle dilation for clearance), runs A*, then
line-of-sight simplifies so the mission has few clean legs.

Coordinate-agnostic (see geo.py). Standalone demo:  python3 planner.py
"""
import heapq
import math

try:
    from .geo import OperatingMap, point_in_polygon, haversine_m
except ImportError:
    from geo import OperatingMap, point_in_polygon, haversine_m


def _ll_to_m(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * 6378137.0 * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * 6378137.0
    return x, y


def _m_to_ll(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / 6378137.0)
    lon = lon0 + math.degrees(x / (6378137.0 * math.cos(math.radians(lat0))))
    return lat, lon


class PathPlanner:
    def __init__(self, op_map: OperatingMap, op_poly=None, cell_m=80.0, clearance_m=150.0):
        self.map = op_map
        self.op_poly = op_poly if op_poly is not None else (
            op_map.operating_polygon() if op_map else None)
        self.exclusions = op_map.exclusions() if op_map else []
        self.cell = cell_m
        self.lat0, self.lon0 = (self.op_poly.centroid() if self.op_poly else (0.0, 0.0))
        if self.op_poly:
            self._build_grid(clearance_m)
        else:
            self.grid = None

    def _blocked_ll(self, lat, lon):
        if self.op_poly is not None and not point_in_polygon(lat, lon, self.op_poly.pts):
            return True
        for z in self.exclusions:
            if point_in_polygon(lat, lon, z.pts):
                return True
        return False

    def _build_grid(self, clearance_m):
        xs, ys = [], []
        for (la, lo) in self.op_poly.pts:
            x, y = _ll_to_m(la, lo, self.lat0, self.lon0)
            xs.append(x); ys.append(y)
        self.minx, self.maxx = min(xs), max(xs)
        self.miny, self.maxy = min(ys), max(ys)
        self.nx = int((self.maxx - self.minx) / self.cell) + 1
        self.ny = int((self.maxy - self.miny) / self.cell) + 1
        # raw occupancy from geometry
        raw = [[False] * self.ny for _ in range(self.nx)]
        for i in range(self.nx):
            for j in range(self.ny):
                x = self.minx + i * self.cell
                y = self.miny + j * self.cell
                la, lo = _m_to_ll(x, y, self.lat0, self.lon0)
                raw[i][j] = self._blocked_ll(la, lo)
        # dilate blocked cells by clearance for safety margin
        r = max(0, int(math.ceil(clearance_m / self.cell)))
        if r == 0:
            self.grid = raw
            return
        grid = [[False] * self.ny for _ in range(self.nx)]
        for i in range(self.nx):
            for j in range(self.ny):
                if not raw[i][j]:
                    continue
                for di in range(-r, r + 1):
                    for dj in range(-r, r + 1):
                        ii, jj = i + di, j + dj
                        if 0 <= ii < self.nx and 0 <= jj < self.ny:
                            grid[ii][jj] = True
        self.grid = grid

    def _cell_of(self, lat, lon):
        x, y = _ll_to_m(lat, lon, self.lat0, self.lon0)
        i = int(round((x - self.minx) / self.cell))
        j = int(round((y - self.miny) / self.cell))
        i = min(max(i, 0), self.nx - 1)
        j = min(max(j, 0), self.ny - 1)
        return i, j

    def _nearest_free(self, i, j):
        if not self.grid[i][j]:
            return i, j
        for rad in range(1, max(self.nx, self.ny)):
            for di in range(-rad, rad + 1):
                for dj in (-rad, rad):
                    for a, b in ((i + di, j + dj), (i + dj, j + di)):
                        if 0 <= a < self.nx and 0 <= b < self.ny and not self.grid[a][b]:
                            return a, b
        return i, j

    def _astar_cells(self, s, g):
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        openq = [(0.0, s)]
        came, gs = {}, {s: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                path = [cur]
                while cur in came:
                    cur = came[cur]; path.append(cur)
                return path[::-1]
            for dx, dy in nbrs:
                a, b = cur[0] + dx, cur[1] + dy
                if not (0 <= a < self.nx and 0 <= b < self.ny) or self.grid[a][b]:
                    continue
                ng = gs[cur] + (1.414 if dx and dy else 1.0)
                if ng < gs.get((a, b), 1e18):
                    gs[(a, b)] = ng
                    came[(a, b)] = cur
                    h = math.hypot(a - g[0], b - g[1])
                    heapq.heappush(openq, (ng + h, (a, b)))
        return None

    def _line_clear_cells(self, a, b):
        d = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        if d == 0:
            return True
        for k in range(d + 1):
            t = k / d
            i = int(round(a[0] + (b[0] - a[0]) * t))
            j = int(round(a[1] + (b[1] - a[1]) * t))
            if self.grid[i][j]:
                return False
        return True

    def _simplify_cells(self, cells):
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self._line_clear_cells(cells[i], cells[j]):
                j -= 1
            out.append(cells[j]); i = j
        return out

    def _cell_center_ll(self, c):
        x = self.minx + c[0] * self.cell
        y = self.miny + c[1] * self.cell
        return _m_to_ll(x, y, self.lat0, self.lon0)

    def plan(self, start_ll, goal_ll, vias=None):
        legs = [start_ll] + list(vias or []) + [goal_ll]
        if self.grid is None:
            return legs
        out_ll = [start_ll]
        for a, b in zip(legs[:-1], legs[1:]):
            sa = self._nearest_free(*self._cell_of(*a))
            sb = self._nearest_free(*self._cell_of(*b))
            cells = self._astar_cells(sa, sb)
            if cells is None:
                raise RuntimeError(f'no path {a} -> {b} in operating area')
            cells = self._simplify_cells(cells)
            for c in cells[1:]:
                out_ll.append(self._cell_center_ll(c))
        # Only append the exact goal if it is legal (inside the operating area and
        # outside every exclusion). If the goal sits in a no-fly zone the route
        # already ends at the nearest safe cell, so we never fly into the zone.
        if not self._blocked_ll(*goal_ll):
            out_ll.append(goal_ll)
        # drop consecutive near-duplicate waypoints (<5 m apart)
        dedup = [out_ll[0]]
        for p in out_ll[1:]:
            if haversine_m(dedup[-1][0], dedup[-1][1], p[0], p[1]) > 5.0:
                dedup.append(p)
        return dedup

    # ---- public helpers for callers that need no-fly-zone awareness ----
    def is_blocked(self, lat, lon):
        """True if (lat,lon) is outside the operating area or inside an exclusion."""
        return self._blocked_ll(lat, lon)

    def nearest_safe_ll(self, lat, lon):
        """Nearest legal point (outside every no-fly zone + clearance) to (lat,lon)."""
        if self.grid is None:
            return (lat, lon)
        i, j = self._nearest_free(*self._cell_of(lat, lon))
        return self._cell_center_ll((i, j))

    def segment_blocked(self, a, b, samples=80):
        """True if the straight segment a->b passes through any no-fly zone."""
        for k in range(samples + 1):
            t = k / samples
            if self._blocked_ll(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t):
                return True
        return False


def _demo():
    import os
    kml = os.environ.get('MARITIME_KML',
        os.path.expanduser('~/maritime_project/assets/chattogram_zones.kml'))
    m = OperatingMap(kml)
    op = m.operating_polygon('Port Gate', '5km')
    pl = PathPlanner(m, op)
    blocked = sum(pl.grid[i][j] for i in range(pl.nx) for j in range(pl.ny))
    print(f'grid {pl.nx}x{pl.ny} cells @ {pl.cell:.0f}m, blocked={blocked} '
          f'({100*blocked/(pl.nx*pl.ny):.0f}%)')
    start = m.dock('Port Gate')

    def straight_blocked(a, b):
        return any(pl._blocked_ll(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                   for t in [i / 60 for i in range(61)])

    # auto-pick a legal (free) goal on the far side of an exclusion so a
    # straight line would be illegal but a detour exists.
    goal = None
    for frac in (0.9, 0.8, 0.7, 0.6):
        for ang in range(0, 360, 15):
            r = frac * 0.035
            cand = (op.centroid()[0] + r * math.cos(math.radians(ang)),
                    op.centroid()[1] + r * math.sin(math.radians(ang)))
            if not pl._blocked_ll(*cand) and straight_blocked(start, cand):
                goal = cand
                break
        if goal:
            break
    goal = goal or (op.centroid()[0] - 0.02, op.centroid()[1] - 0.02)
    naive_bad = straight_blocked(start, goal)
    path = pl.plan(start, goal)
    bad = sum(pl._blocked_ll(la, lo) for (la, lo) in path)
    print(f'start={start[0]:.5f},{start[1]:.5f}  goal={goal[0]:.5f},{goal[1]:.5f}')
    print(f'straight line crosses a blocked zone: {naive_bad}')
    print(f'planned {len(path)} waypoints, in-blocked-zone: {bad} (want 0)')
    for i, (la, lo) in enumerate(path):
        print(f'  {i}: {la:.5f}, {lo:.5f}')


if __name__ == '__main__':
    _demo()
