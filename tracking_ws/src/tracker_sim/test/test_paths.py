import math

from tracker_sim.paths import Route, rounded_loop, walking_waypoints


def test_rounded_square_is_shorter_than_square_and_closed():
    sq = [(0, 0), (100, 0), (100, 100), (0, 100)]
    r = Route(rounded_loop(sq, 7.0))
    # each fillet replaces 2r of straight line with a quarter circle of radius r
    expected = 400 - 4 * (2 * 7 - math.pi * 7 / 2)
    assert abs(r.length - expected) < 0.5
    x0, y0, _ = r.pose_at(0.0)
    x1, y1, _ = r.pose_at(r.length)
    assert math.hypot(x1 - x0, y1 - y0) < 1e-6


def test_route_heading_follows_segments():
    r = Route([(0, 0), (10, 0), (10, 10)], closed=False)
    assert abs(r.pose_at(5)[2]) < 1e-9
    assert abs(r.pose_at(15)[2] - math.pi / 2) < 1e-9


def test_walking_waypoints_time_monotonic_and_speed():
    wps = walking_waypoints([(0, 0), (10, 0), (10, 10)], speed=1.0, mode='pingpong')
    ts = [w[0] for w in wps]
    assert all(b >= a for a, b in zip(ts, ts[1:]))
    # 40 m walked at 1 m/s plus turning time
    assert 40.0 < ts[-1] < 45.0
    assert wps[-1][1:3] == (0, 0)
