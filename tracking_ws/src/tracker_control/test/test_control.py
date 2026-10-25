import math

import numpy as np

from tracker_control.follow_controller import FollowController, FollowParams
from tracker_control.geometry import CameraModel, quat_to_rot
from tracker_control.target_estimator import TargetEstimator

PITCH = math.radians(40)


def yaw_rot(yaw):
    return quat_to_rot(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))


def test_image_centre_projects_along_camera_axis():
    cam = CameraModel.from_fov(640, 480, 1.2, PITCH)
    (xy, rng) = cam.ground_point(320, 240, [0, 0, 10], yaw_rot(0.0))
    assert np.allclose(xy, [10 / math.tan(PITCH), 0], atol=1e-6)
    assert abs(rng - 10 / math.sin(PITCH)) < 1e-6
    (xy, _) = cam.ground_point(320, 240, [0, 0, 10], yaw_rot(math.pi / 2))
    assert np.allclose(xy, [0, 10 / math.tan(PITCH)], atol=1e-6)


def test_pixel_right_of_centre_is_to_the_right():
    cam = CameraModel.from_fov(640, 480, 1.2, PITCH)
    (xy, _) = cam.ground_point(500, 240, [0, 0, 10], yaw_rot(0.0))
    assert xy[1] < 0                     # right of the drone = -y when facing +x
    assert cam.bearing_error(500) < 0    # turn right


def test_estimator_learns_velocity():
    est = TargetEstimator()
    rng = np.random.default_rng(0)
    for k in range(60):
        t = k * 0.1
        est.update(np.array([1.5 * t, 0.5 * t]) + rng.normal(0, 0.3, 2), 15.0, t)
    _, v = est.state_at(6.0)
    assert np.allclose(v, [1.5, 0.5], atol=0.3)


def test_follow_converges_behind_walking_target():
    fp = FollowParams(altitude=10, camera_pitch=PITCH)
    ctl = FollowController(fp)
    drone = np.array([0.0, 20.0, 10.0])
    yaw = 0.0
    tv = np.array([1.4, 0.0])
    dt = 0.05
    for k in range(int(60 / dt)):
        tp = tv * k * dt
        vx, vy, vz, yr, info = ctl.compute(dt, drone, yaw, tp, tv)
        drone += np.array([vx, vy, vz]) * dt
        yaw += yr * dt
    rel = drone[:2] - tp
    assert abs(np.linalg.norm(rel) - ctl.standoff_distance()) < 0.5
    assert rel[0] < -0.9 * ctl.standoff_distance()          # behind (target walks +x)
    los = math.atan2(-rel[1], -rel[0])
    assert abs(math.remainder(los - yaw, 2 * math.pi)) < math.radians(5)


def test_horizontal_camera():
    cam = CameraModel.from_fov(1280, 720, math.radians(81), 0.0)
    fy = cam.fy
    # feet 10 m ahead of a drone at 3 m: depression atan(3/10) below the optical axis
    v = 360 + fy * 3 / 10
    (xy, _) = cam.ground_point(640, v, [0, 0, 3], yaw_rot(0.0))
    assert np.allclose(xy, [10, 0], atol=1e-6)
    assert cam.ground_point(640, 300, [0, 0, 3], yaw_rot(0.0)) is None     # above the horizon
    # automatic distance is undefined without pitch: fall back instead of dividing by zero
    p = FollowParams(camera_pitch=0.0, distance=0.0)
    assert FollowController(p).standoff_distance() == p.min_distance
    assert FollowController(FollowParams(camera_pitch=0.0, distance=10.0)).standoff_distance() == 10.0
