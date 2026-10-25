import math

import numpy as np

from tracker_control.geometry import quat_to_rot
from tracker_hw.frames import enu_velocity_to_ned, euler_ned_to_enu_quat, mavros_mode, ned_to_enu

NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def rot_ned_frd(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def test_attitude_matches_full_frame_change():
    for roll, pitch, yaw in [(0, 0, 0), (0.1, -0.2, 0.3), (-0.3, 0.25, 2.5), (0.05, 0.4, -1.9)]:
        expected = NED_TO_ENU @ rot_ned_frd(roll, pitch, yaw) @ FRD_TO_FLU
        assert np.allclose(quat_to_rot(*euler_ned_to_enu_quat(roll, pitch, yaw)), expected, atol=1e-9)


def test_facing_north_is_enu_yaw_ninety():
    R = quat_to_rot(*euler_ned_to_enu_quat(0.0, 0.0, 0.0))
    assert np.allclose(R[:, 0], [0, 1, 0], atol=1e-9)      # nose points north = +y in ENU


def test_position_and_velocity():
    assert ned_to_enu(1.0, 2.0, -10.0) == (2.0, 1.0, 10.0)
    # ENU: 1 m/s east, climbing 0.5 m/s, turning left (CCW) -> NED: east, up = -z, yaw rate negative
    assert enu_velocity_to_ned(1.0, 0.0, 0.5, 0.2) == (0.0, 1.0, -0.5, -0.2)


def test_mode_names():
    assert mavros_mode('Offboard') == 'OFFBOARD'
    assert mavros_mode('Hold') == 'AUTO.LOITER'
    assert mavros_mode(None) == ''
