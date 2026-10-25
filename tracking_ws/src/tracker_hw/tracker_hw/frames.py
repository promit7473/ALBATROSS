"""Frame conversions between PX4 (local NED, body FRD) and ROS / MAVROS (local ENU, body FLU).

The tracking stack works in the MAVROS conventions (see tracker_control.geometry); the Intelis
bridge speaks PX4's. These are the same conversions MAVROS applies internally.
"""
import math

# PX4 flight mode names (as the bridge reports them) -> MAVROS mode strings
PX4_TO_MAVROS_MODE = {
    'Manual': 'MANUAL', 'Altitude': 'ALTCTL', 'Position': 'POSCTL', 'Position Slow': 'POSCTL',
    'Orbit': 'POSCTL', 'Acro': 'ACRO', 'Stabilized': 'STABILIZED', 'Offboard': 'OFFBOARD',
    'Hold': 'AUTO.LOITER', 'Takeoff': 'AUTO.TAKEOFF', 'Mission': 'AUTO.MISSION',
    'Return': 'AUTO.RTL', 'Land': 'AUTO.LAND', 'Follow Target': 'AUTO.FOLLOW_TARGET',
    'Precision Landing': 'AUTO.PRECLAND',
}


def mavros_mode(px4_name):
    if not px4_name:
        return ''
    return PX4_TO_MAVROS_MODE.get(px4_name, px4_name.upper())


def ned_to_enu(x, y, z):
    return y, x, -z


def euler_ned_to_enu_quat(roll, pitch, yaw):
    """PX4 attitude (FRD body in NED, radians) -> quaternion (x, y, z, w) of FLU body in ENU."""
    r, p, y = roll, -pitch, math.pi / 2 - yaw
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def enu_velocity_to_ned(vx, vy, vz, yaw_rate):
    """Velocity + yaw-rate setpoint, ENU / FLU -> NED / FRD."""
    return vy, vx, -vz, -yaw_rate
