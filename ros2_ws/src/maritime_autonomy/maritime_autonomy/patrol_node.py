#!/usr/bin/env python3
"""Maritime autonomy - Phase 1 autonomous skeleton.

Auto-takeoff -> geofenced patrol loop -> RTL, driven over MAVROS against PX4.
This is the mission backbone the maritime behaviours (detect -> divert ->
buoy-drop) will hang off of.  Default coordinates are the PX4 SITL home so it
runs out-of-the-box; override via params for the real Chattogram patrol area.

If a KML path is supplied the patrol generates waypoints inside the KML
operating area and geofences against its exclusion zones.
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import State, Waypoint, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix

from .geo import OperatingMap, point_in_polygon, haversine_m

# MAV_CMD codes
NAV_WAYPOINT = 16
NAV_TAKEOFF = 22
NAV_RTL = 20
FRAME_GLOBAL_REL_ALT = 3

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST)


def meters_to_ll(lat0, lon0, dnorth, deast):
    dlat = dnorth / 111320.0
    dlon = deast / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


class PatrolNode(Node):
    def __init__(self):
        super().__init__('maritime_patrol')
        self.declare_parameter('center_lat', 47.3977508)
        self.declare_parameter('center_lon', 8.5456073)
        self.declare_parameter('altitude', 30.0)
        self.declare_parameter('box_north_m', 80.0)
        self.declare_parameter('box_east_m', 120.0)
        self.declare_parameter('loops', 2)
        self.declare_parameter('kml_path', '')

        self.center_lat = float(self.get_parameter('center_lat').value)
        self.center_lon = float(self.get_parameter('center_lon').value)
        self.alt = float(self.get_parameter('altitude').value)
        self.bn = float(self.get_parameter('box_north_m').value)
        self.be = float(self.get_parameter('box_east_m').value)
        self.loops = int(self.get_parameter('loops').value)
        self.kml_path = str(self.get_parameter('kml_path').value)

        # load KML if provided
        self.op_map = None
        self.op_poly = None
        if self.kml_path and os.path.isfile(self.kml_path):
            self.op_map = OperatingMap(self.kml_path)
            self.op_poly = self.op_map.operating_polygon()
            if self.op_poly:
                c = self.op_poly.centroid()
                self.get_logger().info(
                    f'KML loaded: {len(self.op_map.zones)} zones, '
                    f'operating area centroid=({c[0]:.5f},{c[1]:.5f})')
                if self.center_lat > 47.0:
                    self.center_lat, self.center_lon = c
                    self.get_logger().info(
                        f'center overridden from KML: ({self.center_lat:.5f},'
                        f'{self.center_lon:.5f})')
            else:
                self.get_logger().warn('KML loaded but no operating polygon found')

        # --- state ---
        self.state = State()
        self.last_reached = -1
        self.fix = None
        self.last_wp = 0

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(WaypointReached, '/mavros/mission/reached',
                                 self._reached_cb, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self._fix_cb, SENSOR_QOS)

        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_push = self.create_client(WaypointPush, '/mavros/mission/push')
        self.cli_clear = self.create_client(WaypointClear, '/mavros/mission/clear')

    # ---- callbacks ----
    def _state_cb(self, msg):
        self.state = msg

    def _reached_cb(self, msg):
        self.last_reached = msg.wp_seq

    def _fix_cb(self, msg):
        self.fix = msg

    def _on_reached(self, seq):
        """Hook fired once each time a new mission waypoint is reached.
        Subclasses override (e.g. to drop a payload)."""
        pass

    # ---- helpers ----
    def _call(self, client, req, what):
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for service {what} ...')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        return fut.result()

    def _spin(self, sec):
        end = self.get_clock().now().nanoseconds + int(sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_fcu(self):
        self.get_logger().info('waiting for FCU connection + GPS fix ...')
        while rclpy.ok() and not (self.state.connected and self.fix is not None):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(
            f'FCU connected. patrol center=({self.center_lat:.6f},{self.center_lon:.6f}) '
            f'alt={self.alt}m box={self.bn}x{self.be}m loops={self.loops}')

    def build_mission(self):
        wps = []

        def wp(cmd, lat, lon, alt, current=False):
            w = Waypoint()
            w.frame = FRAME_GLOBAL_REL_ALT
            w.command = cmd
            w.is_current = current
            w.autocontinue = True
            w.param1 = w.param2 = w.param3 = 0.0
            w.param4 = float('nan')
            w.x_lat = lat
            w.y_long = lon
            w.z_alt = alt
            return w

        def _geofence_ok(lat, lon):
            if self.op_map is None:
                return True
            return self.op_map.geofence_ok(lat, lon, self.op_poly)

        wps.append(wp(NAV_TAKEOFF, self.center_lat, self.center_lon, self.alt, current=True))

        hn, he = self.bn / 2.0, self.be / 2.0
        corners = [(hn, he), (hn, -he), (-hn, -he), (-hn, he)]
        for _ in range(self.loops):
            for dn, de in corners:
                la, lo = meters_to_ll(self.center_lat, self.center_lon, dn, de)
                if not _geofence_ok(la, lo):
                    self.get_logger().warn(
                        f'wp ({la:.5f},{lo:.5f}) outside geofence - clamping to center')
                    la, lo = self.center_lat, self.center_lon
                wps.append(wp(NAV_WAYPOINT, la, lo, self.alt))

        return wps

    def run(self):
        self.wait_for_fcu()
        self._spin(2.0)  # let MAVROS plugins settle before mission transfer

        wps = self.build_mission()
        self.last_wp = len(wps) - 1

        self._call(self.cli_clear, WaypointClear.Request(), 'mission/clear')
        self._spin(1.0)

        ok = False
        for attempt in range(1, 4):
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = wps
            res = self._call(self.cli_push, req, 'mission/push')
            ok = bool(getattr(res, 'success', False))
            xfer = getattr(res, 'wp_transfered', 0)
            self.get_logger().info(
                f'mission push attempt {attempt}: success={ok} transfered={xfer}/{len(wps)}')
            if ok:
                break
            self._spin(2.0)
        if not ok:
            self.get_logger().error('mission push failed after retries - aborting')
            return

        self.get_logger().info('setting mode AUTO.MISSION ...')
        self._call(self.cli_mode, SetMode.Request(base_mode=0, custom_mode='AUTO.MISSION'),
                   'set_mode')
        self._spin(1.0)

        self.get_logger().info('arming ...')
        armed = False
        for attempt in range(1, 4):
            ar = self._call(self.cli_arm, CommandBool.Request(value=True), 'arming')
            armed = bool(getattr(ar, 'success', False))
            self.get_logger().info(f'arm attempt {attempt}: success={armed}')
            if armed:
                break
            self._spin(2.0)
        if not armed:
            self.get_logger().error('arming failed after retries - aborting')
            return
        self._spin(1.0)

        self.get_logger().info('PATROLLING ...')
        t0 = time.time()
        mission_seen = False
        prev_reached = -1
        max_patrol_s = 900.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            mode = self.state.mode
            if mode == 'AUTO.MISSION':
                mission_seen = True
            if self.last_reached != prev_reached:
                prev_reached = self.last_reached
                if self.last_reached >= 0:
                    self._on_reached(self.last_reached)
            if self.last_reached >= 0:
                self.get_logger().info(
                    f'reached wp {self.last_reached}/{self.last_wp}  mode={mode}',
                    throttle_duration_sec=3.0)
            # robust completion: all wps reached, OR vehicle finished mission and
            # dropped to loiter/hold, OR safety timeout (never hang).
            if self.last_reached >= self.last_wp:
                self.get_logger().info('all patrol waypoints reached')
                break
            if mission_seen and mode in ('AUTO.LOITER', 'HOLD'):
                self.get_logger().info('mission finished (vehicle loitering)')
                break
            if time.time() - t0 > max_patrol_s:
                self.get_logger().warn('max patrol time reached - ending')
                break

        self.get_logger().info('patrol complete -> RTL (return + land)')
        self._call(self.cli_mode, SetMode.Request(base_mode=0, custom_mode='AUTO.RTL'),
                   'set_mode(RTL)')
        t1 = time.time()
        while rclpy.ok() and time.time() - t1 < 150.0:
            rclpy.spin_once(self, timeout_sec=0.5)
            if not self.state.armed:
                self.get_logger().info('landed + disarmed')
                break
        self.get_logger().info('patrol skeleton done.')


def main():
    rclpy.init()
    node = PatrolNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
