#!/usr/bin/env python3
"""Task 2 - automatic height adjustment (mission-phase altitude profile).

Flies (via Task-1 exclusion-aware routing) to a target, then automatically
adjusts altitude for the mission phase: transit high -> DESCEND to a low action
altitude over the target -> hold (this is the buoy-drop point for #13) -> climb
back to transit -> RTL. A safe-minimum floor is enforced (relevant over water).

Reuses the hardened PatrolNode run() (upload/arm/AUTO.MISSION/complete/RTL/land).

Params: goal_lat, goal_lon, transit_alt, action_alt, min_alt, hold_s, kml_path.
"""
import rclpy
from mavros_msgs.msg import Waypoint

from sensor_msgs.msg import Range
from .patrol_node import PatrolNode, NAV_TAKEOFF, NAV_WAYPOINT, FRAME_GLOBAL_REL_ALT, SENSOR_QOS, meters_to_ll
from .planner import PathPlanner


class DescendNode(PatrolNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('goal_lat', 0.0)
        self.declare_parameter('goal_lon', 0.0)
        self.declare_parameter('transit_alt', 40.0)
        self.declare_parameter('action_alt', 8.0)
        self.declare_parameter('min_alt', 5.0)     # safe minimum (esp. over water)
        self.declare_parameter('hold_s', 5.0)
        self.goal_lat = float(self.get_parameter('goal_lat').value)
        self.goal_lon = float(self.get_parameter('goal_lon').value)
        self.transit_alt = float(self.get_parameter('transit_alt').value)
        self.min_alt = float(self.get_parameter('min_alt').value)
        self.action_alt = max(float(self.get_parameter('action_alt').value), self.min_alt)
        self.hold_s = float(self.get_parameter('hold_s').value)
        self.range_m = None
        self.create_subscription(Range, '/mavros/distance_sensor/rangefinder', self._range_cb, SENSOR_QOS)
        self.create_subscription(Range, '/mavros/distance_sensor/laser', self._range_cb, SENSOR_QOS)
        self.planner = PathPlanner(self.op_map, self.op_poly) if self.op_map else None

    def _range_cb(self, msg: Range):
        self.range_m = msg.range

    def build_mission(self):
        start = (self.fix.latitude, self.fix.longitude)
        if abs(self.goal_lat) < 1e-4 and abs(self.goal_lon) < 1e-4:
            # Default goal 120m East of start for demonstration / SITL testing
            self.goal_lat, self.goal_lon = meters_to_ll(start[0], start[1], 40.0, 120.0)
            self.get_logger().info(f'no explicit goal provided - using test offset goal ({self.goal_lat:.5f},{self.goal_lon:.5f})')
        goal = (self.goal_lat, self.goal_lon)
        route = self.planner.plan(start, goal) if self.planner else [start, goal]
        self.get_logger().info(
            f'height profile: transit {self.transit_alt:.0f}m -> descend '
            f'{self.action_alt:.0f}m (hold {self.hold_s:.0f}s) -> climb '
            f'{self.transit_alt:.0f}m  (floor {self.min_alt:.0f}m)')

        def wp(cmd, lat, lon, alt, current=False, hold=0.0):
            w = Waypoint()
            w.frame = FRAME_GLOBAL_REL_ALT
            w.command = cmd
            w.is_current = current
            w.autocontinue = True
            w.param1 = hold          # multicopter: time to hold at waypoint
            w.param2 = w.param3 = 0.0
            w.param4 = float('nan')
            w.x_lat = lat
            w.y_long = lon
            w.z_alt = alt
            return w

        wps = [wp(NAV_TAKEOFF, start[0], start[1], self.transit_alt, current=True)]
        for (la, lo) in route[1:]:                       # transit legs (high)
            wps.append(wp(NAV_WAYPOINT, la, lo, self.transit_alt))
        g0, g1 = self.goal_lat, self.goal_lon
        wps.append(wp(NAV_WAYPOINT, g0, g1, self.action_alt))                 # descend
        wps.append(wp(NAV_WAYPOINT, g0, g1, self.action_alt, hold=self.hold_s))  # hold (drop pt)
        wps.append(wp(NAV_WAYPOINT, g0, g1, self.transit_alt))                # climb
        return wps


def main():
    rclpy.init()
    node = DescendNode()
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
