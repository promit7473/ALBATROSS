#!/usr/bin/env python3
"""Task 1 mission node - fly an exclusion-aware planned path.

Reuses the hardened PatrolNode machinery (upload -> arm -> AUTO.MISSION ->
robust completion -> RTL -> land) but replaces the fixed rectangle with a
PathPlanner route from the current position to a goal (optional via-points),
avoiding the KML exclusion zones.

Params: goal_lat, goal_lon, altitude, kml_path (+ via_lats / via_lons lists).
Example:
  ros2 run maritime_autonomy plan_node --ros-args \
    -p kml_path:="/path/Chattogram ... .kml" -p goal_lat:=22.315 -p goal_lon:=91.776
"""
import rclpy
from mavros_msgs.msg import Waypoint

from .patrol_node import PatrolNode, NAV_TAKEOFF, NAV_WAYPOINT, FRAME_GLOBAL_REL_ALT
from .planner import PathPlanner


class PlanNode(PatrolNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('goal_lat', 0.0)
        self.declare_parameter('goal_lon', 0.0)
        self.declare_parameter('via_lats', [])
        self.declare_parameter('via_lons', [])
        self.goal_lat = float(self.get_parameter('goal_lat').value)
        self.goal_lon = float(self.get_parameter('goal_lon').value)
        vlats = list(self.get_parameter('via_lats').value or [])
        vlons = list(self.get_parameter('via_lons').value or [])
        self.vias = list(zip(vlats, vlons))
        self.planner = PathPlanner(self.op_map, self.op_poly) if self.op_map else None

    def build_mission(self):
        start = (self.fix.latitude, self.fix.longitude)
        goal = (self.goal_lat, self.goal_lon)
        if self.planner is not None:
            pts = self.planner.plan(start, goal, self.vias)
            self.get_logger().info(
                f'planned {len(pts)} exclusion-aware waypoints '
                f'({start[0]:.5f},{start[1]:.5f}) -> ({goal[0]:.5f},{goal[1]:.5f})')
        else:
            pts = [start] + self.vias + [goal]
            self.get_logger().warn('no KML map - flying straight legs (no avoidance)')

        def wp(cmd, lat, lon, current=False):
            w = Waypoint()
            w.frame = FRAME_GLOBAL_REL_ALT
            w.command = cmd
            w.is_current = current
            w.autocontinue = True
            w.param1 = w.param2 = w.param3 = 0.0
            w.param4 = float('nan')
            w.x_lat = lat
            w.y_long = lon
            w.z_alt = self.alt
            return w

        wps = [wp(NAV_TAKEOFF, start[0], start[1], current=True)]
        for (la, lo) in pts[1:]:  # skip start (already the takeoff point)
            wps.append(wp(NAV_WAYPOINT, la, lo))
        return wps


def main():
    rclpy.init()
    node = PlanNode()
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
