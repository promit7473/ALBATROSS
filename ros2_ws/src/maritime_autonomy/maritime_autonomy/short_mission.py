#!/usr/bin/env python3
"""Short waypoint circuit near the base: takeoff, fly a compact turning route
(a few waypoints) around the Port Gate, then RTL back to base and land.
Stays within ~300 m of the spawn so the close chase-cam keeps it framed.

Run:  python3 -m maritime_autonomy.short_mission   (after mavros is up)
"""
import rclpy
from mavros_msgs.msg import Waypoint
from .patrol_node import (PatrolNode, NAV_TAKEOFF, NAV_WAYPOINT,
                          FRAME_GLOBAL_REL_ALT, meters_to_ll)

BASE = (22.318725, 91.813156)      # Port Gate = spawn = RTL home
ALT = 35.0                          # m rel-alt
# turning route as (dNorth_m, dEast_m) offsets from base
LEGS = [
    (250, 0),      # N
    (250, 300),    # turn E
    (-50, 300),    # turn S
    (-50, -100),   # turn W
    (150, -100),   # turn N (heading home)
]


class ShortMission(PatrolNode):
    def build_mission(self):
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

        wps = [wp(NAV_TAKEOFF, BASE[0], BASE[1], ALT, current=True)]
        for dn, de in LEGS:
            la, lo = meters_to_ll(BASE[0], BASE[1], dn, de)
            wps.append(wp(NAV_WAYPOINT, la, lo, ALT))
        self.get_logger().info(
            f'SHORT circuit: takeoff@base -> {len(LEGS)} turning waypoints -> RTL')
        return wps


def main():
    rclpy.init()
    node = ShortMission()
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
