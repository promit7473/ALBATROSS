#!/usr/bin/env python3
"""Out-and-back SEA sortie: takeoff at the Port Gate base, fly SW down the
Karnaphuli out over the Bay of Bengal, then RTL back to base and land.

Reuses PatrolNode's flight/mission machinery; only the waypoint list changes.
Run:  ros2 run maritime_autonomy sea_mission   (after mavros is up)
"""
import rclpy
from mavros_msgs.msg import Waypoint
from .patrol_node import PatrolNode, NAV_TAKEOFF, NAV_WAYPOINT, FRAME_GLOBAL_REL_ALT

BASE = (22.318725, 91.813156)      # Port Gate / Dock 1 = spawn = RTL home
ALT = 60.0                          # m rel-alt cruise
# outbound waypoints, SW down the river toward the open sea
ROUTE = [
    (22.30300, 91.80300),          # ~2.5 km  over the river near the port
    (22.26800, 91.78900),          # ~6.5 km  wide estuary
    (22.23800, 91.77600),          # ~10 km   near-shore Bay of Bengal
    (22.21500, 91.76800),          # ~12.5 km open sea  <-- the sea point
]


class SeaMission(PatrolNode):
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
        for la, lo in ROUTE:
            wps.append(wp(NAV_WAYPOINT, la, lo, ALT))
        self.get_logger().info(
            f'SEA sortie: takeoff@base -> {len(ROUTE)} waypoints out to sea -> RTL')
        return wps


def main():
    rclpy.init()
    node = SeaMission()
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
