"""Drives the `traffic_*` cars of the city world around their routes.

Cars are static models moved kinematically through the world's
`set_pose_vector` service, so they need no physics tuning. Behaviour:
  * cruise at the configured speed along a rounded-corner loop,
  * keep a gap to the car ahead on the same route (simple car-following),
  * yield at junctions: do not enter a junction box another car occupies,
  * acceleration-limited speed changes, so stops and starts look natural.
Routes, speeds and names come from config/city.yaml (same file as the world).
"""
import math
import os

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import gz.transport13 as gzt  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from rclpy.node import Node  # noqa: E402

from tracker_sim.paths import Route, rounded_loop  # noqa: E402

FOLLOW_GAP = 9.0        # [m] start slowing when the car ahead is closer than this
STOP_GAP = 6.0          # [m] full stop at this gap
LOOKAHEAD = 7.0         # [m] how far ahead a car checks for junction occupancy
ACCEL = 2.5             # [m/s^2]


class Vehicle:
    def __init__(self, name, model, route_name, route, cruise, s0, heading_offset):
        self.name, self.model, self.route_name, self.route = name, model, route_name, route
        self.cruise, self.s, self.v = cruise, s0, 0.0
        self.heading_offset = heading_offset

    def xy(self, ahead=0.0):
        x, y, _ = self.route.pose_at(self.s + ahead)
        return x, y


class TrafficNode(Node):
    def __init__(self):
        super().__init__('traffic_node')
        default_cfg = os.path.join(get_package_share_directory('tracker_sim'), 'config', 'city.yaml')
        self.declare_parameter('city_config', default_cfg)
        with open(self.get_parameter('city_config').value) as f:
            cfg = yaml.safe_load(f)
        t = cfg['traffic']
        self.world = cfg['world_name']
        self.junction_r = t['intersection_radius']
        offsets = cfg.get('vehicle_heading_offset', {})

        routes = {k: Route(rounded_loop(v, t['corner_radius'])) for k, v in t['routes'].items()}
        self.vehicles = [
            Vehicle(f'traffic_{i}', model, rname, routes[rname], speed, frac * routes[rname].length,
                    math.radians(offsets.get(model, 0.0)))
            for i, (model, rname, speed, frac) in enumerate(t['vehicles'])]
        # Junctions = crossings of road centre lines.
        roads = [s for s in cfg['surfaces'] if s[4] == 'asphalt']
        xs = sorted({r[0] for r in roads if r[3] > r[2]})
        ys = sorted({r[1] for r in roads if r[2] > r[3]})
        self.junctions = [(x, y) for x in xs for y in ys]

        self.gz = gzt.Node()
        self.service = f'/world/{self.world}/set_pose_vector'
        self.dt = 1.0 / t['rate_hz']
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f'{len(self.vehicles)} vehicles on {len(routes)} routes')

    def junction_of(self, x, y):
        for j, (jx, jy) in enumerate(self.junctions):
            if math.hypot(x - jx, y - jy) < self.junction_r:
                return j
        return None

    def target_speed(self, car):
        v = car.cruise
        # car following on the same loop
        for other in self.vehicles:
            if other is car or other.route_name != car.route_name:
                continue
            gap = (other.s - car.s) % car.route.length
            if gap < FOLLOW_GAP:
                v = min(v, car.cruise * max(0.0, (gap - STOP_GAP) / (FOLLOW_GAP - STOP_GAP)))
        # junction yielding: only cars outside a junction must wait
        if self.junction_of(*car.xy()) is None:
            j = self.junction_of(*car.xy(LOOKAHEAD))
            if j is not None and any(o is not car and self.junction_of(*o.xy()) == j
                                     for o in self.vehicles):
                v = 0.0
        return v

    def step(self):
        targets = [self.target_speed(c) for c in self.vehicles]
        req = Pose_V()
        for car, v_t in zip(self.vehicles, targets):
            dv = max(-ACCEL * self.dt, min(ACCEL * self.dt, v_t - car.v))
            car.v = max(0.0, car.v + dv)
            car.s = (car.s + car.v * self.dt) % car.route.length
            x, y, yaw = car.route.pose_at(car.s)
            yaw += car.heading_offset
            p = req.pose.add()
            p.name = car.name
            p.position.x, p.position.y, p.position.z = x, y, 0.0
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.gz.request(self.service, req, Pose_V, Boolean, 200)


def main():
    rclpy.init()
    node = TrafficNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
