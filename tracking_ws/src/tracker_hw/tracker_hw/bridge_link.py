"""Intelis telemetry bridge (WebSocket) -> the MAVROS interface the follow node uses.

On the real drone the flight controller is owned by the Intelis bridge (telem_bridge.py) next
to it; this node signs in as operator and stands in for MAVROS, so follow_node runs unchanged:

  Publishes   /mavros/state                 connected / armed / mode (from bridge 'state')
              /mavros/local_position/pose   ENU pose (from bridge 'pose', LOCAL_POSITION_NED + ATTITUDE)
  Subscribes  /mavros/setpoint_raw/local    velocity + yaw-rate setpoints -> bridge 'track' (NED)
  Services    /mavros/set_mode              OFFBOARD -> track_offboard, AUTO.LOITER -> track_release,
                                            AUTO.LAND -> land, AUTO.RTL -> rtl
              /mavros/cmd/arming            only with allow_arm:=true (default: arm from the dashboard)

The bridge enforces its own limits and watchdog (hover, then Hold, when setpoints stop), so a
dropped link from this computer cannot leave the drone flying on a stale command.
The operator key is read from the environment (GCS_TOKEN), never from parameters.
"""
import asyncio
import json
import os
import ssl
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import websockets

from tracker_hw.frames import enu_velocity_to_ned, euler_ned_to_enu_quat, mavros_mode, ned_to_enu

SET_MODE_CMDS = {'OFFBOARD': 'track_offboard', 'AUTO.LOITER': 'track_release',
                 'AUTO.LAND': 'land', 'AUTO.RTL': 'rtl'}


class BridgeLink(Node):
    def __init__(self):
        super().__init__('bridge_link')
        p = self.declare_parameters('', [
            ('url', 'wss://kibo-pc.taildedc36.ts.net:10000/'),
            ('token_env', 'GCS_TOKEN'),
            ('allow_arm', False),
            ('stale_after', 3.0),               # [s] heartbeat age -> reported as disconnected
        ])
        self.prm = {x.name: x.value for x in p}
        self.token = os.environ.get(self.prm['token_env'], '')
        if not self.token:
            self.get_logger().error(f'${self.prm["token_env"]} is not set: cannot sign in to the bridge')

        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', qos_profile_sensor_data)
        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self.on_setpoint, 10)
        self.create_service(SetMode, '/mavros/set_mode', self.on_set_mode)
        self.create_service(CommandBool, '/mavros/cmd/arming', self.on_arming)

        self.vehicle = {}                       # last bridge 'state'
        self.vehicle_at = 0.0
        self.authed = False
        self.track_ok = False                   # bridge advertises the 'track' feature
        self.ws, self.loop = None, None
        self.last_warn = {}
        self.create_timer(0.1, self.publish_state)
        threading.Thread(target=self._ws_thread, daemon=True).start()

    # ------------------------------------------------------------------ ROS side

    def on_setpoint(self, sp):
        if sp.type_mask & PositionTarget.IGNORE_VX:
            self._warn_once('mask', 'only velocity setpoints are forwarded to the bridge')
            return
        vx, vy, vz, wz = enu_velocity_to_ned(sp.velocity.x, sp.velocity.y, sp.velocity.z, sp.yaw_rate)
        self._send({'cmd': 'track', 'vx': vx, 'vy': vy, 'vz': vz, 'yaw_rate': wz})

    def on_set_mode(self, req, res):
        cmd = SET_MODE_CMDS.get(req.custom_mode)
        if cmd is None:
            self._warn_once('mode', f'mode {req.custom_mode} is not supported through the bridge')
            res.mode_sent = False
        elif cmd == 'track_offboard' and not self.vehicle.get('flying'):
            self._warn_once('offboard', 'Offboard refused: take off from the dashboard first, then engage')
            res.mode_sent = False
        else:
            res.mode_sent = self._send({'cmd': cmd})
        return res

    def on_arming(self, req, res):
        if not self.prm['allow_arm']:
            self._warn_once('arm', 'arming from the tracker is disabled (allow_arm:=false): arm from the dashboard')
            res.success = False
        else:
            res.success = self._send({'cmd': 'arm' if req.value else 'disarm'})
        return res

    def publish_state(self):
        v = self.vehicle
        s = State()
        s.header.stamp = self.get_clock().now().to_msg()
        hb_age = v.get('hb_age')
        s.connected = (self.authed and (v.get('link') or {}).get('state') == 'connected'
                       and hb_age is not None and hb_age < self.prm['stale_after']
                       and time.monotonic() - self.vehicle_at < self.prm['stale_after'])
        s.armed = bool(v.get('armed'))
        s.mode = mavros_mode(v.get('mode'))
        s.guided = s.mode.startswith('AUTO') or s.mode == 'OFFBOARD'
        s.manual_input = bool(v.get('rc'))
        self.state_pub.publish(s)

    def on_pose(self, m):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()      # arrival, same clock as camera frames
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = ned_to_enu(m['x'], m['y'], m['z'])
        q = euler_ned_to_enu_quat(m['roll'], m['pitch'], m['yaw'])
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        self.pose_pub.publish(ps)

    def _warn_once(self, key, text, every=5.0):
        now = time.monotonic()
        if now - self.last_warn.get(key, -every) >= every:
            self.last_warn[key] = now
            self.get_logger().warn(text)

    # ------------------------------------------------------------------ WebSocket side

    def _send(self, obj):
        if not self.authed or self.ws is None:
            return False
        if obj['cmd'].startswith('track') and not self.track_ok:
            return False                        # an old bridge would answer every setpoint with a warning
        asyncio.run_coroutine_threadsafe(self._send_async(json.dumps(obj)), self.loop)
        return True

    async def _send_async(self, text):
        try:
            await self.ws.send(text)
        except Exception:
            pass                                # the receive loop notices and reconnects

    def _ws_thread(self):
        self.loop = asyncio.new_event_loop()
        self.loop.run_until_complete(self._ws_main())

    async def _ws_main(self):
        url = self.prm['url']
        ctx = ssl.create_default_context() if url.startswith('wss') else None
        while rclpy.ok():
            try:
                async with websockets.connect(url, ssl=ctx, open_timeout=10, ping_interval=5,
                                              ping_timeout=5, max_size=2 ** 20) as ws:
                    self.ws = ws
                    self.get_logger().info(f'connected to bridge {url}')
                    await ws.send(json.dumps({'cmd': 'auth', 'token': self.token}))
                    async for raw in ws:
                        self._on_message(json.loads(raw))
            except Exception as e:
                self._warn_once('ws', f'bridge connection: {e!r}; retrying')
            self.ws, self.authed, self.track_ok = None, False, False
            await asyncio.sleep(2.0)

    def _on_message(self, m):
        t = m.get('type')
        if t == 'pose':
            self.on_pose(m)
        elif t == 'state':
            self.vehicle, self.vehicle_at = m, time.monotonic()
        elif t == 'auth':
            was, self.authed = self.authed, bool(m.get('ok'))
            if self.authed and not was:
                self.get_logger().info('signed in as operator')
                if self.track_ok:
                    asyncio.ensure_future(self.ws.send(json.dumps({'cmd': 'track_subscribe'})))
            elif m.get('attempt') and not self.authed:
                self.get_logger().error('bridge rejected the operator key')
        elif t == 'hello':
            self.track_ok = 'track' in m.get('features', [])
            if not self.track_ok:
                self.get_logger().error(f'bridge {m.get("version")} has no tracking support: update telem_bridge.py '
                                        '(video + detection work; the drone will not be commanded)')
            else:
                self.get_logger().info(f'bridge version {m.get("version")}')
        elif t == 'text':
            log = self.get_logger().warn if m.get('level') in ('warn', 'critical') else self.get_logger().info
            log(f'[drone] {m.get("text")}')


def main():
    rclpy.init()
    node = BridgeLink()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
