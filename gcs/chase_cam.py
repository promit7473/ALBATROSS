#!/usr/bin/env python3
"""Chase (third-person follow) camera for the Gazebo sim.

Spawns an invisible camera model and repositions it behind+above the drone every
tick so the web feed shows a smooth follow view (works headless / under Wayland,
unlike the GUI /gui/track cam which can't be captured).

Stability design (v2 -- kills the earlier oscillation):
  * Camera ORIENTATION always LOOKS AT the drone, so the drone stays centred no
    matter how noisy the follow heading is. Framing is decoupled from heading.
  * Camera POSITION eases toward its target each tick (exponential smoothing) ->
    no jitter.
  * Follow heading (where "behind" is) comes ONLY from the high-rate dynamic_pose
    topic, heavily smoothed, and is HELD when the drone is slow (no snap to body
    yaw that caused swinging). pose/info is used only to hold position when idle.

Pure gz-transport (no ROS). Set PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python.
"""
import math
import os
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
from gz.msgs10 import boolean_pb2, entity_pb2, entity_factory_pb2, pose_pb2, pose_v_pb2
from gz.transport13 import Node

WORLD = os.environ.get("GZ_WORLD", "chattogram")
DRONE = os.environ.get("DRONE_NAME", "x500_0")
MODE = os.environ.get("CAM_MODE", "chase")             # "chase", "topdown" or "fpv"
# fpv reuses the topdown_cam world model (2nd feed slot), it's just aimed forward.
MODEL = os.environ.get("CAM_MODEL",
                       "topdown_cam" if MODE in ("topdown", "fpv") else "chase_cam")
CAM_TOPIC = os.environ.get("CAM_TOPIC", "/" + MODEL)
DIST = float(os.environ.get("CHASE_DIST", "2.8"))      # metres behind the drone (chase)
HEIGHT = float(os.environ.get("CHASE_HEIGHT", "1.2"))  # metres above the drone (chase)
TD_HEIGHT = float(os.environ.get("TOPDOWN_HEIGHT", "45.0"))  # metres above drone (nadir)
TD_YAW = float(os.environ.get("TOPDOWN_YAW", "0.0"))        # rad; 0 = north-up
# FPV / onboard-style angled view: sit above+behind the drone and look FORWARD
# along travel heading, tilted down FPV_PITCH_DEG. Paired with the wide (~115 deg)
# lens on topdown_cam, this puts the horizon in the upper frame and lots of
# ground ahead -- a nose-mounted drone-cam look. Raise FPV_PITCH_DEG toward 45
# for a steeper "looking down the nose" feel.
FPV_PITCH = math.radians(float(os.environ.get("FPV_PITCH_DEG", "28.0")))  # downtilt from horizon
FPV_BACK = float(os.environ.get("FPV_BACK", "1.2"))   # metres behind the drone
FPV_UP = float(os.environ.get("FPV_UP", "1.3"))       # metres above the drone (keeps drone out of frame)
FPV_YAW0 = float(os.environ.get("FPV_YAW0", "0.0"))   # rad; heading to use while idle/hovering
UPDATE_HZ = float(os.environ.get("CHASE_HZ", "30"))
VEL_MIN = float(os.environ.get("CHASE_VEL_MIN", "1.0"))    # m/s to trust travel heading
# Heavier low-pass filtering to kill the high-frequency follow-cam jitter/vibration:
# smaller = smoother (but slightly laggier). Position + camera easing are the two
# knobs that most affect visible shake, since the look-at angle is derived from them.
# Keep POSITION tracking responsive so the camera stays CLOSE to a moving drone
# (too much position/ease smoothing makes the cam lag -> drone drifts far ahead).
# The anti-jitter work is done mostly by ORI_SMOOTH (look-at angle low-pass), which
# removes visible shake WITHOUT adding positional lag.
POS_SMOOTH = float(os.environ.get("CHASE_POS_SMOOTH", "0.22"))  # drone-pos low-pass
CAM_EASE = float(os.environ.get("CHASE_CAM_EASE", "0.20"))      # camera-pos easing/tick
HEAD_SMOOTH = float(os.environ.get("CHASE_HEAD_SMOOTH", "0.06"))  # heading low-pass
ORI_SMOOTH = float(os.environ.get("CHASE_ORI_SMOOTH", "0.16"))   # look-at angle low-pass (anti-jitter)

CAM_SDF = f"""<sdf version="1.9">
  <model name="{MODEL}">
    <static>true</static>
    <pose>0 0 5 0 0 0</pose>
    <link name="link">
      <gravity>false</gravity>
      <inertial><mass>0.01</mass>
        <inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <sensor name="cam" type="camera">
        <topic>{CAM_TOPIC}</topic>
        <update_rate>30</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>1.25</horizontal_fov>
          <image><width>1280</width><height>720</height></image>
          <clip><near>0.1</near><far>20000</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""


def rpy_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (cr * cp * cy + sr * sp * sy,   # w
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy)   # z


class ChaseCam:
    def __init__(self):
        self.node = Node()
        self.msg = pose_pb2.Pose()
        self.msg.name = MODEL
        self.set_pose_srv = f"/world/{WORLD}/set_pose"
        self.lock = threading.Lock()
        self.dpos = None           # smoothed drone position [x,y,z]
        self.head = None           # smoothed travel-heading unit vec (hx,hy)
        self.cam = None            # eased camera position [x,y,z]
        self.prev = None           # (t,x,y) for velocity (dynamic topic only)
        self.last_dyn = 0.0
        self.cyaw = None           # smoothed look-at yaw (anti-jitter)
        self.cpitch = None         # smoothed look-at pitch

    def remove(self):
        """Remove any existing camera model so we can respawn cleanly (e.g. as static)."""
        req = entity_pb2.Entity()
        req.name = MODEL
        req.type = entity_pb2.Entity.MODEL
        try:
            self.node.request(f"/world/{WORLD}/remove", req, entity_pb2.Entity,
                              boolean_pb2.Boolean, 3000)
        except Exception:
            pass
        time.sleep(0.6)

    def spawn(self):
        req = entity_factory_pb2.EntityFactory()
        req.sdf = CAM_SDF
        req.name = MODEL
        req.allow_renaming = False
        ok, res = self.node.request(f"/world/{WORLD}/create", req,
                                    entity_factory_pb2.EntityFactory,
                                    boolean_pb2.Boolean, 5000)
        print(f"[{MODEL}] spawn {'OK' if (ok and res.data) else 'FAILED'}", flush=True)
        return ok and res.data

    def _find(self, msg):
        for p in msg.pose:
            if p.name == DRONE:
                return p
        for p in msg.pose:
            if p.name.startswith("x500") and "/" not in p.name:
                return p
        return None

    def on_poses(self, msg, dynamic):
        p = self._find(msg)
        if p is None:
            return
        now = time.monotonic()
        px, py, pz = p.position.x, p.position.y, p.position.z
        with self.lock:
            # position: pose/info only refreshes when dynamic_pose is quiet (idle)
            if dynamic or (now - self.last_dyn) > 0.5:
                if self.dpos is None:
                    self.dpos = [px, py, pz]
                else:
                    a = POS_SMOOTH
                    self.dpos[0] += a * (px - self.dpos[0])
                    self.dpos[1] += a * (py - self.dpos[1])
                    self.dpos[2] += a * (pz - self.dpos[2])
            if dynamic:
                self.last_dyn = now
                if self.prev is not None:
                    dt = now - self.prev[0]
                    if dt > 0.03:
                        vx = (px - self.prev[1]) / dt
                        vy = (py - self.prev[2]) / dt
                        if math.hypot(vx, vy) >= VEL_MIN:
                            n = math.hypot(vx, vy)
                            hx, hy = vx / n, vy / n
                            if self.head is None:
                                self.head = [hx, hy]
                            else:
                                self.head[0] += HEAD_SMOOTH * (hx - self.head[0])
                                self.head[1] += HEAD_SMOOTH * (hy - self.head[1])
                        self.prev = (now, px, py)
                else:
                    self.prev = (now, px, py)

    def tick(self):
        with self.lock:
            d = list(self.dpos) if self.dpos else None
            head = list(self.head) if self.head else None
            cam = list(self.cam) if self.cam else None
        if d is None:
            return
        if MODE == "topdown":
            # sit directly above the drone, looking straight down (nadir)
            tx, ty, tz = d[0], d[1], d[2] + TD_HEIGHT
            if cam is None:
                cam = [tx, ty, tz]
            else:
                cam[0] += CAM_EASE * (tx - cam[0])
                cam[1] += CAM_EASE * (ty - cam[1])
                cam[2] += CAM_EASE * (tz - cam[2])
            qw, qx, qy, qz = rpy_to_quat(0.0, math.pi / 2, TD_YAW)  # +X -> straight down
        elif MODE == "fpv":
            # onboard-style angled view: look FORWARD along the (smoothed) travel
            # heading, tilted down FPV_PITCH. Drone sits low in frame, horizon up
            # top -> matches a nose-mounted FPV camera rather than a nadir view.
            if head is not None and math.hypot(*head) > 1e-3:
                yaw = math.atan2(head[1], head[0])
            elif self.cyaw is not None:
                yaw = self.cyaw                      # hover: hold last heading
            else:
                yaw = FPV_YAW0
            # low-pass the heading (shortest-path wrap) to kill yaw jitter
            if self.cyaw is None:
                self.cyaw = yaw
            else:
                dyaw = (yaw - self.cyaw + math.pi) % (2 * math.pi) - math.pi
                self.cyaw += ORI_SMOOTH * dyaw
            yaw = self.cyaw
            tx = d[0] - FPV_BACK * math.cos(yaw)
            ty = d[1] - FPV_BACK * math.sin(yaw)
            tz = d[2] + FPV_UP
            if cam is None:
                cam = [tx, ty, tz]
            else:
                cam[0] += CAM_EASE * (tx - cam[0])
                cam[1] += CAM_EASE * (ty - cam[1])
                cam[2] += CAM_EASE * (tz - cam[2])
            qw, qx, qy, qz = rpy_to_quat(0.0, FPV_PITCH, yaw)  # forward + down-tilt
        else:
            # chase: sit behind the smoothed travel heading, else hold the spot
            if head is not None and math.hypot(*head) > 1e-3:
                yaw = math.atan2(head[1], head[0])
            else:
                yaw = None
            if yaw is not None:
                tx = d[0] - DIST * math.cos(yaw)
                ty = d[1] - DIST * math.sin(yaw)
            elif cam is not None:
                tx, ty = cam[0], cam[1]            # hover: hold horizontal spot
            else:
                tx, ty = d[0] - DIST, d[1]
            tz = d[2] + HEIGHT
            if cam is None:
                cam = [tx, ty, tz]
            else:
                cam[0] += CAM_EASE * (tx - cam[0])
                cam[1] += CAM_EASE * (ty - cam[1])
                cam[2] += CAM_EASE * (tz - cam[2])
            # orientation: LOOK AT the drone -> always centred, immune to heading noise
            lx, ly, lz = d[0] - cam[0], d[1] - cam[1], d[2] - cam[2]
            h = math.hypot(lx, ly)
            tyaw = math.atan2(ly, lx)
            tpitch = math.atan2(-lz, h)            # negative lz (drone below) -> look down
            # low-pass the look-at angles to kill residual angular jitter (esp. up close)
            if self.cyaw is None:
                self.cyaw, self.cpitch = tyaw, tpitch
            else:
                dyaw = (tyaw - self.cyaw + math.pi) % (2 * math.pi) - math.pi   # shortest wrap
                self.cyaw += ORI_SMOOTH * dyaw
                self.cpitch += ORI_SMOOTH * (tpitch - self.cpitch)
            qw, qx, qy, qz = rpy_to_quat(0.0, self.cpitch, self.cyaw)
        with self.lock:
            self.cam = cam
        self.msg.position.x, self.msg.position.y, self.msg.position.z = cam
        self.msg.orientation.w, self.msg.orientation.x = qw, qx
        self.msg.orientation.y, self.msg.orientation.z = qy, qz
        try:
            self.node.request(self.set_pose_srv, self.msg, pose_pb2.Pose,
                              boolean_pb2.Boolean, 50)
        except Exception:
            pass

    def run(self):
        # The camera model is defined in the world SDF at startup (see chattogram.sdf)
        # -- do NOT spawn it at runtime: gz's Sensors render thread only renders
        # sensors that exist when it initialises, so a runtime-spawned camera never
        # publishes. We just reposition the existing model each tick.
        print(f"[{MODEL}] using world-startup camera model (no runtime spawn)", flush=True)
        self.node.subscribe(pose_v_pb2.Pose_V, f"/world/{WORLD}/dynamic_pose/info",
                            lambda m: self.on_poses(m, True))
        self.node.subscribe(pose_v_pb2.Pose_V, f"/world/{WORLD}/pose/info",
                            lambda m: self.on_poses(m, False))
        print(f"[{MODEL}] {MODE} follow of '{DRONE}', streaming {CAM_TOPIC}", flush=True)
        period = 1.0 / UPDATE_HZ
        while True:
            self.tick()
            time.sleep(period)


if __name__ == "__main__":
    ChaseCam().run()
