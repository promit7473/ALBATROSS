#!/usr/bin/env python3
"""Maritime GCS backend (slice 1: live ops map).

Serves the web UI, proxies live telemetry from the FC interface, and exposes the
KML operating area + exclusion zones as GeoJSON. No web framework -- stdlib only.
"""
import json
import os
import subprocess
import sys
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Gazebo feed now comes from cam_bridge.py (real onboard camera render -> MJPEG),
# which works under Wayland/Xorg/headless. We just proxy it so the UI stays on one port.
CAM_BRIDGE_URL = os.environ.get("CAM_BRIDGE_URL", "http://127.0.0.1:8091/video.mjpeg")

HERE = os.path.dirname(os.path.abspath(__file__))              # <project-root>/gcs
ROOT = os.environ.get("MARITIME_ROOT", os.path.dirname(HERE))  # <project-root> (relocatable)
WEB = os.path.join(HERE, "web")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws/src/maritime_autonomy/maritime_autonomy"))

import geo  # noqa: E402  (OperatingMap/Zone from the autonomy package)
import planner as planner_mod  # noqa: E402  (PathPlanner A* over the operating area)
from fc_iface import MavlinkRestFC  # noqa: E402

KML = os.path.join(ROOT, "assets/chattogram_zones.kml")
BASE = {"name": "Base / Port Gate", "lat": 22.318725, "lon": 91.813156}
OP_FILE = os.path.join(HERE, "operating_area.json")   # user-drawn boundary persistence
FC = MavlinkRestFC()


def load_op_area():
    try:
        with open(OP_FILE) as f:
            return json.load(f)
    except Exception:
        return {"polygon": None}


def save_op_area(poly):
    with open(OP_FILE, "w") as f:
        json.dump({"polygon": poly}, f)

# ---- build KML GeoJSON once at startup ----
def build_geojson():
    om = geo.OperatingMap(KML)
    feats = []
    op = om.operating_polygon("Port Gate", "5km") or om.operating_polygon()
    if op:
        feats.append({"type": "Feature", "properties": {"kind": "operating", "name": op.name},
                      "geometry": {"type": "Polygon",
                                   "coordinates": [[[lo, la] for la, lo in op.pts]]}})
    for z in om.exclusions():
        if getattr(z, "pts", None) and len(z.pts) >= 3:
            feats.append({"type": "Feature",
                          "properties": {"kind": "exclusion", "name": z.name},
                          "geometry": {"type": "Polygon",
                                       "coordinates": [[[lo, la] for la, lo in z.pts]]}})
    return {"type": "FeatureCollection", "base": BASE, "features": feats}

GEOJSON = build_geojson()
print(f"[gcs] KML loaded: {len(GEOJSON['features'])} features "
      f"({sum(1 for f in GEOJSON['features'] if f['properties']['kind']=='exclusion')} exclusions)")

OM = geo.OperatingMap(KML)   # shared, for exclusions


def current_op_zone():
    """Operating boundary = user-drawn (saved) polygon, else the KML default."""
    saved = load_op_area().get("polygon")
    if saved and len(saved) >= 3:
        return geo.Zone("operating (drawn)", "polygon", [(p[0], p[1]) for p in saved])
    return OM.operating_polygon("Port Gate", "5km") or OM.operating_polygon()


_PLANNER = {"sig": None, "pp": None}


def get_planner():
    """Build (and cache) the A* planner; rebuild only when the operating zone changes."""
    op = current_op_zone()
    sig = tuple(op.pts) if op else None
    if _PLANNER["sig"] != sig:
        _PLANNER["pp"] = planner_mod.PathPlanner(OM, op_poly=op, cell_m=150.0, clearance_m=200.0)
        _PLANNER["sig"] = sig
    return _PLANNER["pp"]


def plan_route(start, goal, vias, alt):
    """Single-goal exclusion-aware route. If the goal is in a no-fly zone the route
    ends at the nearest safe point (never inside the zone) and 'clamped' is True."""
    pp = get_planner()
    goal_blocked = pp.is_blocked(goal[0], goal[1])
    route = pp.plan(tuple(start), tuple(goal), [tuple(v) for v in (vias or [])])
    r = [[round(la, 7), round(lo, 7)] for la, lo in route]
    return {"route": r, "clamped": bool(goal_blocked), "safe_goal": (r[-1] if r else None)}


def plan_multi(start, waypoints, default_alt):
    """Ordered waypoints, each with its own altitude. Routes exclusion-aware legs
    between consecutive points so the path NEVER crosses a no-fly zone; a waypoint
    inside a zone is clamped to the nearest safe point. Returns [[lat,lon,alt],...]."""
    pp = get_planner()
    clamped = []
    # route[0] is the START (home on the ground, or the live drone position when
    # airborne) so takeoff happens THERE and every user waypoint is actually flown.
    first_alt = float(waypoints[0].get("alt", default_alt)) if waypoints else float(default_alt)
    out = [[round(start[0], 7), round(start[1], 7), first_alt]]
    prev = (start[0], start[1])
    for idx, wp in enumerate(waypoints):
        gl = (wp["lat"], wp["lon"])
        alt = float(wp.get("alt", default_alt))
        if pp.is_blocked(*gl):
            clamped.append(idx)
        sub = pp.plan(prev, gl)                 # ends outside the zone if goal blocked
        for (la, lo) in sub[1:]:                # skip prev (already emitted)
            out.append([round(la, 7), round(lo, 7), alt])
        if sub:
            prev = (sub[-1][0], sub[-1][1])
    return {"route": out, "clamped": clamped}


def _alt_of(p, default):
    return float(p[2]) if len(p) >= 3 else float(default)


def route_to_mission(route, alt, rtl=True, is_airborne=False, hold_s=15.0, return_wps=None):
    """route/return_wps items may be [lat,lon] or [lat,lon,alt] (per-waypoint height)."""
    items = []
    if not is_airborne and len(route) > 0:
        items.append({"command": "MAV_CMD_NAV_TAKEOFF", "lat": route[0][0],
                      "lon": route[0][1], "alt": _alt_of(route[0], alt)})
        wps = route[1:]
    else:
        wps = route

    n_out = len(wps)
    for i, p in enumerate(wps):
        # hold at the final outbound waypoint (over target) so it hovers visibly
        is_last_out = (i == n_out - 1) and not return_wps
        hold = hold_s if is_last_out else 0.0
        items.append({"command": "MAV_CMD_NAV_WAYPOINT", "lat": p[0], "lon": p[1],
                      "alt": _alt_of(p, alt), "p1": hold})

    # optional user-designed return legs (each with its own height)
    for p in (return_wps or []):
        items.append({"command": "MAV_CMD_NAV_WAYPOINT", "lat": p[0], "lon": p[1],
                      "alt": _alt_of(p, alt), "p1": 0.0})

    if rtl:
        items.append({"command": "MAV_CMD_NAV_RETURN_TO_LAUNCH",
                      "frame": "MAV_FRAME_MISSION", "lat": 0, "lon": 0, "alt": 0})
    return items


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
        pass

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _static(self, path, ctype):
        try:
            with open(path, "rb") as f:
                self._send(200, f.read(), ctype)
        except FileNotFoundError:
            self._send(404, {"error": "not found"})

    def do_GET(self):
        p = self.path.split("?")[0]
        if p == "/" or p == "/index.html":
            self._static(os.path.join(WEB, "index.html"), "text/html")
        elif p == "/app.js":
            self._static(os.path.join(WEB, "app.js"), "application/javascript")
        elif p == "/style.css":
            self._static(os.path.join(WEB, "style.css"), "text/css")
        elif p == "/drone.glb":
            self._static(os.path.join(WEB, "drone.glb"), "model/gltf-binary")
        elif p == "/video.mjpeg":
            return self._stream_video()
        elif p == "/api/kml":
            self._send(200, GEOJSON)
        elif p == "/api/operating_area":
            self._send(200, load_op_area())
        elif p == "/api/telemetry":
            self._send(200, FC.telemetry())
        else:
            self._send(404, {"error": "not found"})

    def _stream_video(self):
        """Proxy the MJPEG feed from cam_bridge.py (real Gazebo camera render)."""
        try:
            up = urllib.request.urlopen(CAM_BRIDGE_URL, timeout=5)
        except Exception as e:
            return self._send(503, {"error": f"camera bridge not up: {e}. "
                                    "Launch SITL (starts cam_bridge automatically)."})
        try:
            self.send_response(200)
            self.send_header("Content-Type",
                             up.headers.get("Content-Type", "multipart/x-mixed-replace;boundary=frame"))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            while True:
                chunk = up.read(16384)
                if not chunk:
                    break
                self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            up.close()

    def do_POST(self):
        p = self.path.split("?")[0]
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            return self._send(400, {"error": "bad json"})
        if p == "/api/operating_area":
            poly = body.get("polygon")   # [[lat,lon],...] ring, or null to clear
            save_op_area(poly)
            npts = len(poly) if poly else 0
            print(f"[gcs] operating area saved ({npts} pts)")
            self._send(200, {"ok": True, "points": npts})
        elif p == "/api/plan":
            try:
                res = plan_route(body["start"], body["goal"],
                                 body.get("vias"), body.get("alt", 40))
                self._send(200, {"ok": True, "route": res["route"],
                                 "clamped": res["clamped"], "safe_goal": res["safe_goal"]})
            except Exception as e:
                self._send(200, {"ok": False, "error": str(e)})
        elif p == "/api/plan_multi":
            try:
                res = plan_multi(body["start"], body["waypoints"], body.get("alt", 40))
                self._send(200, {"ok": True, "route": res["route"], "clamped": res["clamped"]})
            except Exception as e:
                self._send(200, {"ok": False, "error": str(e)})
        elif p == "/api/mission/upload":
            route = body["route"]
            alt = body.get("alt", 40)
            rtl = body.get("rtl", True)
            is_airborne = body.get("is_airborne", False)
            hold_s = body.get("hold_s", 15.0)
            return_wps = body.get("return_wps")
            rtl_alt = body.get("rtl_alt")
            if rtl_alt:                       # adjustable auto-RTL return altitude
                FC.set_param("RTL_RETURN_ALT", float(rtl_alt))
            items = route_to_mission(route, alt, rtl=rtl, is_airborne=is_airborne,
                                     hold_s=hold_s, return_wps=return_wps)
            res = FC.upload_mission(items)
            print(f"[gcs] mission upload: {res}")
            self._send(200, res)
        elif p == "/api/command":
            cmd = body.get("cmd")
            if cmd == "arm": FC.arm(True)
            elif cmd == "disarm": FC.arm(False)
            elif cmd == "rtl": FC.rtl()
            elif cmd == "start_mission": FC.start_mission()
            elif cmd == "reposition":
                lat = body.get("lat")
                lon = body.get("lon")
                alt = body.get("alt", 40)
                # never fly direct into or across a no-fly zone
                pp = get_planner()
                if pp.is_blocked(lat, lon):
                    return self._send(200, {"ok": False, "reason": "goal_blocked",
                        "error": "Goal is inside a no-fly zone. Pick another point or use Plan route."})
                tel = FC.telemetry()
                if tel.get("lat") is not None and pp.segment_blocked((tel["lat"], tel["lon"]), (lat, lon)):
                    return self._send(200, {"ok": False, "reason": "path_blocked",
                        "error": "Direct path crosses a no-fly zone. Use Plan route (it detours)."})
                FC.reposition(lat, lon, alt)
            elif cmd in ("drop", "release_buoy"):
                ch = body.get("channel", 1)
                pwm = body.get("pwm", 2000)
                FC.set_servo(channel=ch, pwm=pwm)
            else:
                return self._send(400, {"error": f"unknown cmd {cmd}"})
            self._send(200, {"ok": True, "cmd": cmd})
        elif p == "/api/sitl/start":
            try:
                subprocess.Popen(["bash", os.path.join(ROOT, "launch/launch_chattogram_clean.sh")])
                self._send(200, {"ok": True, "status": "launching"})
            except Exception as e:
                self._send(500, {"ok": False, "error": str(e)})
        elif p == "/api/sitl/stop":
            try:
                subprocess.run(["pkill", "-KILL", "-f", "gz sim"], capture_output=True)
                subprocess.run(["pkill", "-KILL", "-f", "bin/px4"], capture_output=True)
                subprocess.run(["pkill", "-KILL", "-f", "px4_sitl gz"], capture_output=True)
                subprocess.run(["pkill", "-KILL", "-f", "mavlink2rest"], capture_output=True)
                subprocess.run(["pkill", "-KILL", "-f", "cam_bridge.py"], capture_output=True)
                subprocess.run(["pkill", "-KILL", "-f", "chase_cam.py"], capture_output=True)
                self._send(200, {"ok": True, "status": "stopped"})
            except Exception as e:
                self._send(500, {"ok": False, "error": str(e)})
        elif p == "/api/gazebo/follow":
            try:
                msg = 'track_mode: FOLLOW, follow_target: {name: "x500_0"}, follow_offset: {x: -3.5, y: -3.5, z: 2.2}, follow_pgain: 0.8'
                cmd = ["gz", "topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p", msg]
                subprocess.Popen(cmd)
                self._send(200, {"ok": True, "target": "x500_0"})
            except Exception as e:
                self._send(500, {"ok": False, "error": str(e)})
        else:
            self._send(404, {"error": "not found"})


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    print(f"[gcs] serving on http://0.0.0.0:{port}")
    ThreadingHTTPServer(("0.0.0.0", port), H).serve_forever()
