#!/usr/bin/env python3
"""Maritime GCS backend (slice 1: live ops map).

Serves the web UI, proxies live telemetry from the FC interface, and exposes the
KML operating area + exclusion zones as GeoJSON. No web framework -- stdlib only.
"""
import json
import os
import subprocess
import sys
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Gazebo feed now comes from cam_bridge.py (real onboard camera render -> MJPEG),
# which works under Wayland/Xorg/headless. We just proxy it so the UI stays on one port.
CAM_BRIDGE_URL = os.environ.get("CAM_BRIDGE_URL", "http://127.0.0.1:8091/video.mjpeg")
GCS_MODE = os.environ.get("GCS_MODE", "sitl")  # sitl | real
CAM_BRIDGE2_URL = os.environ.get(
    "CAM_BRIDGE2_URL", CAM_BRIDGE_URL if GCS_MODE == "real"
    else "http://127.0.0.1:8092/video.mjpeg")  # real FPV / sim top-down

HERE = os.path.dirname(os.path.abspath(__file__))              # <project-root>/gcs
ROOT = os.environ.get("MARITIME_ROOT", os.path.dirname(HERE))  # <project-root> (relocatable)
WEB = os.path.join(HERE, "web")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws/src/maritime_autonomy/maritime_autonomy"))

import geo  # noqa: E402  (OperatingMap/Zone from the autonomy package)
import planner as planner_mod  # noqa: E402  (PathPlanner A* over the operating area)
from fc_iface import MavlinkRestFC, FlightControllerError, ensure_mavlink2rest  # noqa: E402

KML = os.path.join(ROOT, "assets/chattogram_zones.kml")
BASE = {"name": "Base / Port Gate", "lat": 22.318725, "lon": 91.813156}
# Banani mesh is a 1.6 km square. PX4 overwrites the SDF GPS origin with
# PX4_HOME_*, so this home is the only point where that texture and the
# reported GPS describe the same ground.
BANANI_ORIGIN = (23.793700, 90.406600, 8.0)
BANANI_RADIUS_M = 900.0
OP_FILE = os.path.join(HERE, "operating_area.json")   # user-drawn boundary persistence
CAM2_VIEW_FILE = os.path.join(os.path.expanduser("~"), ".cam2_view")  # 2nd feed: topdown|fpv
FC = MavlinkRestFC()
_BUILDINGS = {}  # (lat3, lon3) -> list of {ring, height}

# --- CAAB (Bangladesh Civil Aviation Authority) UAS rule enforced in the sim ---
# CAC-5 (RPAS, 2019) sec 4(g): max height 200 ft AGL. The 10-NM airport no-fly
# (sec 4h) is not drawn -- it blankets all of Chattogram; the KML exclusion zones
# are the operational no-fly layer instead. Overridable via env.
CAAB_MAX_ALT_FT = float(os.environ.get("CAAB_MAX_ALT_FT", "200"))
CAAB_MAX_ALT_M = round(CAAB_MAX_ALT_FT * 0.3048, 2)             # 200 ft -> 60.96 m


def sitl_world(lat, lon):
    """Pick the Gazebo world whose terrain matches the requested home.

    Returns (world, lat, lon, home_alt). home_alt is set only when the home
    is snapped to a world origin. A click inside the Banani mesh boots
    banani.sdf at that mesh center. Every other click boots chattogram and
    keeps the requested coordinates. Missing coordinates stay on chattogram
    and leave the launch script's own home defaults in place.
    """
    sdf = os.path.join(ROOT, "external/PX4-Autopilot/Tools/simulation/gz/worlds/banani.sdf")
    try:
        lat_f, lon_f = float(lat), float(lon)
    except (TypeError, ValueError):
        return "chattogram", None, None, None
    if os.path.isfile(sdf) and geo.haversine_m(
            lat_f, lon_f, BANANI_ORIGIN[0], BANANI_ORIGIN[1]) <= BANANI_RADIUS_M:
        return "banani", BANANI_ORIGIN[0], BANANI_ORIGIN[1], BANANI_ORIGIN[2]
    return "chattogram", lat_f, lon_f, None


def caab_clamp_alt(a):
    """Clamp a target altitude (m AGL) to the CAAB ceiling."""
    try:
        a = float(a)
    except (TypeError, ValueError):
        return CAAB_MAX_ALT_M
    return min(a, CAAB_MAX_ALT_M)


def load_op_area():
    try:
        with open(OP_FILE) as f:
            return json.load(f)
    except Exception:
        return {"polygon": None}


def save_op_area(poly):
    with open(OP_FILE, "w") as f:
        json.dump({"polygon": poly}, f)


def fetch_buildings(lat, lon, radius=700):
    """OSM building footprints around a point, extruded later in Cesium."""
    key = (round(lat, 3), round(lon, 3), int(radius))
    if key in _BUILDINGS:
        return _BUILDINGS[key]
    q = (
        f'[out:json][timeout:40];'
        f'(way["building"](around:{int(radius)},{lat:.6f},{lon:.6f}););'
        f'out geom qt;'
    )
    out = []
    data = None
    last_err = None
    for url in ("https://overpass-api.de/api/interpreter",
                "https://overpass.kumi.systems/api/interpreter"):
        try:
            req = urllib.request.Request(
                url, data=q.encode(),
                headers={"Content-Type": "text/plain",
                         "User-Agent": "albatross-gcs/1.0"})
            with urllib.request.urlopen(req, timeout=25) as r:
                data = json.loads(r.read().decode())
            break
        except Exception as e:
            last_err = e
            data = None
    try:
        if data is None:
            raise last_err or RuntimeError("overpass unreachable")
        for el in data.get("elements") or []:
            geom = el.get("geometry") or []
            if len(geom) < 3:
                continue
            ring = [[g["lon"], g["lat"]] for g in geom]
            tags = el.get("tags") or {}
            h = None
            if tags.get("height"):
                try:
                    h = float(str(tags["height"]).split()[0])
                except (TypeError, ValueError):
                    h = None
            if h is None and tags.get("building:levels"):
                try:
                    h = float(str(tags["building:levels"]).split(";")[0]) * 3.2
                except (TypeError, ValueError):
                    h = None
            out.append({"ring": ring, "height": max(6.0, min(h or 10.0, 80.0))})
    except Exception as e:
        print(f"[gcs] buildings fetch failed: {e}")
        out = []
    if out:
        _BUILDINGS[key] = out
    print(f"[gcs] buildings {len(out)} around {lat:.4f},{lon:.4f}")
    return out

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
    first_alt = caab_clamp_alt(waypoints[0].get("alt", default_alt)) if waypoints else caab_clamp_alt(default_alt)
    out = [[round(start[0], 7), round(start[1], 7), first_alt]]
    prev = (start[0], start[1])
    for idx, wp in enumerate(waypoints):
        gl = (wp["lat"], wp["lon"])
        alt = caab_clamp_alt(wp.get("alt", default_alt))
        if pp.is_blocked(*gl):
            clamped.append(idx)
        sub = pp.plan(prev, gl)                 # ends outside the zone if goal blocked
        for (la, lo) in sub[1:]:                # skip prev (already emitted)
            out.append([round(la, 7), round(lo, 7), alt])
        if sub:
            prev = (sub[-1][0], sub[-1][1])
    return {"route": out, "clamped": clamped}


def _alt_of(p, default):
    a = float(p[2]) if len(p) >= 3 else float(default)
    return caab_clamp_alt(a)            # every mission altitude respects the CAAB cap


def route_to_mission(route, alt, rtl=True, is_airborne=False, hold_s=15.0,
                     return_wps=None, rtl_alt=None, home_lat=None, home_lon=None):
    """route/return_wps items may be [lat,lon] or [lat,lon,alt] (per-waypoint height).
    When rtl=True and rtl_alt is provided, the return home is built as explicit
    waypoints (climb to rtl_alt, fly home, descend, land) so the altitude is
    guaranteed and does not depend on the RTL_RETURN_ALT PX4 parameter."""
    items = []
    if not is_airborne and len(route) > 0:
        items.append({"command": "MAV_CMD_NAV_TAKEOFF", "lat": route[0][0],
                      "lon": route[0][1], "alt": _alt_of(route[0], alt)})
        wps = route[1:]
    else:
        wps = route

    n_out = len(wps)
    for i, p in enumerate(wps):
        is_last_out = (i == n_out - 1) and not return_wps
        hold = hold_s if is_last_out else 0.0
        items.append({"command": "MAV_CMD_NAV_WAYPOINT", "lat": p[0], "lon": p[1],
                      "alt": _alt_of(p, alt), "p1": hold})

    for p in (return_wps or []):
        items.append({"command": "MAV_CMD_NAV_WAYPOINT", "lat": p[0], "lon": p[1],
                      "alt": _alt_of(p, alt), "p1": 0.0})

    if rtl and rtl_alt is not None and home_lat is not None and home_lon is not None:
        items.append({"command": "MAV_CMD_NAV_WAYPOINT", "lat": home_lat,
                      "lon": home_lon, "alt": caab_clamp_alt(rtl_alt), "p1": 0.0})
        items.append({"command": "MAV_CMD_NAV_LAND", "lat": home_lat,
                      "lon": home_lon, "alt": 0.0})
    elif rtl:
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
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            # dev UI: always serve the latest asset, never a stale cached copy
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            self.wfile.write(body)
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
        elif p == "/sat.png":
            self._static(os.path.join(WEB, "sat.png"), "image/png")
        elif p == "/intelis_logo.svg":
            self._static(os.path.join(WEB, "intelis_logo.svg"), "image/svg+xml")
        elif p == "/video.mjpeg":
            return self._stream_video(CAM_BRIDGE_URL)
        elif p == "/video2.mjpeg":
            return self._stream_video(CAM_BRIDGE2_URL)
        elif p == "/api/kml":
            self._send(200, GEOJSON)
        elif p == "/api/caab":
            self._send(200, {"max_alt_ft": CAAB_MAX_ALT_FT, "max_alt_m": CAAB_MAX_ALT_M})
        elif p == "/api/operating_area":
            self._send(200, load_op_area())
        elif p == "/api/telemetry":
            self._send(200, FC.telemetry())
        elif p == "/api/mode":
            self._send(200, {"mode": GCS_MODE})
        elif p == "/api/camera/status":
            if GCS_MODE != "real":
                return self._send(200, {"ok": True, "source": "simulation"})
            health_url = urllib.parse.urljoin(CAM_BRIDGE2_URL, "/healthz")
            try:
                with urllib.request.urlopen(health_url, timeout=2) as response:
                    status = json.loads(response.read())
                status.update(source="front_camera", bridge_connected=True)
                self._send(200, status)
            except Exception:
                self._send(200, {"ok": False, "fresh": False, "source": "front_camera",
                                 "bridge_connected": False,
                                 "error": "Front-camera video bridge is unavailable"})
        elif p == "/api/buildings":
            qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            try:
                lat = float(qs.get("lat", [None])[0])
                lon = float(qs.get("lon", [None])[0])
            except (TypeError, ValueError):
                return self._send(400, {"error": "lat,lon required"})
            r = float(qs.get("r", ["700"])[0])
            self._send(200, {"buildings": fetch_buildings(lat, lon, r)})
        else:
            self._send(404, {"error": "not found"})

    def _stream_video(self, url=CAM_BRIDGE_URL):
        """Proxy the MJPEG feed from cam_bridge.py (real Gazebo camera render)."""
        try:
            up = urllib.request.urlopen(url, timeout=5)
        except Exception as e:
            hint = ("FPV RTSP bridge not up." if GCS_MODE == "real"
                    else "Launch SITL (starts cam_bridge automatically).")
            return self._send(503, {"error": f"camera bridge not up: {e}. {hint}"})
        try:
            self.send_response(200)
            self.send_header("Content-Type",
                             up.headers.get("Content-Type", "multipart/x-mixed-replace;boundary=frame"))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            while True:
                chunk = up.read1(16384)
                if not chunk:
                    break
                self.wfile.write(chunk)
        except (OSError, TimeoutError):
            pass
        finally:
            up.close()

    def do_POST(self):
        try:
            self._handle_post()
        except FlightControllerError as e:
            self._send(503, {"ok": False, "error": str(e)})

    def _handle_post(self):
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
            home_lat = body.get("home_lat")
            home_lon = body.get("home_lon")
            if not home_lat or not home_lon:
                t = FC.telemetry()
                home_lat = t.get("lat", 22.318725)
                home_lon = t.get("lon", 91.813156)
            items = route_to_mission(route, alt, rtl=rtl, is_airborne=is_airborne,
                                     hold_s=hold_s, return_wps=return_wps,
                                     rtl_alt=float(rtl_alt) if rtl and rtl_alt else None,
                                     home_lat=home_lat if rtl and rtl_alt else None,
                                     home_lon=home_lon if rtl and rtl_alt else None)
            res = FC.upload_mission(items)
            # flag if any requested altitude was pulled down to the CAAB ceiling
            raw = [float(p[2]) for p in route if len(p) >= 3] + [float(alt)]
            if rtl and rtl_alt:
                raw.append(float(rtl_alt))
            if isinstance(res, dict) and raw and max(raw) > CAAB_MAX_ALT_M:
                res["alt_capped_to_m"] = CAAB_MAX_ALT_M
            print(f"[gcs] mission upload: {res}")
            if isinstance(res, dict) and res.get("ok"):
                FC.sync_qgc()
            self._send(200, res)
        elif p == "/api/command":
            cmd = body.get("cmd")
            if cmd == "arm": FC.arm(True)
            elif cmd == "disarm": FC.arm(False)
            elif cmd == "rtl": FC.rtl()
            elif cmd == "stop_mission": FC.stop_mission()
            elif cmd == "start_mission": FC.start_mission()
            elif cmd == "continue_mission": FC.continue_mission()
            elif cmd == "toggle_mode":
                # Manual<->Auto toggle (web UI button). Returns the resulting mode
                # so the UI can update its label without waiting for the next poll.
                self._send(200, FC.toggle_auto_manual()); return
            elif cmd == "manual": FC.manual_mode()
            elif cmd == "auto": FC.continue_mission()
            elif cmd == "reposition":
                lat = body.get("lat")
                lon = body.get("lon")
                alt = caab_clamp_alt(body.get("alt", 40))   # respect the CAAB ceiling
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
            elif cmd == "cam_view":
                view = body.get("view", "topdown")
                if view not in ("topdown", "fpv"):
                    return self._send(400, {"error": "view must be topdown|fpv"})
                with open(CAM2_VIEW_FILE, "w") as f:
                    f.write(view)   # chase_cam.py polls this to re-aim the 2nd cam live
            elif cmd == "set_speed":
                speed = max(0.5, min(30.0, float(body.get("speed", 5.0))))
                FC.set_speed(speed)
            elif cmd in ("drop", "release_buoy"):
                ch = body.get("channel", 1)
                pwm = body.get("pwm", 2000)
                FC.set_servo(channel=ch, pwm=pwm)
            else:
                return self._send(400, {"error": f"unknown cmd {cmd}"})
            self._send(200, {"ok": True, "cmd": cmd})
        elif p == "/api/sitl/start":
            try:
                mode = body.get("mode", "plan")
                env = os.environ.copy()
                world, lat, lon, snap_alt = sitl_world(body.get("lat"), body.get("lon"))
                env["PX4_GZ_WORLD"] = world
                if lat is not None and lon is not None:
                    env["PX4_HOME_LAT"] = f"{lat:.7f}"
                    env["PX4_HOME_LON"] = f"{lon:.7f}"
                    alt = snap_alt if snap_alt is not None else body.get("alt", 5.0)
                    env["PX4_HOME_ALT"] = str(alt)
                env["SITL_RC_IN_MODE"] = "1" if mode == "manual" else "4"
                subprocess.Popen(["bash", os.path.join(ROOT, "launch/launch_chattogram_clean.sh")],
                                 env=env)
                if mode == "manual":
                    subprocess.Popen(["bash", os.path.join(ROOT, "launch/mav_hub_sitl_rc.sh")])
                self._send(200, {"ok": True, "status": "launching", "mode": mode,
                                 "world": world, "home": {"lat": lat, "lon": lon}})
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
    ensure_mavlink2rest()
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    print(f"[gcs] serving on http://0.0.0.0:{port}  mode={GCS_MODE}")
    ThreadingHTTPServer(("0.0.0.0", port), H).serve_forever()
