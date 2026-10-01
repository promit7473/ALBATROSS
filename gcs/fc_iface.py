"""Flight-controller interface abstraction.

Everything else in the GCS talks ONLY to this interface. Today: MAVLink via
mavlink2rest. Later: add a `DDSOffboardFC` with the same methods for the Jetson.
"""
import json
import math
import os
import re
import struct
import subprocess
import time
import urllib.request
from datetime import datetime, timezone

M2R = "http://127.0.0.1:8088"
_VEH = "/v1/mavlink/vehicles/1/components/1/messages"
TGT_SYS, TGT_COMP = 1, 1
TELEMETRY_MAX_AGE_S = 5.0


class FlightControllerError(RuntimeError):
    """A command could not be delivered or its prerequisites are unavailable."""


def _fresh(record):
    """mavlink2rest keeps messages after link loss; check reception time."""
    try:
        stamp = record["status"]["time"]["last_update"]
        # Rust emits nanoseconds; Python 3.10 accepts 3 or 6 fractional digits.
        stamp = re.sub(r"\.(\d+)(?=Z|[+-]\d{2}:\d{2}$)",
                       lambda m: "." + (m.group(1) + "000000")[:6], stamp)
        updated = datetime.fromisoformat(stamp.replace("Z", "+00:00"))
        if updated.tzinfo is None:
            return False
        age = (datetime.now(timezone.utc) - updated).total_seconds()
        return 0 <= age <= TELEMETRY_MAX_AGE_S
    except (KeyError, TypeError, ValueError, AttributeError):
        return False


def _armed(message):
    mode = message.get("base_mode", "")
    if isinstance(mode, int):
        return bool(mode & 128)
    return "SAFETY_ARMED" in (mode if isinstance(mode, str) else json.dumps(mode))

# PX4 custom main/sub modes for DO_SET_MODE
MAIN_MANUAL, MAIN_ALTCTL, MAIN_POSCTL = 1, 2, 3
MAIN_AUTO = 4
SUB_MISSION, SUB_RTL, SUB_LOITER = 4, 5, 3
SUB_NONE = 0

# "Manual" side of the Manual<->Auto toggle: Position mode (sticks command GPS
# position; releasing = hold). This is the standard RC pilot mode and the safe
# default for a real Pixhawk 6C + RC. Override with MARITIME_MANUAL_MODE
# (1=MANUAL / direct stick, 2=ALTCTL, 3=POSCTL).
TOGGLE_MANUAL_MAIN = int(os.environ.get("MARITIME_MANUAL_MODE", "3"))


def ensure_mavlink2rest():
    try:
        req = urllib.request.Request(M2R + "/v1/mavlink", method="GET")
        with urllib.request.urlopen(req, timeout=0.5) as r:
            return
    except Exception:
        pass
    m2r = "/home/mhpromit7473/mavlink2rest"
    if os.path.exists(m2r):
        try:
            subprocess.Popen([m2r, "-c", "udpin:0.0.0.0:14555", "-s", "0.0.0.0:8088", "--send-initial-heartbeats"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[fc_iface] started background mavlink2rest on :8088")
        except Exception as e:
            print(f"[fc_iface] failed to start mavlink2rest: {e}")

def _get(path, timeout=2.0):
    try:
        with urllib.request.urlopen(M2R + path, timeout=timeout) as r:
            return json.loads(r.read())
    except Exception:
        return None


def _post(message):
    body = json.dumps({"header": {"system_id": 255, "component_id": 0, "sequence": 0},
                       "message": message}).encode()
    req = urllib.request.Request(M2R + "/v1/mavlink", data=body,
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=3.0) as r:
            return r.read()
    except Exception as e:
        raise FlightControllerError("MAVLink bridge could not send the command") from e


class FCInterface:
    def telemetry(self): raise NotImplementedError
    def arm(self, armed): raise NotImplementedError
    def set_mode(self, main, sub): raise NotImplementedError
    def rtl(self): raise NotImplementedError
    def start_mission(self): raise NotImplementedError
    def upload_mission(self, items): raise NotImplementedError
    def set_servo(self, channel, pwm): raise NotImplementedError
    def sync_qgc(self): raise NotImplementedError
    def current_mode(self): raise NotImplementedError
    def manual_mode(self, main=None): raise NotImplementedError
    def toggle_auto_manual(self): raise NotImplementedError


def _hdg_deg(cdeg):
    """GLOBAL_POSITION_INT.hdg is centidegrees. 65535 (and anything outside
    0..359.99 deg) means the autopilot has no heading."""
    try:
        cdeg = float(cdeg)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(cdeg) or cdeg < 0.0 or cdeg >= 36000.0:
        return None
    return cdeg / 100.0


def _int_latlon(lat, lon):
    """MAVLink lat/lon are degE7. Treat 0,0 / tiny values as 'no fix'."""
    try:
        lat, lon = int(lat), int(lon)
    except (TypeError, ValueError):
        return None
    if abs(lat) < 1000 or abs(lon) < 1000:
        return None
    return lat / 1e7, lon / 1e7


def _gps_fix_ok(fix):
    """Horizontal position is usable from a 2D fix upward.

    PX4 still fills GPS_RAW_INT / GLOBAL_POSITION_INT while fix_type is
    NO_GPS. Those coordinates wander and are not a position.
    """
    if isinstance(fix, dict):
        fix = fix.get("type")
    if isinstance(fix, bool) or fix is None:
        return False
    if isinstance(fix, int):
        return fix >= 2
    text = str(fix)
    if text.isdigit():
        return int(text) >= 2
    if "NO_GPS" in text or "NO_FIX" in text:
        return False
    return text.startswith("GPS_FIX_TYPE_")


class MavlinkRestFC(FCInterface):
    def __init__(self):
        self._streams_asked = False

    def _ask_position_streams(self):
        """Ask PX4 to emit GPS / global / home at a usable rate (once)."""
        if self._streams_asked:
            return
        self._streams_asked = True
        # msg ids: GPS_RAW_INT=24, GPS_STATUS=25, GLOBAL_POSITION_INT=33, HOME_POSITION=242
        for msgid, us in ((24, 200000), (25, 1000000), (33, 200000), (242, 1000000)):
            self._cmd("MAV_CMD_SET_MESSAGE_INTERVAL",
                      (msgid, us, 0, 0, 0, 0, 0))
        self._cmd("MAV_CMD_REQUEST_MESSAGE", (242, 0, 0, 0, 0, 0, 0))
        self._cmd("MAV_CMD_REQUEST_MESSAGE", (25, 0, 0, 0, 0, 0, 0))

    # ---------------- telemetry ----------------
    def telemetry(self):
        # One snapshot avoids nine sequential timeouts and inconsistent samples.
        messages = _get(_VEH) or {}
        hb = messages.get("HEARTBEAT")
        if not _fresh(hb):
            self._streams_asked = False
            return {"connected": False}
        try:
            self._ask_position_streams()
        except FlightControllerError:
            self._streams_asked = False  # retry after the bridge recovers
        def live(name):
            record = messages.get(name)
            return record if _fresh(record) else None
        gp = live("GLOBAL_POSITION_INT")
        hud = live("VFR_HUD")
        gps = live("GPS_RAW_INT")
        gpsst = live("GPS_STATUS")
        sysst = live("SYS_STATUS")
        att = live("ATTITUDE")
        # Home is event-driven and may legitimately be older than telemetry.
        home = messages.get("HOME_POSITION")
        altm = live("ALTITUDE")
        t = {"connected": True}
        fix_ok = None
        if gps:
            m = gps["message"]
            ft = m.get("fix_type")
            fix_ok = _gps_fix_ok(ft)
            t.update(sats=m.get("satellites_visible"),
                     fix=(ft.get("type") if isinstance(ft, dict) else ft))
            if m.get("eph") not in (None, 9999):
                t["eph"] = m.get("eph")
            if m.get("h_acc") not in (None, 4294967295):
                t["h_acc_m"] = m["h_acc"] / 1000.0
            if m.get("v_acc") not in (None, 4294967295) and m.get("v_acc") < 1e8:
                t["v_acc_m"] = m["v_acc"] / 1000.0
        if att:
            m = att["message"]
            t.update(roll=math.degrees(m.get("roll", 0.0)),
                     pitch=math.degrees(m.get("pitch", 0.0)),
                     yaw=math.degrees(m.get("yaw", 0.0)))
        if gp:
            m = gp["message"]
            # A fresh NO_GPS / NO_FIX sentence means the global lat/lon is not a fix.
            # No GPS sentence at all (simulator snapshots) still uses global position.
            if fix_ok is not False:
                ll = _int_latlon(m.get("lat"), m.get("lon"))
                if ll:
                    t.update(lat=ll[0], lon=ll[1])
            t.update(alt_msl=m["alt"] / 1000.0, rel_alt=m["relative_alt"] / 1000.0,
                     hdg=_hdg_deg(m.get("hdg")), vx=m["vx"] / 100.0, vy=m["vy"] / 100.0)
        if hud:
            m = hud["message"]
            t.update(groundspeed=m.get("groundspeed"), climb=m.get("climb"))
        if gps and fix_ok and "lat" not in t:
            ll = _int_latlon(gps["message"].get("lat"), gps["message"].get("lon"))
            if ll:
                t.update(lat=ll[0], lon=ll[1])
        if gpsst:
            m = gpsst["message"]
            n = int(m.get("satellites_visible") or 0)
            prn = m.get("satellite_prn") or []
            used = m.get("satellite_used") or []
            el = m.get("satellite_elevation") or []
            az = m.get("satellite_azimuth") or []
            snr = m.get("satellite_snr") or []
            detail = []
            for i in range(min(n, 20, len(prn) if isinstance(prn, list) else 0)):
                detail.append({
                    "prn": prn[i] if i < len(prn) else i,
                    "used": bool(used[i]) if i < len(used) else False,
                    "el": el[i] if i < len(el) else 0,
                    "az": az[i] if i < len(az) else 0,
                    "snr": snr[i] if i < len(snr) else 0,
                })
            if detail:
                t["gps_sats"] = detail
        if home:
            m = home["message"]
            ll = _int_latlon(m.get("latitude"), m.get("longitude"))
            if ll:
                t["home_lat"], t["home_lon"] = ll
        if altm and t.get("rel_alt") is None:
            m = altm["message"]
            if m.get("altitude_relative") is not None:
                t["rel_alt"] = m["altitude_relative"]
        if sysst:
            m = sysst["message"]
            t.update(battery_v=m.get("voltage_battery", 0) / 1000.0,
                     battery_pct=m.get("battery_remaining"))
        if hb:
            m = hb["message"]
            t["armed"] = _armed(m)
            t["custom_mode"] = m.get("custom_mode")
        return t

    # ---------------- commands ----------------
    def _cmd(self, command, p=(0,)*7):
        self._heartbeat()
        _post({"type": "COMMAND_LONG", "command": {"type": command},
               "param1": float(p[0]), "param2": float(p[1]), "param3": float(p[2]),
               "param4": float(p[3]), "param5": float(p[4]), "param6": float(p[5]),
               "param7": float(p[6]), "target_system": TGT_SYS,
               "target_component": TGT_COMP, "confirmation": 0})

    def arm(self, armed):
        self._cmd("MAV_CMD_COMPONENT_ARM_DISARM", (1 if armed else 0, 0, 0, 0, 0, 0, 0))
        return True

    def set_mode(self, main, sub):
        # base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
        self._cmd("MAV_CMD_DO_SET_MODE", (1, main, sub, 0, 0, 0, 0))
        return True

    def rtl(self):
        return self.set_mode(MAIN_AUTO, SUB_RTL)

    def current_mode(self):
        """Decode HEARTBEAT.custom_mode into a PX4 (main, sub) mode tuple.
        Reject commands when the heartbeat is missing or stale."""
        hb = self._heartbeat()
        cm = hb["message"].get("custom_mode", 0) or 0
        return ((cm >> 16) & 0xff, (cm >> 24) & 0xff)

    def _heartbeat(self):
        hb = _get(f"{_VEH}/HEARTBEAT")
        if not _fresh(hb):
            raise FlightControllerError("No live aircraft heartbeat; check the telemetry link")
        return hb

    def manual_mode(self, main=None):
        """Hand control to the pilot (default Position mode). The RC then has full
        authority; GCS autonomy stays suspended until toggled back to Auto.
        Does NOT arm/disarm -- the pilot/RC owns arming on real hardware."""
        self.set_mode(main if main is not None else TOGGLE_MANUAL_MAIN, SUB_NONE)
        return True

    def toggle_auto_manual(self):
        """Smart flip used by the web UI's Manual<->Auto toggle button.
        - in any AUTO.* sub-mode  -> manual (POSCTL)  : pilot gets the sticks
        - otherwise               -> AUTO.MISSION     : resume uploaded mission
        Resumes (does not restart) the mission, and never auto-arms."""
        cur = self.current_mode()
        if cur and cur[0] == MAIN_AUTO:
            self.manual_mode()
            return {"ok": True, "to": "manual", "main": TOGGLE_MANUAL_MAIN,
                    "label": "Manual"}
        self.continue_mission()
        return {"ok": True, "to": "auto", "main": MAIN_AUTO,
                "sub": SUB_MISSION, "label": "Auto"}

    def stop_mission(self):
        """Halt the active mission: hold/loiter in place (safe) and wipe the
        uploaded mission so the next Upload starts clean."""
        self.set_mode(MAIN_AUTO, SUB_LOITER)          # AUTO.LOITER -> hover in place
        time.sleep(0.2)
        _post({"type": "MISSION_CLEAR_ALL", "target_system": TGT_SYS,
               "target_component": TGT_COMP, "mission_type": {"type": "MAV_MISSION_TYPE_ALL"}})
        return True

    def start_mission(self):
        hb = self._heartbeat()
        real = os.environ.get("GCS_MODE") == "real"
        if real and not _armed(hb["message"]):
            raise FlightControllerError("Aircraft is disarmed; arm separately before starting the mission")
        _post({"type": "MISSION_SET_CURRENT", "seq": 0, "target_system": TGT_SYS, "target_component": TGT_COMP})
        time.sleep(0.1)
        self.set_mode(MAIN_AUTO, SUB_MISSION)
        time.sleep(0.4)
        if not real:
            self.arm(True)
        return True

    def continue_mission(self):
        """Resume the uploaded mission from the CURRENT item (unlike start_mission,
        this does not reset to seq 0) -- used by the 'Proceed' button after Alt-Hold."""
        self.set_mode(MAIN_AUTO, SUB_MISSION)
        return True

    def reposition(self, lat, lon, alt):
        # MAV_CMD_DO_REPOSITION = 192 (flies directly to coordinates in guided mode)
        self._cmd("MAV_CMD_DO_REPOSITION", (-1, 1, 0, float('nan'), lat, lon, alt))
        return True

    def set_speed(self, speed, kind=1):
        # MAV_CMD_DO_CHANGE_SPEED = 178; p1=type (1=ground speed), p2=speed m/s, p3=throttle(-1)
        self._cmd("MAV_CMD_DO_CHANGE_SPEED", (kind, float(speed), -1, 0, 0, 0, 0))
        return True

    def set_servo(self, channel=1, pwm=2000):
        # MAV_CMD_DO_SET_SERVO = 183 (releases payload via PWM relay / actuator)
        self._cmd("MAV_CMD_DO_SET_SERVO", (channel, pwm, 0, 0, 0, 0, 0))
        return True

    def set_param(self, name, value, ptype="MAV_PARAM_TYPE_REAL32"):
        """Set a PX4 parameter via mavlink2rest PARAM_SET.

        The MAVLink field is a float. INT32 parameters are the integer's
        bits stored in that float; sending the number 3.0 for an int param
        stores 1077936128. mavlink2rest serialises param_id as a string.
        """
        if ptype == "MAV_PARAM_TYPE_INT32":
            wire = struct.unpack("<f", struct.pack("<i", int(value)))[0]
        else:
            wire = float(value)
        _post({"type": "PARAM_SET", "param_id": name, "param_value": wire,
               "param_type": {"type": ptype},
               "target_system": TGT_SYS, "target_component": TGT_COMP})
        return True

    # ---------------- mission upload (MISSION_INT protocol) ----------------
    def _mreq_counter(self):
        d = _get(f"{_VEH}/MISSION_REQUEST_INT")
        if not d:
            return None
        return d["status"]["time"]["counter"], d["message"]["seq"]

    def _mack(self):
        d = _get(f"{_VEH}/MISSION_ACK")
        if not d:
            return None
        mt = d["message"].get("mavtype")
        mt = mt.get("type") if isinstance(mt, dict) else mt
        return (d["status"]["time"]["counter"], mt)

    def upload_mission(self, items):
        """items: list of dicts {command, lat, lon, alt, frame, current, p1..p4}."""
        self._heartbeat()
        n = len(items)
        if not n:
            return {"ok": False, "error": "Mission is empty"}
        base_ack = self._mack()
        base_ack_ctr = base_ack[0] if base_ack else -1
        base_req = self._mreq_counter()
        base_req_ctr = base_req[0] if base_req else -1
        _post({"type": "MISSION_COUNT", "count": n, "target_system": TGT_SYS,
               "target_component": TGT_COMP, "mission_type": {"type": "MAV_MISSION_TYPE_MISSION"},
               "opaque_id": 0})
        t0 = time.time()
        sent = set()
        last_req_ctr = base_req_ctr
        while time.time() - t0 < 20.0:
            # ACK => done (accepted) or failed (error type)
            ack = self._mack()
            if ack is not None and ack[0] != base_ack_ctr:
                if ack[1] == "MAV_MISSION_ACCEPTED":
                    _post({"type": "MISSION_SET_CURRENT", "seq": 0, "target_system": TGT_SYS, "target_component": TGT_COMP})
                    return {"ok": True, "count": n}
                return {"ok": False, "error": ack[1], "sent": sorted(sent)}
            r = self._mreq_counter()
            if r and r[0] != last_req_ctr:
                last_req_ctr = r[0]
                seq = r[1]
                if not 0 <= seq < n:
                    return {"ok": False, "error": "Aircraft requested an invalid mission item"}
                self._send_item(items[seq], seq)
                sent.add(seq)
            # also proactively (re)send seq 0 right after count in case first request was missed
            elif 0 not in sent and time.time() - t0 > 0.3:
                self._send_item(items[0], 0); sent.add(0)
            time.sleep(0.03)
        return {"ok": False, "error": "timeout", "sent": sorted(sent)}

    def _send_item(self, it, seq):
        _post({"type": "MISSION_ITEM_INT",
               "seq": seq, "command": {"type": it["command"]},
               "frame": {"type": it.get("frame", "MAV_FRAME_GLOBAL_RELATIVE_ALT")},
               "current": 1 if seq == 0 else 0, "autocontinue": 1,
               "param1": float(it.get("p1", 0)), "param2": float(it.get("p2", 0)),
               "param3": float(it.get("p3", 0)), "param4": float(it.get("p4", 0)),
               "x": int(round(it.get("lat", 0) * 1e7)), "y": int(round(it.get("lon", 0) * 1e7)),
               "z": float(it.get("alt", 0)),
               "target_system": TGT_SYS, "target_component": TGT_COMP,
               "mission_type": {"type": "MAV_MISSION_TYPE_MISSION"}})

    # ---------------- QGC mission sync ----------------
    def sync_qgc(self):
        """Make the mission just uploaded via the web GCS appear in
        QGroundControl's Plan view.

        QGC's PlanManager only loads missions during initial vehicle connection
        (InitialConnectStateMachine) — it ignores unsolicited MISSION_COUNT and
        even disconnects its message handler when idle. So the only reliable way
        to surface a mission uploaded by a *different* GCS (us, via mavlink2rest)
        is to restart QGC: it re-connects to PX4 on :14550 and re-downloads the
        stored mission as part of its connect sequence.

        Runs in a daemon thread so the web request returns immediately; QGC
        restarts ~2 s later. Set QGC_AUTO_SYNC=0 to suppress (e.g. headless)."""
        # Never interrupt a real operator's GCS to refresh its mission display.
        # Use QGC's Download from Vehicle action after uploading through the web.
        if os.environ.get("GCS_MODE") == "real":
            return
        if os.environ.get("QGC_AUTO_SYNC", "1") != "1":
            return
        import threading
        threading.Thread(target=self._sync_qgc_thread, daemon=True).start()

    def _sync_qgc_thread(self):
        try:
            time.sleep(2.0)
            subprocess.run(["pkill", "-KILL", "-f", "QGroundControl"],
                           capture_output=True, timeout=5)
            time.sleep(1.5)
            root = os.environ.get(
                "MARITIME_ROOT",
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            subprocess.Popen(["bash", os.path.join(root, "launch", "qgc.sh")],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[fc_iface] QGC restarted — will re-download mission from PX4")
        except Exception as e:
            print(f"[fc_iface] QGC sync (restart) failed: {e}")
