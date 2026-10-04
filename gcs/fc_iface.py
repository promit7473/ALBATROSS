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
import threading
import uuid
import urllib.request
from datetime import datetime, timezone

M2R = "http://127.0.0.1:8088"
_VEH = "/v1/mavlink/vehicles/1/components/1/messages"
TGT_SYS, TGT_COMP = 1, 1
GCS_SYS, GCS_COMP = 246, 190  # distinct from QGC and the RC bridge
TELEMETRY_MAX_AGE_S = 5.0


class FlightControllerError(RuntimeError):
    """A command could not be delivered or its prerequisites are unavailable."""


def _record_age(record):
    """mavlink2rest keeps messages after link loss; check reception time."""
    try:
        stamp = record["status"]["time"]["last_update"]
        # Rust emits nanoseconds; Python 3.10 accepts 3 or 6 fractional digits.
        stamp = re.sub(r"\.(\d+)(?=Z|[+-]\d{2}:\d{2}$)",
                       lambda m: "." + (m.group(1) + "000000")[:6], stamp)
        updated = datetime.fromisoformat(stamp.replace("Z", "+00:00"))
        if updated.tzinfo is None:
            return None
        age = (datetime.now(timezone.utc) - updated).total_seconds()
        return age if age >= 0 else None
    except (KeyError, TypeError, ValueError, AttributeError):
        return None


def _fresh(record):
    age = _record_age(record)
    return age is not None and age <= TELEMETRY_MAX_AGE_S


def sensor_health(messages):
    """Read-only diagnostics. Never infer health from a flag without presence."""
    rows = []
    def row(name, source, state, detail):
        age = _record_age(messages.get(source))
        if age is None: state, detail = 'unknown', 'No timestamped report'
        elif age > TELEMETRY_MAX_AGE_S: state, detail = 'stale', 'Report expired'
        rows.append(dict(name=name, state=state, detail=detail,
                         age_s=round(age, 1) if age is not None else None))
    def flag(value, name, bit):
        if isinstance(value, int): return bool(value & bit)
        if isinstance(value, dict): value = value.get('bits', value.get('type', ''))
        if isinstance(value, int): return bool(value & bit)
        return name in str(value).replace('|', ' ').replace(',', ' ').split()
    system = (messages.get('SYS_STATUS') or {}).get('message', {})
    for name, suffix, bit in [('Gyroscope','3D_GYRO',1),('Accelerometer','3D_ACCEL',2),
            ('Compass','3D_MAG',4),('Barometer','ABSOLUTE_PRESSURE',8),('RC receiver','RC_RECEIVER',65536)]:
        key = 'MAV_SYS_STATUS_SENSOR_' + suffix
        present = flag(system.get('onboard_control_sensors_present'),key,bit)
        enabled = flag(system.get('onboard_control_sensors_enabled'),key,bit)
        healthy = flag(system.get('onboard_control_sensors_health'),key,bit)
        state, detail = ('unknown','Not reported present') if not present else (
            ('warning','Reported disabled') if not enabled else (
            ('healthy','Reported healthy') if healthy else ('warning','Reported unhealthy')))
        row(name,'SYS_STATUS',state,detail)
    gps = (messages.get('GPS_RAW_INT') or {}).get('message', {})
    fix = gps.get('fix_type', {})
    fix = fix.get('type') if isinstance(fix,dict) else fix
    row('GPS','GPS_RAW_INT','healthy' if _gps_fix_ok(fix) else 'warning',
        str(fix or 'Unknown fix').replace('GPS_FIX_TYPE_','') + ' · ' + str(gps.get('satellites_visible','—')) + ' satellites')
    distance = (messages.get('DISTANCE_SENSOR') or {}).get('message', {})
    cm = distance.get('current_distance')
    valid = (isinstance(cm,(int,float)) and math.isfinite(cm) and
             distance.get('min_distance',float('inf')) <= cm <= distance.get('max_distance',-1)
             and distance.get('signal_quality') != 1)
    orientation = distance.get('orientation', {})
    orientation = orientation.get('type') if isinstance(orientation,dict) else orientation
    downward = orientation in (25,'MAV_SENSOR_ROTATION_PITCH_270')
    row('Downward range' if downward else 'Range sensor','DISTANCE_SENSOR',
        'healthy' if valid else 'warning', f'{cm / 100:.2f} m · quality {distance.get("signal_quality", "unknown")}' if valid else 'Invalid or unavailable range')
    battery = (messages.get('BATTERY_STATUS') or {}).get('message', {})
    current = battery.get('current_battery')
    row('Battery current','BATTERY_STATUS','healthy' if isinstance(current,(int,float)) and current > 0 else 'unknown',
        ('0 A reported; sensing unverified' if current == 0 else f'{current / 100:.2f} A') if isinstance(current,(int,float)) and current >= 0 else 'Current unavailable')
    estimator = (messages.get('ESTIMATOR_STATUS') or {}).get('message', {})
    flags = estimator.get('flags', '')
    constant = flag(flags,'ESTIMATOR_CONST_POS_MODE',128)
    row('Position estimator','ESTIMATOR_STATUS','warning' if constant else 'unknown',
        'Constant-position mode; navigation not confirmed' if constant else 'Report available; readiness checked separately')
    heartbeat = (messages.get('HEARTBEAT') or {}).get('message', {})
    state = heartbeat.get('system_status', {})
    state = state.get('type') if isinstance(state,dict) else state
    row('Aircraft state','HEARTBEAT','healthy' if state in (3,4,'MAV_STATE_STANDBY','MAV_STATE_ACTIVE') else 'warning',str(state or 'Unknown').replace('MAV_STATE_',''))
    return rows


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
    body = json.dumps({"header": {"system_id": GCS_SYS, "component_id": GCS_COMP, "sequence": 0},
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
        self.mission_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.uploaded_items = None
        self.mission_token = None
        self.mission_id = None
        self.interrupted = threading.Event()

    def _ask_position_streams(self):
        """Ask PX4 to emit GPS / global / home at a usable rate (once)."""
        if self._streams_asked:
            return
        self._streams_asked = True
        # msg ids: GPS_RAW_INT=24, GPS_STATUS=25, GLOBAL_POSITION_INT=33, HOME_POSITION=242
        for msgid, us in ((24, 200000), (25, 1000000), (33, 200000), (242, 1000000), (65, 500000), (230, 500000), (42, 500000)):
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
            # A disconnected/rebooted vehicle must not inherit a previous mission
            # authorization held by another tab or an in-flight web request.
            self.mission_token = None
            self.mission_id = None
            self.interrupted.set()
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
        rc = live("RC_CHANNELS")
        mission = live("MISSION_CURRENT")
        estimator = live("ESTIMATOR_STATUS")
        sysst = live("SYS_STATUS")
        att = live("ATTITUDE")
        # Home is event-driven and may legitimately be older than telemetry.
        home = messages.get("HOME_POSITION")
        altm = live("ALTITUDE")
        t = {"connected": True, "sensor_health": sensor_health(messages)}
        t['rc_available'] = bool(rc and rc['message'].get('chancount', 0) > 0
                                 and rc['message'].get('rssi', 255) != 0)
        t['rc_status'] = 'receiving' if t['rc_available'] else 'unknown'
        if rc:
            rssi = rc['message'].get('rssi', 255)
            t['rc_rssi'] = rssi if isinstance(rssi, (int, float)) and 0 <= rssi < 255 else None
        t['mission_token'] = self.mission_token
        if mission:
            m = mission['message']
            t.update(mission_seq=m.get('seq'), mission_total=m.get('total'), mission_id=m.get('mission_id'))
            if self.mission_id and m.get('mission_id') and self.mission_id != m['mission_id']:
                self.mission_token = None
                t.update(mission_token=None, mission_changed=True)
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
        # Conservatively require a global position and, when supplied, estimator
        # horizontal/vertical absolute-position flags. PX4 still decides acceptance.
        position_ok = 'lat' in t and 'lon' in t and t.get('rel_alt') is not None
        if estimator:
            flags = estimator['message'].get('flags', 0)
            if isinstance(flags, dict): flags = flags.get('bits', [])
            if isinstance(flags, int):
                position_ok = position_ok and bool(flags & 16) and bool(flags & 32) and not bool(flags & 128)
            else:
                position_ok = position_ok and 'POS_HORIZ_ABS' in str(flags) and 'POS_VERT_ABS' in str(flags) and 'CONST_POS_MODE' not in str(flags)
        t['position_hold_available'] = bool(position_ok)
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
        self.checked_command("MAV_CMD_DO_REPOSITION", (-1, 1, 0, float('nan'), lat, lon, alt))
        return True

    def set_speed(self, speed, kind=1):
        # MAV_CMD_DO_CHANGE_SPEED = 178; p1=type (1=ground speed), p2=speed m/s, p3=throttle(-1)
        self.checked_command("MAV_CMD_DO_CHANGE_SPEED", (kind, float(speed), -1, 0, 0, 0, 0))
        return True

    def set_servo(self, channel=1, pwm=2000):
        # MAV_CMD_DO_SET_SERVO = 183 (releases payload via PWM relay / actuator)
        self.checked_command("MAV_CMD_DO_SET_SERVO", (channel, pwm, 0, 0, 0, 0, 0))
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
        if not self._mission_reply(d):
            return None
        return d["status"]["time"]["counter"], d["message"]["seq"]

    def _mack(self):
        d = _get(f"{_VEH}/MISSION_ACK")
        if not self._mission_reply(d):
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
            if self.interrupted.is_set():
                raise FlightControllerError("Mission operation interrupted by a flight control")
            # ACK => done (accepted) or failed (error type)
            ack = self._mack()
            if ack is not None and ack[0] != base_ack_ctr:
                if ack[1] == "MAV_MISSION_ACCEPTED":
                    if len(sent) != n:
                        return {"ok": False, "error": "Premature mission acknowledgement; upload not verified"}
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

    # QGC is an independent client. Never kill/restart it to refresh its editor.
    def sync_qgc(self):
        return

    @staticmethod
    def _enum(value):
        return value.get('type') if isinstance(value, dict) else value

    def _mission_reply(self, record):
        if not record or not _fresh(record):
            return False
        m = record.get('message', {})
        return (m.get('target_system') == GCS_SYS and m.get('target_component') == GCS_COMP
                and self._enum(m.get('mission_type')) in (0, 'MAV_MISSION_TYPE_MISSION'))

    def _wait_record(self, name, baseline, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.interrupted.is_set() and name.startswith('MISSION_'):
                raise FlightControllerError('Mission operation interrupted by a flight control')
            d = _get(f'{_VEH}/{name}', timeout=1)
            if d and _fresh(d) and d.get('status', {}).get('time', {}).get('counter') != baseline:
                if predicate(d['message']):
                    return d
            time.sleep(.08)
        raise FlightControllerError(f'{name} timed out; result is unknown. Check aircraft state before retrying.')

    @staticmethod
    def _counter(d):
        return (d or {}).get('status', {}).get('time', {}).get('counter')

    def checked_command(self, command, params=(0,)*7, emergency=False):
        # Emergency bypasses normal command serialization. Never retry a potentially
        # non-idempotent flight action automatically after an ambiguous timeout.
        acquired = False
        if not emergency:
            acquired = self.command_lock.acquire(blocking=False)
            if not acquired:
                raise FlightControllerError('Another flight command is pending')
        try:
            baseline = self._counter(_get(f'{_VEH}/COMMAND_ACK'))
            self._cmd(command, params)
            d = self._wait_record('COMMAND_ACK', baseline, lambda m:
                self._enum(m.get('command')) == command
                and m.get('target_system') == GCS_SYS and m.get('target_component') == GCS_COMP)
            result = self._enum(d['message'].get('result'))
            if result not in (0, 5, 'MAV_RESULT_ACCEPTED', 'MAV_RESULT_IN_PROGRESS'):
                raise FlightControllerError(f'Aircraft rejected command: {result}')
            return {'ok': True, 'accepted': True, 'result': result}
        finally:
            if acquired: self.command_lock.release()

    def download_mission(self):
        baseline = self._counter(_get(f'{_VEH}/MISSION_COUNT'))
        _post({'type': 'MISSION_REQUEST_LIST', 'target_system': TGT_SYS,
               'target_component': TGT_COMP, 'mission_type': {'type': 'MAV_MISSION_TYPE_MISSION'}})
        reply = self._wait_record('MISSION_COUNT', baseline, lambda m:
            m.get('target_system') == GCS_SYS and m.get('target_component') == GCS_COMP
            and self._enum(m.get('mission_type')) in (0, 'MAV_MISSION_TYPE_MISSION'))
        count = int(reply['message']['count'])
        if not 0 <= count <= 2000:
            raise FlightControllerError('Invalid aircraft mission count')
        items = []
        for seq in range(count):
            baseline = self._counter(_get(f'{_VEH}/MISSION_ITEM_INT'))
            _post({'type': 'MISSION_REQUEST_INT', 'seq': seq, 'target_system': TGT_SYS,
                   'target_component': TGT_COMP, 'mission_type': {'type': 'MAV_MISSION_TYPE_MISSION'}})
            d = self._wait_record('MISSION_ITEM_INT', baseline, lambda m:
                m.get('seq') == seq and m.get('target_system') == GCS_SYS
                and m.get('target_component') == GCS_COMP
                and self._enum(m.get('mission_type')) in (0, 'MAV_MISSION_TYPE_MISSION'))
            items.append(d['message'])
        _post({'type': 'MISSION_ACK', 'target_system': TGT_SYS, 'target_component': TGT_COMP,
               'mavtype': {'type': 'MAV_MISSION_ACCEPTED'}, 'mission_type': {'type': 'MAV_MISSION_TYPE_MISSION'}})
        return items, reply['message'].get('opaque_id')

    def verify_uploaded(self):
        if not self.uploaded_items:
            raise FlightControllerError('Upload a mission from this workspace first')
        actual, mission_id = self.download_mission()
        matches = len(actual) == len(self.uploaded_items)
        for want, got in zip(self.uploaded_items, actual):
            expected_frame = want.get('frame', 'MAV_FRAME_GLOBAL_RELATIVE_ALT')
            frame = self._enum(got.get('frame'))
            if frame == 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT': frame = 'MAV_FRAME_GLOBAL_RELATIVE_ALT'
            matches = matches and self._enum(got.get('command')) == want['command'] and frame == expected_frame
            for key, value in [('x', round(want.get('lat', 0)*1e7)), ('y', round(want.get('lon', 0)*1e7)),
                               ('z', want.get('alt', 0)), *[(f'param{i}', want.get(f'p{i}', 0)) for i in range(1, 5)]]:
                try:
                    a, b = float(got.get(key, 0)), float(value)
                    matches = matches and (math.isclose(a, b, rel_tol=0, abs_tol=.01) or (math.isnan(a) and math.isnan(b)))
                except (TypeError, ValueError): matches = False
        if not matches:
            self.mission_token = None
            raise FlightControllerError('Aircraft mission changed or differs from draft. Review and upload again.')
        self.mission_id = mission_id or None

    def upload_verified(self, items):
        if not self.mission_lock.acquire(blocking=False):
            raise FlightControllerError('Another mission transfer is in progress')
        try:
            self.interrupted.clear()
            self.mission_token = None
            result = self.upload_mission(items)
            if not result.get('ok'): return result
            self.uploaded_items = items
            self.verify_uploaded()
            if self.interrupted.is_set():
                raise FlightControllerError("Upload verification interrupted")
            self.mission_token = uuid.uuid4().hex
            return dict(result, verified=True, mission_token=self.mission_token)
        finally:
            self.mission_lock.release()

    def clear_verified(self):
        self.execute('hold')
        if not self.mission_lock.acquire(blocking=False):
            raise FlightControllerError('Hold requested; mission transfer is stopping. Retry clear after it finishes.')
        try:
            self.interrupted.clear()
            self.mission_token = None
            self.uploaded_items = None
            baseline = self._counter(_get(f'{_VEH}/MISSION_ACK'))
            _post({'type':'MISSION_CLEAR_ALL', 'target_system':TGT_SYS, 'target_component':TGT_COMP,
                   'mission_type':{'type':'MAV_MISSION_TYPE_MISSION'}})
            ack = self._wait_record('MISSION_ACK', baseline, lambda m:
                m.get('target_system') == GCS_SYS and m.get('target_component') == GCS_COMP
                and self._enum(m.get('mission_type')) in (0, 'MAV_MISSION_TYPE_MISSION'))
            if self._enum(ack['message'].get('mavtype')) not in (0, 'MAV_MISSION_ACCEPTED'):
                raise FlightControllerError('Aircraft rejected mission clear')
            items, _ = self.download_mission()
            if items: raise FlightControllerError('Aircraft still has mission items after clear')
            return {'ok':True, 'verified':True}
        finally:
            self.mission_lock.release()

    def inspect_mission(self):
        if not self.mission_lock.acquire(blocking=False):
            raise FlightControllerError('Another mission transfer is in progress')
        try:
            self._heartbeat()
            self.interrupted.clear()
            items, mission_id = self.download_mission()
            # MAVLink uses NaN for unspecified yaw/parameters. JSON does not.
            def json_value(value):
                if isinstance(value, float) and not math.isfinite(value):
                    return None
                if isinstance(value, dict):
                    return {key: json_value(v) for key, v in value.items()}
                if isinstance(value, list):
                    return [json_value(v) for v in value]
                return value
            return {'ok': True, 'items': json_value(items), 'mission_id': mission_id}
        finally:
            self.mission_lock.release()

    def _require_navigation(self, require_home=False):
        t = self.telemetry()
        if not t.get('connected') or not t.get('position_hold_available'):
            raise FlightControllerError('Navigation unavailable: valid live position required')
        if require_home:
            lat, lon = t.get('home_lat'), t.get('home_lon')
            if (not isinstance(lat, (int, float)) or not isinstance(lon, (int, float))
                    or not math.isfinite(lat) or not math.isfinite(lon)
                    or not -90 <= lat <= 90 or not -180 <= lon <= 180):
                raise FlightControllerError('Aircraft home position unavailable')
        return t

    def execute(self, action, token=None):
        if action in ('start', 'resume'):
            if not self.mission_lock.acquire(blocking=False):
                raise FlightControllerError('Another mission transfer is in progress')
            try:
                self.interrupted.clear()
                if not token or token != self.mission_token:
                    raise FlightControllerError('Mission is not verified for this draft. Upload and review first.')
                self.verify_uploaded()
                if not _armed(self._heartbeat()['message']):
                    raise FlightControllerError('Arm separately and confirm armed state before starting')
                if self.interrupted.is_set(): raise FlightControllerError('Mission start interrupted')
                self._require_navigation()
                if action == 'start':
                    self.checked_command('MAV_CMD_DO_SET_MISSION_CURRENT', (0, 1, 0, 0, 0, 0, 0))
                if self.interrupted.is_set(): raise FlightControllerError('Mission start interrupted')
                # Recheck after the mission-current acknowledgement: position may have been lost.
                self._require_navigation()
                if self.interrupted.is_set(): raise FlightControllerError('Mission start interrupted')
                return self.checked_command('MAV_CMD_DO_SET_MODE', (1, MAIN_AUTO, SUB_MISSION, 0, 0, 0, 0))
            finally:
                self.mission_lock.release()
        if action in ('hold', 'manual', 'rtl', 'land', 'emergency_stop', 'disarm'):
            self.interrupted.set()
        if action == 'rtl':
            self._require_navigation(require_home=True)
        if action == 'hold':
            if not self.telemetry().get('position_hold_available'):
                raise FlightControllerError('Position hold unavailable: waiting for a valid global position estimate')
        if action == 'manual' and os.environ.get('GCS_MODE') == 'real' and not self.telemetry().get('rc_available'):
            raise FlightControllerError('RC input unavailable or unverified; check receiver before takeover')
        commands = {
            'arm': ('MAV_CMD_COMPONENT_ARM_DISARM', (1, 0, 0, 0, 0, 0, 0)),
            'disarm': ('MAV_CMD_COMPONENT_ARM_DISARM', (0, 0, 0, 0, 0, 0, 0)),
            'emergency_stop': ('MAV_CMD_COMPONENT_ARM_DISARM', (0, 21196, 0, 0, 0, 0, 0)),
            'hold': ('MAV_CMD_DO_SET_MODE', (1, MAIN_AUTO, SUB_LOITER, 0, 0, 0, 0)),
            'manual': ('MAV_CMD_DO_SET_MODE', (1, TOGGLE_MANUAL_MAIN, 0, 0, 0, 0, 0)),
            'rtl': ('MAV_CMD_DO_SET_MODE', (1, MAIN_AUTO, SUB_RTL, 0, 0, 0, 0)),
            'land': ('MAV_CMD_DO_SET_MODE', (1, MAIN_AUTO, 6, 0, 0, 0, 0)),
        }
        if action not in commands: raise ValueError('Unknown flight action')
        command, params = commands[action]
        result = self.checked_command(command, params, emergency=action == 'emergency_stop')
        if action == 'manual': result['mode_main'] = TOGGLE_MANUAL_MAIN
        return result
