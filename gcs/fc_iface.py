"""Flight-controller interface abstraction.

Everything else in the GCS talks ONLY to this interface. Today: MAVLink via
mavlink2rest. Later: add a `DDSOffboardFC` with the same methods for the Jetson.
"""
import json
import os
import subprocess
import time
import urllib.request

M2R = "http://127.0.0.1:8088"
_VEH = "/v1/mavlink/vehicles/1/components/1/messages"
TGT_SYS, TGT_COMP = 1, 1

# PX4 custom main/sub modes for DO_SET_MODE
MAIN_AUTO = 4
SUB_MISSION, SUB_RTL, SUB_LOITER = 4, 5, 3


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
            subprocess.Popen([m2r, "-c", "udpin:0.0.0.0:14550", "-s", "0.0.0.0:8088", "--send-initial-heartbeats"],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print("[fc_iface] started background mavlink2rest on :8088")
        except Exception as e:
            print(f"[fc_iface] failed to start mavlink2rest: {e}")

ensure_mavlink2rest()


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
        return None


class FCInterface:
    def telemetry(self): raise NotImplementedError
    def arm(self, armed): raise NotImplementedError
    def set_mode(self, main, sub): raise NotImplementedError
    def rtl(self): raise NotImplementedError
    def start_mission(self): raise NotImplementedError
    def upload_mission(self, items): raise NotImplementedError
    def set_servo(self, channel, pwm): raise NotImplementedError


class MavlinkRestFC(FCInterface):
    # ---------------- telemetry ----------------
    def telemetry(self):
        gp = _get(f"{_VEH}/GLOBAL_POSITION_INT")
        hb = _get(f"{_VEH}/HEARTBEAT")
        hud = _get(f"{_VEH}/VFR_HUD")
        gps = _get(f"{_VEH}/GPS_RAW_INT")
        sysst = _get(f"{_VEH}/SYS_STATUS")
        t = {"connected": gp is not None or hb is not None}
        if gp:
            m = gp["message"]
            t.update(lat=m["lat"] / 1e7, lon=m["lon"] / 1e7,
                     alt_msl=m["alt"] / 1000.0, rel_alt=m["relative_alt"] / 1000.0,
                     hdg=m["hdg"] / 100.0, vx=m["vx"] / 100.0, vy=m["vy"] / 100.0)
        if hud:
            m = hud["message"]
            t.update(groundspeed=m.get("groundspeed"), climb=m.get("climb"))
        if gps:
            m = gps["message"]; ft = m.get("fix_type")
            t.update(sats=m.get("satellites_visible"),
                     fix=(ft.get("type") if isinstance(ft, dict) else ft))
        if sysst:
            m = sysst["message"]
            t.update(battery_v=m.get("voltage_battery", 0) / 1000.0,
                     battery_pct=m.get("battery_remaining"))
        if hb:
            m = hb["message"]; bm = m.get("base_mode", "")
            bm = bm if isinstance(bm, str) else json.dumps(bm)
            t["armed"] = "SAFETY_ARMED" in bm
            t["custom_mode"] = m.get("custom_mode")
        return t

    # ---------------- commands ----------------
    def _cmd(self, command, p=(0,)*7):
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

    def stop_mission(self):
        """Halt the active mission: hold/loiter in place (safe) and wipe the
        uploaded mission so the next Upload starts clean."""
        self.set_mode(MAIN_AUTO, SUB_LOITER)          # AUTO.LOITER -> hover in place
        time.sleep(0.2)
        _post({"type": "MISSION_CLEAR_ALL", "target_system": TGT_SYS,
               "target_component": TGT_COMP, "mission_type": {"type": "MAV_MISSION_TYPE_ALL"}})
        return True

    def start_mission(self):
        _post({"type": "MISSION_SET_CURRENT", "seq": 0, "target_system": TGT_SYS, "target_component": TGT_COMP})
        time.sleep(0.1)
        self.set_mode(MAIN_AUTO, SUB_MISSION)
        time.sleep(0.4)
        self.arm(True)
        return True

    def reposition(self, lat, lon, alt):
        # MAV_CMD_DO_REPOSITION = 192 (flies directly to coordinates in guided mode)
        self._cmd("MAV_CMD_DO_REPOSITION", (-1, 1, 0, float('nan'), lat, lon, alt))
        return True

    def set_servo(self, channel=1, pwm=2000):
        # MAV_CMD_DO_SET_SERVO = 183 (releases payload via PWM relay / actuator)
        self._cmd("MAV_CMD_DO_SET_SERVO", (channel, pwm, 0, 0, 0, 0, 0))
        return True

    def set_param(self, name, value, ptype="MAV_PARAM_TYPE_REAL32"):
        """Set a PX4 parameter (e.g. RTL_RETURN_ALT) via mavlink2rest PARAM_SET.
        mavlink2rest serialises the char[16] param_id as a plain string."""
        _post({"type": "PARAM_SET", "param_id": name, "param_value": float(value),
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
        n = len(items)
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
