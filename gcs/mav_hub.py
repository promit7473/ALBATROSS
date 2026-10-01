#!/usr/bin/env python3
"""MAVLink hub for the real aircraft.

Fans packets between ONE vehicle endpoint and the two local GCS ports:

  vehicle  -->  127.0.0.1:14550  (QGroundControl AutoConnect UDP)
           -->  127.0.0.1:14555  (mavlink2rest / web GCS)
  QGC/web  -->  vehicle

Vehicle endpoint (first that works):
  1. Pixhawk USB serial (PIXHAWK_DEV), if present
  2. SIYI HM30 UDP datalink  (SIYI_HOST:SIYI_PORT, default 192.168.144.12:19856)

RC stays native on the Pixhawk (receiver on the RC port). This process is
telemetry / GCS only — it does not inject stick commands.
"""
import glob
import json
import os
import select
import socket
import sys
import time
import urllib.request

try:
    import serial
except ImportError:
    serial = None

QGC_ADDR = (os.environ.get("QGC_HOST", "127.0.0.1"),
            int(os.environ.get("QGC_PORT", "14550")))
M2R_ADDR = (os.environ.get("M2R_HOST", "127.0.0.1"),
            int(os.environ.get("M2R_PORT", "14555")))
SIYI_HOST = os.environ.get("SIYI_HOST", "192.168.144.12")
SIYI_PORT = int(os.environ.get("SIYI_PORT", "19856"))
LOCAL_SIYI_PORT = int(os.environ.get("SIYI_LOCAL_PORT", "19856"))
PIXHAWK_BAUD = int(os.environ.get("PIXHAWK_BAUD", "115200"))
FORWARD = os.environ.get("MAV_HUB_FORWARD", "1") != "0"
SITL_RC = os.environ.get("MAV_HUB_SITL_RC", "0") == "1"
M2R_REST = os.environ.get("RC_M2R", "http://127.0.0.1:8088")


def _is_serial_tty(path):
    """True for a ttyACM/ttyUSB/ttyS node. /dev/pixhawk is sometimes a raw USB
    device node (bus/usb/...), which is not a MAVLink serial port."""
    try:
        if not path or not os.path.exists(path):
            return False
        base = os.path.basename(os.path.realpath(path))
    except OSError:
        return False
    return base.startswith(("ttyACM", "ttyUSB", "ttyS"))


def find_pixhawk(explicit=None):
    if explicit:
        return explicit if _is_serial_tty(explicit) else None
    if _is_serial_tty("/dev/pixhawk"):
        return "/dev/pixhawk"
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        name = os.path.basename(path).lower()
        if any(k in name for k in ("px4", "pixhawk", "auterion", "fmu", "holybro")):
            if _is_serial_tty(path):
                return path
    for path in sorted(glob.glob("/dev/ttyACM*")):
        if _is_serial_tty(path):
            return path
    return None


_rc_decoder = None
_last_rc_post = 0.0


def _sitl_rc_from(buf):
    """When SITL_RC=1, decode RC_CHANNELS from the real aircraft and push
    MANUAL_CONTROL to mavlink2rest (PX4 SITL). Real motors are not armed."""
    global _rc_decoder, _last_rc_post
    if not SITL_RC or not buf:
        return
    try:
        from pymavlink.dialects.v20 import common as ml
        if _rc_decoder is None:
            _rc_decoder = ml.MAVLink(None)
            _rc_decoder.robust_parsing = True
        now = time.time()
        for i in range(len(buf)):
            msg = _rc_decoder.parse_char(buf[i:i + 1])
            if msg is None or msg.get_type() != "RC_CHANNELS":
                continue
            if now - _last_rc_post < 0.05:
                continue
            ch = [int(getattr(msg, f"chan{i}_raw", 1500) or 1500) for i in range(1, 5)]
            roll = max(-1000, min(1000, (ch[0] - 1500) * 2))
            pitch = max(-1000, min(1000, (ch[1] - 1500) * 2))
            thr = max(0, min(1000, (ch[2] - 1000)))
            yaw = max(-1000, min(1000, (ch[3] - 1500) * 2))
            body = json.dumps({
                "header": {"system_id": 254, "component_id": 0, "sequence": 0},
                "message": {"type": "MANUAL_CONTROL", "target": 1,
                            "x": int(pitch), "y": int(roll), "z": int(thr),
                            "r": int(yaw), "buttons": 0}
            }).encode()
            req = urllib.request.Request(
                M2R_REST + "/v1/mavlink", data=body,
                headers={"Content-Type": "application/json"}, method="POST")
            try:
                urllib.request.urlopen(req, timeout=0.3)
            except Exception:
                pass
            _last_rc_post = now
    except Exception:
        pass


def _gcs_heartbeat():
    """MAVLink v2 HEARTBEAT from a GCS (sys 255 / comp 190)."""
    try:
        from pymavlink.dialects.v20 import common as ml
        mav = ml.MAVLink(None, srcSystem=255, srcComponent=190)
        msg = mav.heartbeat_encode(
            ml.MAV_TYPE_GCS, ml.MAV_AUTOPILOT_INVALID, 0, 0, ml.MAV_STATE_ACTIVE)
        return msg.pack(mav)
    except Exception:
        return None


def main():
    explicit = os.environ.get("PIXHAWK_DEV") or (sys.argv[1] if len(sys.argv) > 1 else "")
    serial_dev = find_pixhawk(explicit or None)
    ser = None
    if serial_dev:
        if serial is None:
            print("[mav_hub] pyserial missing — USB path skipped", flush=True)
        else:
            try:
                ser = serial.Serial(serial_dev, PIXHAWK_BAUD, timeout=0)
                print(f"[mav_hub] vehicle = serial {serial_dev} @ {PIXHAWK_BAUD}",
                      flush=True)
            except Exception as e:
                print(f"[mav_hub] serial open failed ({e}); using SIYI UDP", flush=True)
                ser = None

    veh = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    veh.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    veh.bind(("0.0.0.0", LOCAL_SIYI_PORT))
    siyi_peer = (SIYI_HOST, SIYI_PORT)

    gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gcs.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    gcs.bind(("0.0.0.0", 0))  # replies from QGC / mavlink2rest land here

    if ser is None:
        print(f"[mav_hub] vehicle = SIYI UDP {siyi_peer[0]}:{siyi_peer[1]} "
              f"(local :{LOCAL_SIYI_PORT})", flush=True)
    print(f"[mav_hub] forward={FORWARD} sitl_rc={SITL_RC} "
          f"-> QGC {QGC_ADDR[0]}:{QGC_ADDR[1]}  "
          f"mavlink2rest {M2R_ADDR[0]}:{M2R_ADDR[1]}", flush=True)

    hb = _gcs_heartbeat()
    last_hb = 0.0
    last_siyi_poke = 0.0
    n_up = n_down = 0
    last_report = time.time()

    def to_gcs(buf):
        nonlocal n_down
        if not buf:
            return
        _sitl_rc_from(buf)
        if FORWARD:
            gcs.sendto(buf, QGC_ADDR)
            gcs.sendto(buf, M2R_ADDR)
        n_down += 1

    def to_vehicle(buf):
        nonlocal n_up
        if not buf:
            return
        if ser is not None:
            try:
                ser.write(buf)
            except Exception as e:
                print(f"[mav_hub] serial write: {e}", flush=True)
        else:
            veh.sendto(buf, siyi_peer)
        n_up += 1

    fds = [gcs]
    if ser is not None:
        fds.append(ser)
    else:
        fds.append(veh)
    last_serial_try = time.time()
    last_serial_fail_log = 0.0

    def drop_serial():
        nonlocal ser
        if ser is None:
            return
        try:
            ser.close()
        except Exception:
            pass
        if ser in fds:
            fds.remove(ser)
        ser = None
        if veh not in fds:
            fds.append(veh)
        print("[mav_hub] serial closed; using SIYI UDP", flush=True)

    def try_serial():
        """Pick up a Pixhawk that was plugged in, or busy, after startup."""
        nonlocal ser, serial_dev, last_serial_try, last_serial_fail_log
        if ser is not None or serial is None:
            return
        now = time.time()
        if now - last_serial_try < 3.0:
            return
        last_serial_try = now
        serial_dev = find_pixhawk(explicit or None)
        if not serial_dev:
            return
        try:
            opened = serial.Serial(serial_dev, PIXHAWK_BAUD, timeout=0)
        except Exception as e:
            if now - last_serial_fail_log > 30.0:
                print(f"[mav_hub] serial open failed ({e}); still on SIYI UDP",
                      flush=True)
                last_serial_fail_log = now
            return
        ser = opened
        if veh in fds:
            fds.remove(veh)
        fds.append(ser)
        print(f"[mav_hub] vehicle = serial {serial_dev} @ {PIXHAWK_BAUD}",
              flush=True)

    while True:
        now = time.time()
        try_serial()
        if hb and now - last_hb > 1.0:
            to_vehicle(hb)
            last_hb = now
        # SIYI only starts streaming to a peer after it has heard from it.
        # Leave that socket quiet while the USB flight controller is the link.
        if ser is None and now - last_siyi_poke > 1.0:
            veh.sendto(hb or b"\xfd", siyi_peer)
            last_siyi_poke = now
        if now - last_report > 5.0:
            link = f"serial {serial_dev}" if ser is not None else (
                f"peer {siyi_peer[0]}:{siyi_peer[1]}")
            print(f"[mav_hub] up {n_up} pkts  down {n_down} pkts  {link}",
                  flush=True)
            n_up = n_down = 0
            last_report = now

        r, _, _ = select.select(fds, [], [], 0.2)
        for fd in r:
            if fd is veh:
                buf, addr = veh.recvfrom(2048)
                if addr[0].startswith("192.168.144."):
                    siyi_peer = addr
                to_gcs(buf)
            elif fd is gcs:
                buf, _addr = gcs.recvfrom(2048)
                to_vehicle(buf)
            elif ser is not None and fd is ser:
                try:
                    buf = ser.read(4096)
                except Exception as e:
                    print(f"[mav_hub] serial read: {e}", flush=True)
                    drop_serial()
                    continue
                to_gcs(buf)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
