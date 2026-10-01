#!/usr/bin/env python3
"""Stage-2 RC bridge: read RC_CHANNELS from a real Pixhawk (wireless receiver
on its RC port) and forward as MANUAL_CONTROL to PX4 SITL, so the physical
controller flies the Gazebo-simulated drone.

Why this exists
---------------
In Stage 2 the drone is the *simulated* one (PX4 SITL + Gazebo), but the pilot
wants to use the real RC transmitter + receiver. The receiver is wired to the
real Pixhawk's RC port (e.g. CRSF on GPS2), and the Pixhawk is connected to the
laptop over USB (/dev/ttyACM0). This bridge:

  receiver --wireless--> Pixhawk --USB/MAVLink--> [rc_to_sitl.py] --HTTP--> mavlink2rest --MAVLink--> PX4 SITL

It reads RC_CHANNELS (PWM 1000-2000) from the Pixhawk, maps the first 4
channels to MANUAL_CONTROL (x=pitch, y=roll, z=throttle, r=yaw), and POSTs the
message to mavlink2rest (the same REST path the web GCS already uses for all
its commands), which forwards it to PX4 SITL.

Requires SITL to have COM_RC_IN_MODE=1 (MAVLink manual control enabled); the
stage-2 launcher sets this automatically.

Usage
-----
    python3 rc_to_sitl.py [PIXHAWK_DEV] [BAUD]
    # defaults: /dev/ttyACM0  115200
    # override MAVLink REST target: RC_M2R=http://127.0.0.1:8088
    # override send rate:          RC_RATE_HZ=20
    # reverse channels:           RC_REV_ROLL=1 RC_REV_PITCH=1 RC_REV_YAW=1

Channel map (AETR / mode-2 convention, configurable)
    ch1=roll  ch2=pitch  ch3=throttle  ch4=yaw  ch5+=aux (mode switches, etc.)

Exit: Ctrl-C, or when the Pixhawk link drops for >5 s.
"""
import json
import os
import sys
import time
import urllib.request

from pymavlink import mavutil

# per-channel reversal (1 = invert sign). Throttle is never reversed here.
REV_ROLL  = os.environ.get("RC_REV_ROLL",  "0") == "1"
REV_PITCH = os.environ.get("RC_REV_PITCH", "0") == "1"
REV_YAW   = os.environ.get("RC_REV_YAW",   "0") == "1"


def post_manual_control(m2r, target_sys, x, y, z, r, buttons=0):
    """POST a MANUAL_CONTROL message to mavlink2rest -> PX4 SITL.
    x=pitch, y=roll, z=throttle[0..1000], r=yaw  (all -1000..1000 except z)."""
    body = json.dumps({"header": {"system_id": 254, "component_id": 0, "sequence": 0},
                       "message": {"type": "MANUAL_CONTROL", "target": target_sys,
                                   "x": int(x), "y": int(y), "z": int(z), "r": int(r),
                                   "buttons": buttons}}).encode()
    req = urllib.request.Request(m2r + "/v1/mavlink", data=body,
                                 headers={"Content-Type": "application/json"},
                                 method="POST")
    try:
        urllib.request.urlopen(req, timeout=0.5)
    except Exception:
        pass  # mavlink2rest may be briefly down during SITL boot; keep going


def pwm_to_axis(pwm, reverse=False):
    """PWM 1000-2000 (trim 1500) -> -1000..1000."""
    v = (pwm - 1500) * 2          # 500 span -> 1000 span
    if reverse:
        v = -v
    return max(-1000, min(1000, int(v)))


def pwm_to_throttle(pwm):
    """PWM 1000-2000 -> 0..1000 (throttle is non-negative)."""
    return max(0, min(1000, int((pwm - 1000) * 1)))


def main():
    DEV = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("RC_PIXHAWK_DEV", "/dev/ttyACM0")
    BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else int(os.environ.get("RC_PIXHAWK_BAUD", "115200"))
    M2R = os.environ.get("RC_M2R", "http://127.0.0.1:8088")
    RATE_HZ = float(os.environ.get("RC_RATE_HZ", "20"))
    TARGET_SYS = int(os.environ.get("RC_TARGET_SYS", "1"))

    print(f"[rc_bridge] source: {DEV} @ {BAUD}  ->  SITL via {M2R}  ({RATE_HZ:.0f} Hz)")
    print(f"[rc_bridge] reversals: roll={REV_ROLL} pitch={REV_PITCH} yaw={REV_YAW}")

    # --- connect to the real Pixhawk (source of RC data) --------------------
    m = mavutil.mavlink_connection(DEV, baud=BAUD, source_system=254, source_component=0)
    print(f"[rc_bridge] waiting for HEARTBEAT from Pixhawk on {DEV}...")
    if m.wait_heartbeat(timeout=8) is None:
        print(f"[rc_bridge] NO HEARTBEAT from Pixhawk on {DEV}. Is it powered & connected?")
        print(f"[rc_bridge] If the receiver is on the Pixhawk, the Pixhawk must be powered")
        print(f"[rc_bridge] (USB power alone is fine — it need NOT be armed).")
        sys.exit(1)
    print(f"[rc_bridge] Pixhawk link ok (sys={m.target_system} comp={m.target_component})")

    # ask the Pixhawk to stream RC channels at ~the rate we forward
    try:
        m.mav.request_data_stream_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, int(RATE_HZ) or 5)
    except Exception:
        pass

    # --- main loop: RC_CHANNELS -> MANUAL_CONTROL ---------------------------
    period = 1.0 / RATE_HZ
    last_rc_t = time.time()
    last_post = 0.0
    rc_msgs = 0
    zero_streak = 0
    print(f"[rc_bridge] forwarding RC -> SITL. Move your sticks! (Ctrl-C to stop)")

    try:
        while True:
            msg = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=1.0)
            now = time.time()

            if msg is not None:
                rc_msgs += 1
                last_rc_t = now
                ch = [getattr(msg, f"chan{i}_raw", 0) for i in range(1, 17)]
                nonzero = sum(1 for v in ch[:4] if v != 0)
                if nonzero == 0:
                    zero_streak += 1
                    if zero_streak == 1 or zero_streak % 100 == 0:
                        print(f"[rc_bridge] WARNING: RC channels all 0 — receiver not feeding "
                              f"data (unbound? unpowered? wrong port?). Pixhawk CRSF link is "
                              f"configured but no wireless frames arriving.")
                else:
                    if zero_streak > 0:
                        print(f"[rc_bridge] RC link LIVE again ({nonzero}/4 stick channels active)")
                    zero_streak = 0

                roll  = pwm_to_axis(ch[0], REV_ROLL)
                pitch = pwm_to_axis(ch[1], REV_PITCH)
                thr   = pwm_to_throttle(ch[2])
                yaw   = pwm_to_axis(ch[3], REV_YAW)
                buttons = 0x01 if ch[4] > 1500 else 0   # aux ch5 -> arm button

                if now - last_post >= period:
                    post_manual_control(M2R, TARGET_SYS, pitch, roll, thr, yaw, buttons)
                    last_post = now
            else:
                if now - last_rc_t > 5.0:
                    print(f"[rc_bridge] no RC_CHANNELS from Pixhawk for {now-last_rc_t:.0f}s "
                          f"— Pixhawk link lost?")
                    last_rc_t = now

            if msg is None and now - last_post > 1.0:
                post_manual_control(M2R, TARGET_SYS, 0, 0, 0, 0, 0)

    except KeyboardInterrupt:
        print("\n[rc_bridge] stopped")
    finally:
        m.close()


if __name__ == "__main__":
    main()
