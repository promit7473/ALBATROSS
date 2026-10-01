#!/bin/bash
# After SITL is up, reuse the SIYI radio as an RC stick source for the SIM
# (MANUAL_CONTROL -> mavlink2rest :8088). Stop forwarding real telemetry onto
# :14550/:14555 so QGC / the web GCS own the SITL vehicle. The real Pixhawk
# stays disarmed.
set -u
ROOT="${MARITIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
sleep 14
python3 - <<'PY'
import os, signal
key = "/gcs/" + "mav_hub.py"
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\x00", b" ").decode()
    except Exception:
        continue
    if key in cmd and "python" in cmd:
        os.kill(int(pid), signal.SIGTERM)
PY
sleep 1
export MAV_HUB_FORWARD=0 MAV_HUB_SITL_RC=1 PYTHONUNBUFFERED=1
exec python3 "$ROOT/gcs/mav_hub.py" >> "$HOME/mav_hub.log" 2>&1
