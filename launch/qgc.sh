#!/bin/bash
# Launch QGroundControl for the Chattogram SITL stack.
#
# Zero configuration required. PX4's default GCS link sends telemetry to
# 127.0.0.1:14550, which is exactly the port QGC's factory AutoConnect-UDP
# listens on -- so QGC connects to the drone on its own. The web GCS runs
# independently via mavlink2rest on :14555 (fed by a separate PX4 push link),
# so the two never contend for a port.
#
# Usage: launch/qgc.sh
set -u

# Prefer the installed QGroundControl 5.1.4 launcher. That build needs glibc 2.38,
# so ~/.local/bin/qgroundcontrol starts it with a private glibc 2.39. Fall back to
# the v4.4.5 AppImage, which runs on the system glibc. Override with QGC_APPIMAGE.
QGC="${QGC_APPIMAGE:-}"
if [ -z "$QGC" ]; then
  if [ -x "$HOME/.local/bin/qgroundcontrol" ]; then
    QGC="$HOME/.local/bin/qgroundcontrol"
  else
    for c in "$HOME/QGroundControl-v4.4.5.AppImage" "$HOME/QGroundControl.AppImage"; do
      [ -x "$c" ] && { QGC="$c"; break; }
    done
  fi
fi

if [ -z "$QGC" ] || [ ! -x "$QGC" ]; then
  echo "QGroundControl not found (looked for ~/.local/bin/qgroundcontrol, then v4.4.5)." >&2
  echo "Set QGC_APPIMAGE=/path/to/QGroundControl.AppImage or chmod +x it." >&2
  exit 1
fi

# The relay (mav_hub or PX4's UDP link) feeds QGC on :14550. Pixhawk
# AutoConnect would also open /dev/ttyACM* and steal the flight controller
# from that relay. QGC 5 stores settings under QGroundControl/; older
# builds use QGroundControl.org/. Set QGC_AUTOCONNECT_PIXHAWK=1 to keep
# the direct USB link.
if [ "${QGC_AUTOCONNECT_PIXHAWK:-0}" != "1" ]; then
  CONF="${XDG_CONFIG_HOME:-$HOME/.config}"
  for INI in "$CONF/QGroundControl/QGroundControl.ini" \
             "$CONF/QGroundControl.org/QGroundControl.ini"; do
    [ -f "$INI" ] || continue
    python3 - "$INI" << 'PY'
import sys
path = sys.argv[1]
lines = open(path).read().splitlines()
out = []
in_section = False
seen = False
wrote_px = wrote_udp = False
for line in lines:
    if line.startswith("[") and line.endswith("]"):
        if in_section:
            if not wrote_px:
                out.append("autoConnectPixhawk=false")
            if not wrote_udp:
                out.append("autoConnectUDP=true")
        in_section = line == "[AutoConnect]"
        seen = seen or in_section
        out.append(line)
        wrote_px = wrote_udp = False
        continue
    if in_section and line.startswith("autoConnectPixhawk="):
        out.append("autoConnectPixhawk=false")
        wrote_px = True
        continue
    if in_section and line.startswith("autoConnectUDP="):
        out.append("autoConnectUDP=true")
        wrote_udp = True
        continue
    out.append(line)
if in_section:
    if not wrote_px:
        out.append("autoConnectPixhawk=false")
    if not wrote_udp:
        out.append("autoConnectUDP=true")
elif not seen:
    if out and out[-1] != "":
        out.append("")
    out.extend(["[AutoConnect]", "autoConnectPixhawk=false", "autoConnectUDP=true"])
open(path, "w").write("\n".join(out) + "\n")
PY
    echo "  -> Pixhawk USB AutoConnect off in $INI"
  done
fi

echo "Launching QGroundControl ($QGC)"
echo "  -> auto-connects to PX4 on :14550 (web GCS is independent on :14555)"
setsid "$QGC" > "$HOME/qgc.log" 2>&1 &
echo "  started (log: $HOME/qgc.log)"
