# HM30 front camera and live connection checks

The web GCS can show the drone's front camera through the existing RTSP-to-MJPEG
bridge. A separate SIYI viewing app is not required for this path. Camera video
and MAVLink telemetry have separate connection status.

## Laptop connection

SIYI documents LAN video output to a PC using the ground unit's 4-pin-to-RJ45
cable and a laptop network interface on `192.168.144.X`. A USB Ethernet adapter
can provide the laptop's RJ45 port. Its Type-C video instructions use USB
networking/tethering; the presence of USB telemetry alone does not establish
camera network access. Consult the instructions for the unit's firmware before
changing its USB/video mode.

Sources: [HM30 manual v1.2, sections 6.5–6.6](https://siyi.biz/siyi_file/HM30/HM30_User_Manual_en_v1.2.pdf),
[current HM30 downloads](https://www.siyi.biz/en/product/image-digital-link/hm30/download/).

This project's configured camera URL is:

```text
rtsp://192.168.144.25:8554/main.264
```

That is the documented address for the original SIYI IP camera. Other camera
models can use a different stream path; set `FPV_RTSP_URL` to the camera's actual
RTSP URL when launching the bridge.

## Read-only checks on the laptop

Run these in the laptop's normal terminal. They do not arm the aircraft or
change flight settings:

```bash
ip -br address
ip route get 192.168.144.25
timeout 10s ffprobe -v error -rtsp_transport tcp \
  -select_streams v:0 -show_entries stream=codec_name,width,height \
  -of json rtsp://192.168.144.25:8554/main.264
curl --max-time 3 --noproxy '*' http://127.0.0.1:8091/healthz
curl --max-time 3 --noproxy '*' http://127.0.0.1:8080/api/mode
curl --max-time 3 --noproxy '*' http://127.0.0.1:8080/api/camera/status
```

- The route must reach the HM30 network through the intended adapter.
- `ffprobe` should report a video codec and dimensions. A timeout can mean
  unreachable camera, incorrect stream URL, unpowered camera, or a down radio link.
- Bridge health reports `fresh: true` only while recent camera frames arrive.
- The web GCS must report `mode: real` to use the front-camera pane.

`launch/start_real.sh` already starts the camera bridge and configures both video
proxy routes. It restarts telemetry processes as well, so use it when starting a
grounded hardware session. Changes in Python processes require a restart; reload
the browser for UI changes. The existing `launch/restart_gcs.sh` assumes simulation
unless given appropriate environment settings and does not restart the RTSP bridge.

## Current behavior and limits

The real camera pane starts automatically and reports live, waiting, paused, or
bridge unavailable. Frames older than two seconds are replaced by a waiting
image. The default FFmpeg decoder restarts after five seconds without a complete
frame. These indicators describe the received stream, not camera orientation.

The GCS requires a heartbeat received within five seconds to report a live
aircraft link. Successful command delivery is displayed as a request; the flight
controller's acceptance must still be checked in live flight status. In real
mode, Start requires an already armed aircraft and does not arm it implicitly.
Waypoint mission execution remains the existing PX4 mission implementation;
these changes do not implement visual navigation or obstacle avoidance.

Offline regression checks:

```bash
python3 -m unittest discover -s gcs/tests -v
node gcs/tests/test_ui_commands.cjs
```

These checks mock network and aircraft operations. They do not validate the
physical USB/radio link, camera stream, or an autonomous flight.
