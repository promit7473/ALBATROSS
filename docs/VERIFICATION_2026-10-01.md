# Hardware and interface verification — 2026-10-01

Overall result, updated after the direct USB connection: the web GCS and
QGroundControl now share the Pixhawk 6C on USB. The HM30 camera is live. GPS
has no fix, and the HM30 Ethernet port still carries no MAVLink, so wireless
telemetry and autonomous flight are not verified. No arming, mission upload,
mode change, or payload command was sent.

| Area | Observed result |
| --- | --- |
| LAN | Laptop `enp0s31f6` is up at `192.168.144.30/24`; route to HM30 is through this adapter. |
| HM30 ground unit | `192.168.144.12` answered both ICMP probes in the preceding connection check. |
| Camera | `/api/camera/status` reports fresh frames. Browser decoded 960 × 540 MJPEG (bridge resizes the previously verified 1280 × 720 HEVC source). |
| Camera controls | Browser verified enlarge, fullscreen, reset, and mouse drag resizing at a 1440 × 1000 viewport. |
| USB | Direct Type-C link enumerates as Auterion PX4 FMU v6C on `/dev/ttyACM0`. `gcs/mav_hub.py` owns that port and forwards it to QGC `:14550` and the web bridge `:14555`. |
| MAVLink relay | USB path is carrying traffic in both directions. HM30 UDP `192.168.144.12:19856` still returns no MAVLink packets; pings to `.11`, `.12`, and the camera `.25` succeed. |
| Vehicle discovery | Web `/api/telemetry` reports `connected: true`. Aircraft is disarmed, landed, in LOITER. Battery about 15.6 V. GPS fix is `NO_GPS`, 0 satellites, so latitude and longitude are not published. |
| QGC | Running on UDP 14550 with Pixhawk USB AutoConnect off, so it does not take `/dev/ttyACM0` from the relay. |
| Aircraft 3D | Cesium initializes without uncaught JavaScript errors. Model is hidden without GPS. Injected test GPS moves/shows it; simulated link loss hides it. This is synthetic display verification, not real position. |
| Map imagery | Cesium still reported `tilesLoaded: false` after the test wait. Complete terrain/imagery rendering was not verified. |
| Offline regression checks | 20 Python tests and 4 JavaScript tests passed, including stale data, real-start arming separation, mission ACK acceptance/rejection, command errors, camera reconnect, and QGC restart prevention. |

## Remaining autonomy limitations found in code

- Most commands report transport delivery, without waiting for `COMMAND_ACK`.
  Live flight status must confirm their result.
- The FC adapter is configured for PX4 system/component `1/1`, which matches
  the vehicle on the USB link (`MAV_SYS_ID=1`, `MAV_COMP_ID=1`).
- The browser can supply the mission return-home coordinates. The server does
  not independently require a current aircraft home position for that upload.
- When an explicit return altitude/home is supplied, `route_to_mission` adds a
  direct home waypoint and land item; it does not route that return leg through
  the exclusion-aware planner or add a separate vertical climb item.
- Draft mission editors are not bidirectionally synchronized. The web uploader
  waits for a new aircraft mission ACK; QGC must download the vehicle mission
  after an upload. No physical upload was attempted.
- Actual GPS fix, arming checks, failsafe settings, RC override, RTL behavior,
  mission execution, and payload hardware were not verified.

## Next physical checks

The flight controller's only MAVLink serial port is TELEM1 at 57600 baud,
Normal mode (`MAV_0_CONFIG=101`, `SER_TEL1_BAUD=57600`, `MAV_0_MODE=0`).
Firmware read over USB is PX4 1.18.0-beta, airframe `SYS_AUTOSTART=4001`,
system id 1. The direct Type-C link works. Wireless MAVLink still does not:
the HM30 ground unit and camera answer on Ethernet, and nothing replies on
UDP port 19856. On the HM30 screen, datalink output has to be UDP aimed at
this laptop, and the air-unit UART has to be the wire on TELEM1. GPS needs a
sky view before the map or 3D model has a real position.

Browser testing used a separate session with all non-GET API requests blocked.
Synthetic GPS was injected only into that test browser. It was not sent to QGC,
the relay, or the aircraft. Browser evidence is saved in
`/tmp/albatross-browser-report.json` and `/tmp/albatross-verified.png`.
