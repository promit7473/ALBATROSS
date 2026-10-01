# QGroundControl Setup

## Current HM30 hardware connection

For `launch/start_real.sh`, one `gcs/mav_hub.py` process owns the aircraft link
and forwards the same MAVLink traffic to QGC on `127.0.0.1:14550` and the web
bridge on `127.0.0.1:14555`. Replies from both applications return through that
relay. QGC's UDP AutoConnect should be enabled. Do not also open the same serial
device in QGC when the relay owns it.

The HM30 network video connection and flight-controller telemetry are separate.
Receiving camera video does not prove MAVLink is arriving. The HM30 datalink
output mode, flight-controller UART wiring, baud rate, and MAVLink configuration
must match the selected telemetry transport. For the default Ethernet relay,
the expected endpoint is `192.168.144.12:19856` using UDP.

Live position and flight mode update in both applications from the aircraft.
Draft mission editors do not automatically mirror each other. After a successful
web mission upload, use QGC's **Download from Vehicle** to retrieve the stored
mission. The web GCS no longer restarts QGC for mission synchronization in real
mode. Upload/flight verification is still required; sharing telemetry is not a
flight-readiness check.

The browser's **Aircraft 3D** is a Cesium view of MAVLink position and attitude.
Gazebo is the separate SITL simulator. Missing telemetry/GPS hides the browser's
aircraft model rather than placing it at the map center.

The following sections describe the simulator and alternate MAVROS USB setup.

QGC runs **alongside** the web GCS, not instead of it, and needs **zero manual
configuration**. The trick: PX4's default GCS link already sends telemetry to
`127.0.0.1:14550`, which is exactly where QGC's factory AutoConnect listens. So
QGC gets that port, and the web GCS's `mavlink2rest` is moved to a dedicated
`:14555`. Neither ever fights for a port.

There are two scenarios: **SITL** (Gazebo) and **real Pixhawk**.

---

## 1. SITL (Gazebo / `launch_chattogram_clean.sh`)

### Data path
```
                          +-- 127.0.0.1:14550 --> QGroundControl  (factory AutoConnect)
PX4 SITL --(mavlink)------|
  default link -> :14550  +-- 127.0.0.1:14555 --> mavlink2rest --> web GCS
  pushed link  -> :14555                          (REST :8088)     (:8080)
```
- PX4's **default** GCS link targets `127.0.0.1:14550` → QGC picks it up.
- The launcher adds a **second** PX4 mavlink instance that pushes to
  `127.0.0.1:14555`:
  ```
  mavlink start -x -u 14556 -o 14555 -r 4000000 -f
  ```
  `mavlink2rest` listens there (`udpin:0.0.0.0:14555`) and serves the web GCS.
  `-x` enables MAVLink FTP so parameter/mission download works.

### QGC setup
None. QGC's default **AutoConnect → UDP** listens on `:14550` out of the box and
connects automatically. Do not add a manual comm link, and leave AutoConnect on.

### Launch
```bash
# bring up SITL + web GCS, then QGC:
LAUNCH_QGC=1 launch/start_all.sh

# or launch QGC by itself against an already-running SITL:
launch/qgc.sh
```
Give SITL ~15 s to boot; QGC connects as soon as PX4 is up.

---

## 2. Real Pixhawk (`ros2_ws/run_sim_imu.sh`)

### Data path
```
Pixhawk --USB(/dev/ttyACM0)--> MAVROS --(gcs_url udp :14550)--> QGroundControl
```
**MAVROS opens the USB serial port exclusively** — QGC cannot also open it over
USB. MAVROS forwards MAVLink to a GCS over UDP via its `gcs_url`
(`udp://@localhost:14550`), and QGC's default AutoConnect on `:14550` picks it
up. No manual QGC setup here either.

### Launch
```bash
ros2_ws/run_sim_imu.sh /dev/ttyACM0:921600
```
QGC starts automatically at the end of that script.

---

## Port reference

| Port  | Used by                                    | Scenario     |
|-------|--------------------------------------------|--------------|
| 14550 | QGroundControl (PX4 default link / MAVROS) | both         |
| 14555 | mavlink2rest input (web GCS)               | SITL         |
| 14556 | PX4 local socket for the web-GCS push link | SITL         |
| 8088  | mavlink2rest REST API                      | SITL         |
| 8080  | web GCS server                             | both         |

## Troubleshooting
- **QGC shows "Waiting for vehicle connection":** confirm SITL finished booting
  (`tail ~/sitl_clean.log`) and that AutoConnect UDP is enabled in QGC
  (Application Settings → AutoConnect).
- **Web GCS blank but QGC works (or vice-versa):** the links are independent, so
  check the one that's down — `mavlink2rest` on `:14555`/`:8088` for the web GCS,
  the PX4 default link for QGC.
- **Parameters won't load:** the link must reach a `-x` (FTP-enabled) mavlink
  instance. Both the default link and the `:14555` push use `-x`.
