# Maritime Autonomy — Chattogram Port UAV

<div align="justify">

A full **software-in-the-loop autonomous maritime-patrol simulator**: PX4 SITL +
Gazebo (real Chattogram satellite/terrain world) + ROS 2 autonomy + a browser
Ground Control Station with live map, 3D view, a **no-fly-aware multi-waypoint
mission planner**, and a real-time **chase-camera feed** rendered from the sim.

</div>

<div align="center">

![Autonomous maritime patrol — chase cam over Chattogram port](media/demo.gif)

*Autonomous mission flown in PX4 SITL, rendered in Gazebo over the real Chattogram port terrain, captured from the in-sim chase camera. ([full video](media/demo.mp4))*

</div>

---

## Architecture

```mermaid
flowchart LR
  UI["Browser UI<br/>Leaflet 2D + Cesium 3D<br/>mission editor + feed"] -->|HTTP :8080| GCS["server.py<br/>GCS backend<br/>planner A* / no-fly"]
  GCS -->|REST :8088| M2R[mavlink2rest]
  M2R -->|MAVLink UDP :14550| PX4["PX4 SITL (+ MAVROS)"]
  PX4 <-->|sim| GZ["Gazebo<br/>Chattogram world"]
  GZ -->|gz-transport| CAM["chase_cam.py<br/>cam_bridge.py"]
  CAM -->|MJPEG :8091 proxied| GCS
  KML[("KML operating area<br/>+ exclusions")] --> GCS
```

User manual (launch, GUI operation and troubleshooting):
[`docs/LAUNCH_GUIDE.pdf`](docs/LAUNCH_GUIDE.pdf)
([editable source](docs/LAUNCH_GUIDE.tex)).
Architecture: [`docs/SYSTEM_OVERVIEW.pdf`](docs/SYSTEM_OVERVIEW.pdf).

## Features

- **Exclusion-aware path planning** — A* over the saved operating boundary and
  KML exclusions. Blocked waypoints are moved to a safe edge; review the planned
  endpoint before upload. Uploaded segments are checked against current geometry.
  This is a sampled/grid prototype, not a flight-safety guarantee.
- **Multi-waypoint mission editor** — drop an ordered chain of waypoints, each with
  its **own altitude**; adjustable auto-RTL return altitude.
- **Live web GCS** — 2D map (Leaflet) + 3D view (Cesium), telemetry HUD, arm /
  start / RTL / reposition / cargo-drop, all over MAVLink.
- **Manual ⇄ Auto toggle** — one button hands control to the RC pilot (POSCTL) or
  resumes the uploaded mission (AUTO.MISSION). On a real Pixhawk the RC always
  keeps override priority; the toggle just commands the PX4 flight mode.
- **Chase-camera feed** — a real Gazebo camera that follows the drone (look-at,
  travel-facing), streamed as MJPEG. Works headless / under Wayland (no screen
  scraping).
- **Draggable, resizable** feed and 3D windows.
- **Smooth takeoff & landing** — jerk/accel-limited PX4 tuning applied on launch.

## Repository layout

```
maritime_project/
├── gcs/         web GCS: server.py, fc_iface.py, rc_to_sitl.py, chase_cam.py, cam_bridge.py, web/
├── ros2_ws/     ROS 2 workspace — pkg maritime_autonomy (planner, geo, mission nodes, world)
├── launch/      start.sh (3-stage selector), start_all.sh, start_sitl_manual.sh, start_real.sh
├── assets/      chattogram_zones.kml (operating area + exclusion zones)
├── docs/        SYSTEM_OVERVIEW + LAUNCH_GUIDE (.tex/.pdf)
├── media/       demo.mp4 / demo.gif
└── external/    symlinks to PX4-Autopilot and mavlink2rest (not committed)
```

Paths are self-locating: `server.py` derives the project root from its own location
(override with `MARITIME_ROOT`), and the shell scripts derive it from their path.

## Prerequisites

- **ROS 2 Humble**, **Gazebo Harmonic** (`gz sim`)
- **[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)** built for SITL, with the
  `chattogram` gz world installed
- **[mavlink2rest](https://github.com/mavlink/mavlink2rest)** binary
- Python: `numpy`, `opencv-python`, and the Gazebo Python bindings
  (`gz.transport13`, `gz.msgs10`)

Link the two external dependencies into the project:

```bash
mkdir -p external
ln -s /path/to/PX4-Autopilot external/PX4-Autopilot
ln -s /path/to/mavlink2rest  external/mavlink2rest
```

## Three stages

The system runs in three stages, all launched through one selector:

```bash
bash launch/start.sh 1     # Stage 1: planning sim (autonomous, no RC)
bash launch/start.sh 2     # Stage 2: sim + manual RC control (fly the sim)
bash launch/start.sh 3     # Stage 3: real Pixhawk (no sim)
# also: planning | manual | real
```

### Stage 1 — Planning simulation (no Pixhawk, no RC)

The "earlier version": PX4 SITL + Gazebo + web GCS. `COM_RC_IN_MODE=4` (manual
control disabled). Pure autonomous mission planning.

```bash
bash launch/start.sh 1      # = launch/start_all.sh
```

Open **http://localhost:8080**, allow roughly 15 s for warmup, and click **start**
in the **CHASE CAM** or **TOP-DOWN CAM** pane. Check LINK and received position.

> **➕ Add waypoints** → **Plan route** (routes around no-fly zones) →
> **Upload mission** → **Arm** → **Start**.

After drawing/editing the boundary, click **Save** before planning. Boundary
**Reset** restores the default KML polygon and also requires Save. Waypoint or
boundary edits clear the preview; replan before uploading. The default whole-metre
altitude limit is 60 m. Auto-RTL adds a planned return and landing at upload; the
map preview currently shows only the outbound route.

### Stage 2 — Manual control in the sim (RC receiver wireless)

Same Gazebo sim, but the pilot flies the *simulated* drone with the real RC
transmitter. `COM_RC_IN_MODE=1` (MAVLink manual control enabled) +
`rc_to_sitl.py` bridges RC from the real Pixhawk to SITL:

```
RC transmitter --wireless--> receiver (on Pixhawk) --USB/MAVLink--> rc_to_sitl.py
      --HTTP/MANUAL_CONTROL--> mavlink2rest --MAVLink--> PX4 SITL (Gazebo)
```

```bash
bash launch/start.sh 2      # = launch/start_sitl_manual.sh
```

The receiver stays wired to the Pixhawk (CRSF on its RC port); the Pixhawk is
connected to the laptop over USB (`/dev/ttyACM0`, USB power only — it need not
be armed). The bridge reads `RC_CHANNELS` and forwards `MANUAL_CONTROL` to
SITL via mavlink2rest. QGC connects to the *SITL* on :14550, not the Pixhawk.

In the web UI: toggle to **Manual** (POSCTL) to fly with sticks, or **Auto**
to resume an uploaded mission. Arm via the web GCS **Arm** button, then fly.
Watch `~/rc_bridge.log` for `RC link LIVE` (or the all-zeros warning if the
receiver is unbound/unpowered).

### Stage 3 — Real Pixhawk (no sim)

The drone is the real Pixhawk 6C in the sky. RC → receiver → Pixhawk is
handled natively by PX4 firmware (do **not** use `rc_to_sitl.py` here). The
web GCS talks MAVLink over the SIYI HM30 radio (UDP `192.168.144.12:19856`)
or a USB cable if the Pixhawk is plugged in, and offers the Manual ⇄ Auto
toggle. QGC pops up on `:14550`. FPV from the HM30 camera is proxied into
the web UI (`rtsp://192.168.144.25:8554/main.264`).

```bash
bash launch/start.sh 3      # = launch/start_real.sh
# laptop Ethernet on the SIYI LAN (192.168.144.30); QGC + FPV + web GCS
```

Starts the web GCS (:8080) + `mavlink2rest` + `mav_hub.py` (SIYI UDP or
Pixhawk USB → `:14550` / `:14555`) + `rtsp_bridge.py` (FPV MJPEG on `:8091`)
+ QGroundControl. No Gazebo, no SITL-only bypass params — set RC/datalink
failsafe (`NAV_RCL_ACT`, `NAV_DLL_ACT`, `COM_RCL_EXCEPT`) in QGC.

Real missions require live aircraft GPS for planning and aircraft-reported home
for explicit Auto-RTL uploads. **Start** requires separate arming. A flight-command
“request sent” message means delivery; confirm its effect in live telemetry.
Simulator start/stop controls are unavailable in real mode.

For SITL, restart the web server and chase camera after Python changes (leaves
PX4/Gazebo running):

```bash
GCS_MODE=sitl bash launch/restart_gcs.sh
```

Reload the browser for GUI-only changes. The restart helper starts a Gazebo chase
camera; for a grounded real-hardware session, rerun Stage 3 with the same settings
instead. Stage 3 also restarts the telemetry relay, video bridge and QGroundControl.

## Compact operator workspace

The map, FPV and Gazebo camera are docked panes with shared resize dividers,
maximize/collapse controls and saved split proportions. The main view retains
**2D map / Aircraft 3D / GPS** tabs. **Mission** and **Details** open drawers.

Use **Choose destination** to create and plan a quick draft. The floating round
**ARM** button requests arming; after telemetry confirms it, a separate **GO**
press uploads, reads back and starts the mission. **HOVER**, **RETURN**, RC manual,
Resume auto and Land here remain accessible over the views. The guarded red
**STOP** control requests emergency forced disarm, which can cause an airborne
vehicle to fall; it is distinct from hover and landing.

Web mission transfers are serialized and verified against aircraft readback.
**Read aircraft mission / QGC changes** displays a separate amber aircraft route.
QGC is never restarted for synchronization. Draft editors remain independent;
use one mission writer at a time. New flight actions check command acknowledgements
and wait for telemetry confirmation. Gazebo is a streamed 3D camera view, not an
interactive embedded Gazebo client. Local-position-only hover is not implemented.

Offline checks (no vehicle connection):

```bash
python3 -m unittest discover -s gcs/tests -v
node gcs/tests/test_ui_commands.cjs
node gcs/tests/test_workspace.cjs
```

The workspace suite also runs the existing GUI wiring tests. Browser rendering,
video, and physical flight behavior need host/SITL verification.

## Ports

| Port  | Service |
|-------|---------|
| 8080  | GCS web UI + API + `/video.mjpeg` proxy |
| 8091  | chase-cam / FPV MJPEG bridge |
| 8088  | mavlink2rest REST API |
| 14550 | PX4 → QGroundControl (MAVLink UDP) |
| 14555 | PX4 → mavlink2rest (web GCS MAVLink UDP) |

## Notes

- The session here is **Wayland**, so the feed is a rendered Gazebo camera, not a
  desktop screen grab (which returns black under XWayland).
- `launch_chattogram_clean.sh` auto-applies **SITL-only** arming bypass params
  (`CBRK_SUPPLY_CHK`, `NAV_RCL_ACT`, …) — do **not** use these on real hardware.
  `COM_RC_IN_MODE` is set by `SITL_RC_IN_MODE` env (4 = planning, 1 = manual);
  Stage 3 (`start_real.sh`) applies none of these.
- The ROS 2 `build/`/`install/` dirs are not committed; run `colcon build` in
  `ros2_ws/` before using the MAVROS mission nodes.

## Roadmap

- Migrate onboard control from MAVROS to **uXRCE-DDS** (offboard) for the Jetson
- Downward-facing (nadir) camera feed on the real-hardware path (FPV RTSP is live)
- Rangefinder-based AGL fusion for over-water flight
