# tracking_ws: drone person tracking (ROS 2 + Gazebo + PX4)

A PX4 quadrotor with a downward-tilted camera flies over a simulated city full of
walking people and moving cars. A YOLO detector and a multi-object tracker run on
the camera feed. You click any person and the drone locks on and follows them: it
holds a stand-off distance and altitude, keeps them centred, swings round behind
them, and re-acquires them after occlusions.

Stack: ROS 2 Humble · Gazebo Harmonic (gz-sim 8) · PX4 SITL (offboard via MAVROS) · Ultralytics YOLO11.

```
tracking_ws/
├── scripts/
│   ├── setup_assets.sh        download models / people / YOLO weights, generate the world
│   ├── build.sh               colcon build --symlink-install
│   ├── run.sh                 start everything
│   ├── stop.sh                kill leftovers (gz, px4, mavros, nodes)
│   └── test.sh                unit tests
├── assets/                    (git-ignored, created by setup_assets.sh)
│   ├── models/                buildings, cars, props, recoloured animated people
│   └── weights/               yolo11n.pt, yolo11s.pt
└── src/
    ├── tracker_interfaces/    msgs: Track, TrackArray, TargetState, FollowStatus · srvs: LockTarget, Command
    ├── tracker_sim/           simulation
    │   ├── config/city.yaml       city layout: roads, buildings, traffic routes, people (single source of truth)
    │   ├── tools/build_world.py   city.yaml -> worlds/city.sdf (+ people meshes in assorted clothing colours)
    │   ├── tools/fix_classic_materials.py   Gazebo-classic OGRE materials -> PBR (else they render black)
    │   ├── models/x500_tracker/   PX4 x500 + camera pitched 40 deg down (640x480, 69 deg HFOV)
    │   ├── worlds/city.sdf        generated world
    │   ├── launch/sim.launch.py   gz server/GUI + PX4 SITL + bridge + traffic
    │   └── tracker_sim/           gz_camera_bridge, traffic_node, paths (route geometry)
    ├── tracker_perception/    detect -> track -> lock
    │   └── tracker_perception/    detector, kalman_box, byte_tracker, camera_motion, appearance, target_lock, perception_node
    ├── tracker_control/       geolocate -> estimate -> guide -> fly
    │   └── tracker_control/       geometry, target_estimator, follow_controller, follow_node
    ├── tracker_ui/            operator console (click to lock, keys, HUD, mini-map)
    └── tracker_bringup/       tracking.launch.py + config/tracking.yaml (all tunables)
```

## Quick start

Requirements: ROS 2 Humble with `mavros`, Gazebo Harmonic with the python bindings
(`python3-gz-transport13`, `python3-gz-msgs10`), a built PX4-Autopilot
(`make px4_sitl`, default location `~/PX4-Autopilot`, override with `PX4_DIR`), and
`pip install ultralytics`.

```bash
cd tracking_ws
scripts/setup_assets.sh      # once: ~150 MB of models + weights, generates the world
scripts/build.sh
scripts/run.sh               # Gazebo + PX4 + MAVROS + perception + controller + console
```

In the **Drone Tracker** window:

| input | action |
|---|---|
| `t` | take off (arms, OFFBOARD, climbs to the follow altitude) |
| `w a s d` / `q e` | while hovering: move 5 m forward/left/back/right, yaw 30 deg |
| **left click** on a person | lock: the drone starts following |
| right click | release the lock (the drone hovers) |
| `h` | release the lock and hover in place |
| `m` | follow mode: `behind` → `hold_bearing` → `orbit` |
| `+` `-` / `0` | follow distance ±2 m / back to automatic |
| `r` `f` | altitude ±2 m |
| `l` | land |
| `Esc` | close the console (the drone keeps its current behaviour) |

The same commands are available without the GUI:

```bash
ros2 service call /follow/command tracker_interfaces/srv/Command "{command: takeoff}"
ros2 service call /tracking/lock tracker_interfaces/srv/LockTarget "{track_id: 12}"
ros2 run rqt_image_view rqt_image_view /tracking/image_annotated
```

Options: `scripts/run.sh gui:=false console:=false weights:=/path/to/model.pt`.
PX4 runs as **instance 1** (MAVLink 14541/14581, system id 2), so its parameters
and ports never clash with a normal `make px4_sitl` (instance 0). Stop any other
Gazebo first, because two worlds on one gz-transport network interfere.

## Architecture

```
 Gazebo (city.sdf)                                     PX4 SITL (instance 1)
   ├─ camera ──gz-transport──► gz_camera_bridge ──► /camera/image_raw           ▲ velocity + yaw-rate setpoints
   ├─ cars ◄──set_pose_vector── traffic_node                                  │ (/mavros/setpoint_raw/local)
   └─ actors (scripted walks)                                                 │
                                                                              MAVROS ◄── /mavros/state, local_position
 /camera/image_raw ─► perception_node                                         ▲
                        YOLO ─► camera-motion ─► ByteTrack ─► target lock     │
                        └─► /tracking/tracks, /tracking/target ─► follow_node ┘
                            /tracking/image_annotated ─► operator_console ─► /tracking/lock, /follow/command
```

The stock Humble `ros_gz_bridge` is built for Gazebo Fortress and can't talk to
Harmonic, so `gz_camera_bridge` reads the camera through the gz-transport python
bindings instead.

## Algorithms

### Perception (`tracker_perception`)
1. **Detection**: YOLO11 (COCO) restricted to person/car/truck/bus/bike classes, with the
   confidence floor at 0.1 on purpose (see stage 2 below).
2. **Camera-motion compensation** (`camera_motion.py`, as in BoT-SORT): the camera yaws
   and translates with the drone, so every box shifts between frames. Corners are
   tracked with pyramidal Lucas-Kanade optical flow (people's boxes masked out) and a
   similarity transform is fitted with RANSAC. The tracker warps all Kalman predictions
   with it before association.
3. **Multi-object tracking** (`byte_tracker.py`, ByteTrack-style). Each track has a
   constant-velocity Kalman filter on (cx, cy, aspect, height), with noise scaled by box height.
   * Stage 1: high-confidence detections ↔ all confirmed tracks, Hungarian matching on
     `1 − BIoU·score`, never across classes. BIoU is the IoU of slightly enlarged boxes
     (C-BIoU), which still overlap for small, fast-moving boxes.
   * Stage 2: low-confidence detections ↔ tracks that were visible last frame. A
     partly occluded person scores low, and using those detections keeps their identity.
   * New tracks need `min_hits` consecutive matches to be confirmed. Unmatched tracks
     coast for `max_lost_frames`, with no size change while unobserved.
4. **Target lock + re-identification** (`target_lock.py`, `appearance.py`). While the
   locked person is visible, a hue-saturation histogram of torso and legs is learned
   (only from frames where the person doesn't overlap anyone else). If the track is
   lost, new tracks become candidates only if they are the same class, were never seen
   at the same time as the target, and pass a size gate, a spatial gate that widens
   with time lost, and an appearance gate (Bhattacharyya distance). The best candidate
   must win 3 frames in a row before the lock moves to it. The lock is released after
   `reid_max_lost_time` seconds.

### Control (`tracker_control`)
1. **Geolocation** (`geometry.py`): the bottom-centre pixel of the box (the feet)
   becomes an optical ray, `d_w = R_wb · R_bc(pitch) · R_co · K⁻¹ [u v 1]`, which is
   intersected with the ground plane. The drone pose comes from the pose buffer at the
   frame's timestamp, which compensates for detection latency.
2. **Target estimation** (`target_estimator.py`): a constant-velocity Kalman filter on
   `[x y vx vy]` with white-acceleration process noise. Measurement noise grows with
   slant range, and a chi-square gate (99 %) rejects outliers. This gives the
   person's world position and velocity even through short dropouts.
3. **Guidance** (`follow_controller.py`):
   * stand-off point `p_des = p_target + D·[cos b, sin b]` at altitude `H`. By default
     `D = (H − h_centre)/tan(camera_pitch)`, which keeps the person vertically centred
     with a fixed camera (10 m altitude → 10.8 m stand-off).
   * bearing `b`: in `behind` mode it swings round behind a walking target
     (rate-limited to 15 deg/s) and holds when the target stops. `hold_bearing` keeps
     the bearing fixed and `orbit` circles the target.
   * velocity command `v = v_target + D·ḃ·t̂ + Kp·e + Ki∫e`, saturated in speed and
     rate-limited in acceleration. The target-velocity feed-forward removes the lag
     when following a moving person.
   * yaw rate from the target's pixel offset (lowest latency) plus a line-of-sight-rate
     feed-forward. If no fresh detection exists, the estimated geometry is used instead.
   * safety: minimum altitude and a geofence radius around home.
4. **Flight state machine** (`follow_node.py`):
   `DISCONNECTED → LANDED → TAKEOFF → HOVER ⇄ FOLLOW ⇄ SEARCH`, plus `LANDING` and `PILOT`.
   After losing sight, the drone keeps following the prediction for `coast_time`
   (3 s), then hovers and turns toward where the target was heading (SEARCH). If the
   pilot switches PX4 out of OFFBOARD, the node stops commanding (PILOT).

### Simulation (`tracker_sim`)
* **City**: a 2×2 grid of blocks with roads, sidewalks, lane markings and zebra crossings:
  a residential block (houses, apartments), a commercial block (gas station, fast food,
  grocery, parking lot), a civic block (police, fire station, school, offices) and a park
  with a plaza and fountain. Everything is generated from `config/city.yaml`. Models come
  from `osrf/gazebo_models`; their classic materials are converted to PBR.
* **People**: 22 animated actors: sidewalk and park walkers, joggers, random
  wanderers and chatting groups. Each has its own shirt, trousers and skin colour
  (the mesh is recoloured), which looks realistic and gives re-ID something to work with.
* **Traffic**: 7 cars drive closed loops with rounded corners, car-following gaps,
  junction yielding and acceleration limits. Another 13 cars are parked.

## Tuning and GPU notes

All parameters live in `src/tracker_bringup/config/tracking.yaml`.

* **GPU (e.g. RTX 5060)**: `device: auto` uses CUDA when PyTorch sees it. RTX 50-series
  cards need a PyTorch build with CUDA 12.8 or newer (`cu128`). With a GPU, use
  `imgsz: 960` (better at spotting distant people) or a larger model (`yolo11m.pt`).
  Gazebo also renders much faster on a discrete GPU.
* **CPU only**: the pipeline runs at about 5–6 Hz with `yolo11s` on a laptop CPU (tested).
  Tracking still works, but expect more brief losses during fast turns. `yolo11n` is 2×
  faster but misses small people seen from above.
* The camera mount in `models/x500_tracker/model.sdf` must match `camera_pitch_deg`,
  `camera_hfov` and `camera_offset`.
* Changing the city: edit `config/city.yaml`, then rerun `scripts/setup_assets.sh`, which
  regenerates the world.

## Tested behaviour (CPU laptop, Intel iGPU)

* Takeoff → hover → lock → follow → land, all through the services and the console.
* Geolocation error on a static person: about 0.5–1 m from 10–25 m away.
* Steady stand-off held at 10.8 ± 0.1 m.
* A walker followed for 2 minutes around the plaza loop, with re-ID recovering the
  target after each loss.

## Known limitations

* Geolocation assumes flat ground at z = 0, which holds in this world.
* No obstacle avoidance. The follow altitude (10 m) clears trees and most buildings,
  but not the office block (19 m) or the apartments (15 m). Raise the altitude (`r`)
  near them.
* Wall-clock time is used throughout (no `use_sim_time`). If Gazebo runs slower than
  real time, target velocities are under-estimated and the PI term makes up the rest.
* Appearance re-ID uses colour histograms, so people dressed alike can be confused
  after a long occlusion. A learned re-ID embedding (e.g. OSNet) would slot into
  `appearance.py`.
