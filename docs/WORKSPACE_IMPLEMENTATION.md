# Docked workspace implementation

The main map and two camera panes share resize dividers. Split proportions persist
in browser storage. Panes expand/restore and camera panes collapse. Flight controls
remain outside the pane layout, including workspace fullscreen. Compact layouts
stack the map above the two cameras. Mission and Details open drawers.

The main view retains 2D, Cesium aircraft 3D, and GPS tabs. Gazebo is a streamed
simulation chase-camera view, not an embedded interactive Gazebo client. In real
mode the Gazebo pane explains its unavailable source. The FPV pane uses the single
real camera stream; in SITL it uses the second simulated camera in FPV orientation.

## Flight workflow

Choose destination creates a local draft and requests a route. Review Mission
settings before acting. ARM requires its own press; telemetry must confirm arming.
GO uploads and reads back the mission if necessary, then reads it back again
before requesting start. Resume rechecks the stored mission without resetting the
current item. The normal Disarm action remains available in Mission.

Hover requests PX4 hold while preserving the mission. This prototype conservatively
requires global position/relative altitude and checks absolute-position estimator
flags when present. It does not yet support local-position-only hover. RC manual
requires received RC channels in real mode; unknown RSSI is identified in the header.
Return requests aircraft RTL settings, while Auto-RTL in Mission builds an explicit
planned return and landing sequence. Land here is a separate confirmed action.

Emergency motor stop uses MAV_CMD_COMPONENT_ARM_DISARM with disarm and force=21196,
not flight termination. Its dialog requires explicit acknowledgement. It bypasses
the normal command lock, but still requires communication and aircraft response.
An airborne vehicle can fall. The supported command paths were checked against the
local PX4 Commander and Navigator source; no physical activation was tested.

## QGC integration

The web adapter uses GCS identity 246/190, with aircraft target still fixed at 1/1.
Mission requests and acknowledgements are filtered by recipient and mission type.
Premature accepted ACKs cannot complete uploads. Web mission operations are locked
against overlapping web transfers; this does not lock QGC. Upload success includes
readback comparison of command, frame, coordinates, altitude and parameters.

Supported mission-ID changes invalidate the verification token. Readback before
start/resume remains mandatory. Read aircraft mission downloads a separate amber,
read-only navigation preview, preserving the local blue draft. Non-navigation
commands need inspection in QGC. QGC must still download after a web upload;
draft editors are not automatically synchronized. QGC is never restarted by sync.

Flight commands wait for addressed, matching COMMAND_ACK; the floating controls
also wait for requested telemetry state. Ambiguous timeouts are reported rather
than automatically resending flight actions. Hover/RC/return/emergency actions
interrupt a pending web mission operation. Independent simultaneous writes or mode
changes from QGC remain possible: operate one mission writer at a time.

## Verification

Run from the repository root:

```bash
python3 -m unittest discover -s gcs/tests -v
node gcs/tests/test_ui_commands.cjs
node gcs/tests/test_workspace.cjs
```

55 Python tests and 27 JavaScript tests passed. The workspace suite includes the
10 earlier GUI tests plus 13 workspace tests. Tests mock aircraft transport and
DOM/map objects; one Python test runs the complete upload/readback/start protocol
against a simulated message responder. JavaScript checks cover stale responses,
GO interruption, return-setting changes during upload, guarded motor stop, resize
limits, and read-only aircraft mission inspection.

Browser rendering, actual CDN/WebGL behavior, video streaming, and live PX4/QGC
operation remain unverified. The current execution sandbox rejects local listening
sockets and Chrome startup. No aircraft command was sent during development.

Host acceptance checks in a separate SITL session:

1. Launch `GCS_MODE=sitl bash launch/start.sh 1` and open localhost:8080.
2. Check map, FPV and Gazebo panes at desktop and narrow widths. Resize with mouse
   and keyboard, expand/restore, collapse, reload for saved proportions, fullscreen.
3. Exercise 2D/3D/GPS and drawer controls without losing the flight overlay.
4. Choose destination, inspect settings, ARM, GO; verify mode and mission in QGC.
5. Hover, resume, return, and test invalid position/connection conditions.
6. Upload from QGC, read the aircraft mission in the web drawer, and verify the
   old draft cannot start without being re-uploaded/reviewed.
7. Test emergency motor-stop behavior only in SITL before a separate controlled
   hardware validation. Verify command rejection and link loss remain explicit.

The existing launch scripts restart shared services. Do not use a simulation
launch to validate this while conducting real-aircraft operations.

## Readability and stability pass

Larger labels (16–18 px), telemetry (22 px), primary controls (88–104 px on
desktop), and 28 px SVG flight icons improve laptop readability. Large text adds
15% scaling and persists locally; narrow layouts reflow. Details includes a
50-entry session event log. Lost-link age is displayed and backend verification
is revoked on heartbeat loss. Startup preflight detects conflicting real/SITL
relay processes without stopping them; simulation mode is explicitly set by
its launchers and the broad Ruby-process kill has been removed.

See [SITL acceptance run](SITL_ACCEPTANCE.md) for visual, operational, failure and
endurance checks. This is an incremental hardening pass, not a completed process
supervisor or a certification of live stability.


### Compact cockpit refinement
- The header uses compact icon-led telemetry cards; GPS count and fix come from aircraft telemetry and clear on disconnect.
- Mission and Details remain visible. Settings groups text size, layout reset, and workspace fullscreen.
- Flight actions share one floating panel. The separate aircraft motor-stop panel retains its confirmation dialog.
- Camera panes display stopped, connecting, or unavailable states instead of broken-image labels. Use Stop then Start to retry a failed feed.
- MJPEG images remain renderable while the streaming HTTP response is open; a load event is not required before showing video.
- Window dividers still resize adjacent panes. Narrow screens reflow header instruments and controls.

Validation: mocked browser wiring tests cover stream state transitions and GPS changes; live browser layout and SITL video acceptance remain required.


### Cockpit palette
The final theme section in `gcs/web/workspace.css` defines the shared charcoal/slate palette. Cyan marks selected controls and the primary ARM/GO action; green marks confirmed readiness or active feeds, amber marks incomplete readiness, and red marks motor-stop controls. Disabled actions retain readable labels with subdued backgrounds. Map panels use 92% opaque charcoal with blur; map imagery and overlays retain their geographic colors.

Visual contrast refinement: disabled actions retain brighter labels and outlines without the cyan active fill. Camera placeholder tiles are 64px with a 34px drawable area, explicitly accounting for border-box sizing.

### Sensor diagnostics and navigation checks
Details → Sensor health shows aircraft-reported presence/enabled/health, GPS fix, range, battery current, estimator mode, and aircraft state. Ages over five seconds are marked stale; missing reports remain unknown. Zero battery current is explicitly unverified. The range is sensor distance, not a replacement for relative flight altitude or horizontal positioning.
Backend start/resume now requires live navigation both before mission reset and before entering mission mode. Return requires live position and valid home. Constant-position estimator mode cannot enable position hold. Return waypoints preserve the requested outbound destination hold.
These backend changes require a controlled GCS server restart; the active aircraft connection was not restarted during implementation.
