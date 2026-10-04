# GUI wiring verification — 2026-10-04

The GUI and backend wiring fixes pass offline regression checks. A running,
rendered prototype was not verified in this session: the execution sandbox
rejected local server sockets and Chrome startup. No aircraft commands were sent.

Implemented fixes:

- Waypoint altitude editing now uses the same configured ceiling as the main
  altitude input and writes the clamped value back into the editor.
- Boundary edits/reset invalidate the route and require a successful save before
  planning. Reset displays the default KML boundary rather than an empty map
  while the backend silently uses that boundary.
- Replanning clears the old uploadable route. Responses arriving after a waypoint
  or boundary edit are discarded. Planning/upload buttons disable while pending.
- Route changes and clears synchronize the Cesium route entity with the 2D map.
  Missing Cesium restores 2D with an error message. Failed initialization can retry.
- Arm/Disarm text follows received telemetry. Unknown mode no longer defaults to
  SITL for arming. Missing attitude/heading data no longer looks fully live.
- Simulator start/stop/follow endpoints reject requests in real-aircraft mode.
  Failed simulator-stop requests no longer display success.
- Mission uploads validate coordinates, altitude, hold time and route segments
  against the current saved boundary. Malformed requests return JSON errors.
- Real-aircraft uploads use reported HOME_POSITION and airborne state rather
  than trusting the browser's home and airborne fields. Explicit return-home
  missions require aircraft home. The return leg first reaches the requested
  altitude at the last outbound point, then uses the exclusion-aware planner.
- Reposition requires a live position so the direct path can be checked.

Run all offline checks from the repository root:

```bash
python3 -m unittest discover -s gcs/tests -v
node gcs/tests/test_ui_commands.cjs
node gcs/tests/test_gui_wiring.cjs
```

Results: 31 Python tests and 14 JavaScript tests passed. The new JavaScript suite
executes the complete app against a minimal DOM/map and mocked API, including
all button bindings, plan/upload, command payloads, failed requests and stale
planner responses. It does not validate layout, WebGL or real network behavior.
Python tests exercise the actual HTTP handlers and mission builder with a mocked
flight controller; they do not open sockets or send MAVLink.

To verify the prototype on the host, use the existing planning simulator:

```bash
bash launch/start.sh 1
```

Open http://localhost:8080. Add waypoints, change a waypoint height, plan, upload,
then exercise Arm/Start against SITL. Edit/save the boundary and confirm a replan
is required. Toggle 3D, clear/replan a route, and check both camera feeds. This
launches the project's simulator stack and restarts its MAVLink bridge; use a
simulation session separate from real-aircraft operation.

Remaining verification: browser rendering and CDN dependencies, Gazebo startup,
video streaming, mission execution/ACKs, and physical aircraft behavior. Most
flight-command success responses still mean transport delivery; received
telemetry must confirm the action. The planner remains a sampled/grid prototype,
not a formally verified geometric or flight-safety system. Return segments are
validated at upload, but the preview currently shows the outbound route only.
