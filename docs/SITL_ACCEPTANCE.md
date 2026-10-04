# Operator workspace acceptance run

Run this on a host in a dedicated SITL session. Do not use a real aircraft for
fault injection or motor-stop testing. Record the commit/revision, browser,
viewport, PX4 version and QGC version with the results.

## Startup

1. Run `GCS_MODE=sitl bash launch/start.sh 1` from the repository root.
2. If preflight reports conflicting PIDs, review/end the previous session and
   retry. The check itself does not terminate anything.
3. Open localhost:8080. Verify SIMULATION, aircraft link, GPS and QGC vehicle
   identity. A launch message alone does not confirm service readiness.
4. Start the FPV and Gazebo camera panes. Verify their sources separately from
   aircraft telemetry. Confirm a disconnected camera is not interpreted as
   an aircraft-link failure.

## Readability and layout

Test 1440×900, 1280×800 and a narrow 390 px viewport, normal and Large text.

- Read mode, battery, altitude, GPS and command feedback without zooming.
- Switch Large text, reload and confirm the preference persists.
- Resize both dividers by pointer and keyboard. Maximize/restore every pane.
- Confirm ARM/GO, Hover, Return and emergency access are visible and do not overlap.
- Open both drawers; scroll to every control and event record; close with Escape.
- Test browser zoom at 200%, keyboard focus, and reduced-motion preference.
- Verify 2D, Aircraft 3D and GPS selection and workspace fullscreen.

## Mission and control

1. Choose destination, inspect the endpoint, altitude and Auto-RTL settings.
2. ARM: verify it does not start the mission and waits for aircraft state.
3. GO: verify upload/readback completes before the start request. Observe the
   same stored mission in QGC after Download from Vehicle.
4. Hover: verify PX4 enters hold and the mission is retained.
5. Resume: verify the current item resumes instead of restarting.
6. In a manual-capable SITL session, verify RC takeover and autonomous resumption.
7. Return: verify aircraft RTL behavior and home destination. Test Land here
   separately in simulation.
8. Open/cancel emergency stop: no command should be sent. In simulation only,
   explicitly confirm motor stop and verify the resulting disarmed state.
9. Inspect Details → Session events for requested, acknowledged and confirmed
   actions. This log is per browser session, limited to 50 entries, not a durable
   flight recorder.

## Failure and concurrency

- Change a waypoint/return setting while upload is pending: GO must not continue.
- Request Hover during upload: pending GO must be interrupted.
- Reject an action in PX4: GUI must display rejection rather than success.
- Disconnect telemetry: controls disable, last-seen age appears, verification
  is revoked. Reconnect and require a new mission verification.
- Restart PX4 or the bridge: no old mission token should authorize a start.
- Change mode from QGC: the confirmed-mode display follows telemetry.
- Upload a different mission from QGC: read it in the amber aircraft preview;
  the blue draft stays unchanged. Verify old start/resume authorization fails.
- Repeat with two browser tabs. Backend transfer locking must prevent overlapping
  web uploads. QGC has independent authority; use one mission writer at a time.
- Interrupt camera streaming and restore it. Confirm video recovery independently.

## Endurance

Run for at least two hours, switching views and resizing periodically. Record
browser memory, server memory, telemetry responsiveness and error counts at the
start, hourly and end. Check that the event log stays bounded and that buttons
recover from completed/rejected requests. Save failures with timestamps and logs.

## Current verification limits

Offline unit/integration checks pass; this acceptance run has not been executed
in the restricted development sandbox. Startup preflight is a conflict detector,
not a full process supervisor: port ownership, PID-scoped stop/restart, durable
command audit logging, bundled offline dependencies and hardware failsafe checks
remain separate hardening work. Existing launch scripts still contain other broad
cleanup commands; no claim of complete lifecycle isolation is made.
