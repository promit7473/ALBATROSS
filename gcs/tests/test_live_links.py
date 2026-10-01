"""Offline regression tests: never open the vehicle connection or send commands."""
import io
import json
import os
import sys
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import fc_iface as fc
import mav_hub
import rtsp_bridge as camera
import server


def record(message, age=0):
    timestamp = datetime.now(timezone.utc) - timedelta(seconds=age)
    return {"message": message, "status": {"time": {"last_update": timestamp.isoformat()}}}


def heartbeat(armed=False, age=0):
    return record({"base_mode": 128 if armed else 0, "custom_mode": 4 << 16}, age)


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.fc = fc.MavlinkRestFC()
        self.fc._streams_asked = True
        self.get = patch.object(fc, "_get").start()
        self.post = patch.object(fc, "_post").start()
        self.addCleanup(patch.stopall)

    def test_stale_heartbeat_does_not_mean_live_connection(self):
        self.get.return_value = {"HEARTBEAT": heartbeat(True, age=30)}
        self.assertEqual(self.fc.telemetry(), {"connected": False})
        self.post.assert_not_called()
        self.assertFalse(self.fc._streams_asked)

    def test_live_heartbeat_does_not_make_old_position_live(self):
        self.get.return_value = {
            "HEARTBEAT": heartbeat(True),
            "GLOBAL_POSITION_INT": record({"lat": 223187250, "lon": 918131560}, age=30),
        }
        result = self.fc.telemetry()
        self.assertTrue(result["connected"])
        self.assertTrue(result["armed"])
        self.assertNotIn("lat", result)
        self.get.assert_called_once_with(fc._VEH)

    def test_no_gps_fix_does_not_publish_coordinates(self):
        self.get.return_value = {
            "HEARTBEAT": heartbeat(),
            "GLOBAL_POSITION_INT": record({
                "lat": 511897071, "lon": 655236807, "alt": 10000,
                "relative_alt": 880, "hdg": 8800, "vx": 0, "vy": 0}),
            "GPS_RAW_INT": record({
                "fix_type": {"type": "GPS_FIX_TYPE_NO_GPS"},
                "lat": 511897071, "lon": 655236807,
                "satellites_visible": 0, "eph": 9999}),
        }
        result = self.fc.telemetry()
        self.assertTrue(result["connected"])
        self.assertNotIn("lat", result)
        self.assertNotIn("lon", result)
        self.assertEqual(result["sats"], 0)
        self.assertEqual(result["fix"], "GPS_FIX_TYPE_NO_GPS")
        self.assertAlmostEqual(result["rel_alt"], 0.88)

    def test_three_d_fix_publishes_coordinates(self):
        self.get.return_value = {
            "HEARTBEAT": heartbeat(),
            "GPS_RAW_INT": record({
                "fix_type": {"type": "GPS_FIX_TYPE_3D_FIX"},
                "lat": 223187250, "lon": 918131560,
                "satellites_visible": 12, "eph": 80}),
        }
        result = self.fc.telemetry()
        self.assertAlmostEqual(result["lat"], 22.318725)
        self.assertEqual(result["sats"], 12)

    def test_live_position_and_event_driven_home(self):
        self.get.return_value = {
            "HEARTBEAT": heartbeat(),
            "GLOBAL_POSITION_INT": record({"lat": 223187250, "lon": 918131560,
                "alt": 10000, "relative_alt": 5000, "hdg": 65535, "vx": 0, "vy": 0}),
            "HOME_POSITION": record({"latitude": 223187250, "longitude": 918131560}, age=60),
        }
        result = self.fc.telemetry()
        self.assertAlmostEqual(result["lat"], 22.318725)
        self.assertEqual(result["rel_alt"], 5)
        self.assertIsNone(result["hdg"])
        self.assertAlmostEqual(result["home_lon"], 91.813156)

    def test_unknown_timestamp_is_not_live(self):
        for value in (None, {}, {"status": {"time": {"last_update": "bad"}}}):
            self.assertFalse(fc._fresh(value))

    def test_bridge_nanosecond_timestamps_work_on_python_310(self):
        value = heartbeat()
        stamp = value["status"]["time"]["last_update"]
        value["status"]["time"]["last_update"] = stamp.replace("+00:00", "123Z")
        self.assertTrue(fc._fresh(value))

    def test_armed_flag_supports_bridge_encodings(self):
        for value in (128, "MAV_MODE_FLAG_SAFETY_ARMED", {"bits": ["MAV_MODE_FLAG_SAFETY_ARMED"]}):
            self.assertTrue(fc._armed({"base_mode": value}))
        self.assertFalse(fc._armed({"base_mode": 1}))

    def test_stale_link_cannot_send_mode_or_arm_commands(self):
        self.get.return_value = heartbeat(age=30)
        for action in (lambda: self.fc.arm(True), self.fc.toggle_auto_manual):
            with self.assertRaises(fc.FlightControllerError):
                action()
        self.post.assert_not_called()

    @patch.dict(os.environ, {"GCS_MODE": "real"})
    def test_real_start_does_not_implicitly_arm(self):
        self.get.return_value = heartbeat()
        with self.assertRaisesRegex(fc.FlightControllerError, "arm separately"):
            self.fc.start_mission()
        self.post.assert_not_called()

    @patch.dict(os.environ, {"GCS_MODE": "real"})
    @patch.object(fc.time, "sleep")
    def test_real_start_when_armed_only_sends_mission_commands(self, _sleep):
        self.get.return_value = heartbeat(True)
        self.fc.start_mission()
        sent = [call.args[0] for call in self.post.call_args_list]
        self.assertEqual(len(sent), 2)
        self.assertEqual(sent[0]["type"], "MISSION_SET_CURRENT")
        self.assertEqual(sent[1]["command"]["type"], "MAV_CMD_DO_SET_MODE")

    @patch.object(fc.time, "sleep")
    def test_invalid_requested_mission_item_fails_cleanly(self, _sleep):
        self.get.return_value = heartbeat()
        with patch.object(self.fc, "_mack", return_value=None), \
             patch.object(self.fc, "_mreq_counter", side_effect=[None, (1, 9)]):
            result = self.fc.upload_mission([{"command": "MAV_CMD_NAV_WAYPOINT"}])
        self.assertFalse(result["ok"])
        self.assertIn("invalid mission item", result["error"])

    @patch.object(fc.time, "sleep")
    def test_upload_waits_for_new_aircraft_acceptance(self, _sleep):
        self.get.return_value = heartbeat()
        item = {"command": "MAV_CMD_NAV_WAYPOINT", "lat": 22.3, "lon": 91.8, "alt": 10}
        with patch.object(self.fc, "_mack", side_effect=[(4, "MAV_MISSION_ACCEPTED"),
                (4, "MAV_MISSION_ACCEPTED"), (5, "MAV_MISSION_ACCEPTED")]), \
             patch.object(self.fc, "_mreq_counter", side_effect=[None, (1, 0)]):
            result = self.fc.upload_mission([item])
        self.assertEqual(result, {"ok": True, "count": 1})
        types = [call.args[0]["type"] for call in self.post.call_args_list]
        self.assertEqual(types, ["MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_SET_CURRENT"])

    def test_aircraft_rejection_does_not_report_upload_success(self):
        self.get.return_value = heartbeat()
        with patch.object(self.fc, "_mack", side_effect=[None, (1, "MAV_MISSION_DENIED")]), \
             patch.object(self.fc, "_mreq_counter", return_value=None):
            result = self.fc.upload_mission([{"command": "MAV_CMD_NAV_WAYPOINT"}])
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"], "MAV_MISSION_DENIED")
        self.assertEqual(len(self.post.call_args_list), 1)


class CommandErrorTests(unittest.TestCase):
    @patch.dict(os.environ, {"GCS_MODE": "real", "QGC_AUTO_SYNC": "1"})
    def test_real_mission_sync_never_restarts_qgc(self):
        with patch("threading.Thread") as thread:
            fc.MavlinkRestFC().sync_qgc()
        thread.assert_not_called()

    @patch.object(fc.urllib.request, "urlopen", side_effect=OSError("offline"))
    def test_transport_failure_is_not_success(self, _open):
        with self.assertRaises(fc.FlightControllerError):
            fc._post({"type": "COMMAND_LONG"})

    def test_api_returns_json_error_for_failed_command(self):
        handler = object.__new__(server.H)
        handler.path = "/api/command"
        payload = json.dumps({"cmd": "arm"}).encode()
        handler.headers = {"Content-Length": str(len(payload))}
        handler.rfile = io.BytesIO(payload)
        handler._send = Mock()
        with patch.object(server.FC, "arm", side_effect=fc.FlightControllerError("Link lost")):
            handler.do_POST()
        handler._send.assert_called_once_with(503, {"ok": False, "error": "Link lost"})


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.old = camera._latest.copy()
        self.addCleanup(lambda: camera._latest.update(self.old))

    @patch.object(camera.time, "monotonic", return_value=100)
    def test_disconnected_camera_does_not_repeat_stale_frame(self, _clock):
        camera._latest.update(jpeg=b"old frame", ts=90)
        frame, status = camera.frame_snapshot()
        self.assertEqual(frame, camera._WAITING)
        self.assertFalse(status["fresh"])
        self.assertEqual(status["frame_age_s"], 10)

    @patch.object(camera.time, "monotonic", return_value=100)
    def test_reconnected_camera_displays_new_frame(self, _clock):
        camera._latest.update(jpeg=b"new frame", ts=99.5)
        frame, status = camera.frame_snapshot()
        self.assertEqual(frame, b"new frame")
        self.assertTrue(status["fresh"])

    def test_waiting_camera_has_no_frame_age(self):
        camera._latest.update(jpeg=None, ts=0)
        _, status = camera.frame_snapshot()
        self.assertFalse(status["fresh"])
        self.assertIsNone(status["frame_age_s"])

    def test_stalled_decoder_is_killed_before_reconnect(self):
        process = Mock()
        process.poll.return_value = None
        with patch.object(camera.subprocess, "Popen", return_value=process) as spawn, \
             patch.object(camera.time, "monotonic", side_effect=[0, 0, 6]), \
             patch.object(camera.select, "select", return_value=([], [], [])), \
             patch.object(camera.time, "sleep", side_effect=InterruptedError("stop test")):
            with self.assertRaises(InterruptedError):
                camera._ffmpeg_loop()
        process.kill.assert_called_once()
        process.wait.assert_called_once()
        process.stdout.close.assert_called_once()
        self.assertNotIn("stderr", spawn.call_args.kwargs)

    @patch.object(server, "GCS_MODE", "real")
    @patch.object(server.urllib.request, "urlopen", side_effect=OSError("offline"))
    def test_camera_api_reports_missing_bridge(self, _open):
        handler = object.__new__(server.H)
        handler.path = "/api/camera/status"
        handler._send = Mock()
        handler.do_GET()
        code, result = handler._send.call_args.args
        self.assertEqual(code, 200)
        self.assertFalse(result["bridge_connected"])
        self.assertFalse(result["fresh"])


class PixhawkDetectTests(unittest.TestCase):
    def test_raw_usb_node_is_not_used_as_the_serial_port(self):
        by_id = "/dev/serial/by-id/usb-Auterion_PX4_FMU-if00"

        def exists(path):
            return path in ("/dev/pixhawk", by_id)

        def realpath(path):
            if path == "/dev/pixhawk":
                return "/dev/bus/usb/003/007"
            if path == by_id:
                return "/dev/ttyACM0"
            return path

        with patch.object(mav_hub.os.path, "exists", side_effect=exists), \
             patch.object(mav_hub.os.path, "realpath", side_effect=realpath), \
             patch.object(mav_hub.glob, "glob", return_value=[by_id]):
            self.assertFalse(mav_hub._is_serial_tty("/dev/pixhawk"))
            self.assertEqual(mav_hub.find_pixhawk(), by_id)
            self.assertIsNone(mav_hub.find_pixhawk("/dev/bus/usb/003/007"))


if __name__ == "__main__":
    unittest.main()
