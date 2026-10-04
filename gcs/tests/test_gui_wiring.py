"""HTTP-handler and mission wiring checks without sockets or a flight controller."""
import io
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server


class GuiApiTests(unittest.TestCase):
    def post(self, path, payload):
        handler = object.__new__(server.H)
        handler.path = path
        raw = json.dumps(payload).encode()
        handler.headers = {'Content-Length': str(len(raw))}
        handler.rfile = io.BytesIO(raw)
        handler._send = Mock()
        handler.do_POST()
        return handler._send.call_args.args

    def test_bad_payloads_return_json_instead_of_dropping_connection(self):
        for payload in ([], None, {'route': []}, {'route': [[None, 91], [22, 91]]}):
            code, result = self.post('/api/mission/upload', payload)
            self.assertEqual(code, 400)
            self.assertFalse(result['ok'])

    def test_invalid_boundary_is_not_persisted(self):
        with patch.object(server, 'save_op_area') as save:
            for polygon in ([[22, 91]], [[22, 91], [23, 91], [float('nan'), 91]]):
                self.assertEqual(self.post('/api/operating_area', {'polygon': polygon})[0], 400)
            save.assert_not_called()

    @patch.object(server, 'GCS_MODE', 'real')
    def test_real_mode_cannot_launch_or_kill_simulator_processes(self):
        with patch.object(server.subprocess, 'Popen') as spawn, patch.object(server.subprocess, 'run') as run:
            for path in ('/api/sitl/start', '/api/sitl/stop', '/api/gazebo/follow'):
                code, result = self.post(path, {})
                self.assertEqual(code, 409)
                self.assertFalse(result['ok'])
            spawn.assert_not_called()
            run.assert_not_called()

    def mission_fixture(self):
        home = [server.BASE['lat'], server.BASE['lon']]
        goal = [home[0] + .005, home[1] + .005]
        route = server.plan_multi(home, [{'lat': goal[0], 'lon': goal[1], 'alt': 45}], 45)['route']
        return home, {'route': route, 'rtl': True, 'rtl_alt': 60}

    @patch.object(server, 'GCS_MODE', 'real')
    def test_upload_requires_aircraft_home_and_ignores_browser_home(self):
        home, payload = self.mission_fixture()
        payload.update(home_lat=1, home_lon=2)
        with patch.object(server, 'FC') as fc:
            fc.telemetry.return_value = {'connected': True, 'armed': False}
            code, result = self.post('/api/mission/upload', payload)
            self.assertEqual(code, 503)
            self.assertIn('home position', result['error'])
            fc.upload_verified.assert_not_called()
            fc.telemetry.return_value.update(home_lat=home[0], home_lon=home[1])
            fc.upload_verified.return_value = {'ok': True, 'count': 7}
            self.assertEqual(self.post('/api/mission/upload', payload)[0], 200)
            items = fc.upload_verified.call_args.args[0]
            self.assertEqual([items[-1]['lat'], items[-1]['lon']], home)
            self.assertEqual(items[0]['command'], 'MAV_CMD_NAV_TAKEOFF')

    def test_invalid_altitude_never_reaches_aircraft(self):
        _, payload = self.mission_fixture()
        with patch.object(server, 'FC') as fc:
            for altitude in (float('nan'), float('inf'), -10, 'invalid'):
                payload['alt'] = altitude
                self.assertEqual(self.post('/api/mission/upload', payload)[0], 400)
            fc.upload_verified.assert_not_called()

    def test_route_is_rechecked_after_boundary_changes(self):
        _, payload = self.mission_fixture()
        with patch.object(server, 'get_planner') as planner, patch.object(server, 'FC') as fc:
            planner.return_value.segment_blocked.return_value = True
            code, result = self.post('/api/mission/upload', payload)
            self.assertEqual(code, 400)
            self.assertIn('plan again', result['error'])
            fc.upload_verified.assert_not_called()

    def test_return_leg_climbs_at_last_waypoint_and_uses_planner_detour(self):
        planner = Mock()
        planner.is_blocked.return_value = False
        planner.segment_blocked.return_value = False
        planner.plan.return_value = [(22.32, 91.82), (22.33, 91.81), (22.31, 91.80)]
        with patch.object(server, 'get_planner', return_value=planner):
            items = server.route_to_mission([[22.31, 91.80, 20], [22.32, 91.82, 30]], 45,
                                           rtl_alt=60, home_lat=22.31, home_lon=91.80)
        self.assertEqual([(i['lat'], i['lon'], i['alt']) for i in items[2:-1]],
                         [(22.32, 91.82, 60), (22.33, 91.81, 60), (22.31, 91.80, 60)])
        self.assertEqual(items[-1]['command'], 'MAV_CMD_NAV_LAND')

    def test_reposition_requires_position_before_sending(self):
        with patch.object(server, 'get_planner') as planner, patch.object(server, 'FC') as fc:
            planner.return_value.is_blocked.return_value = False
            fc.telemetry.return_value = {'connected': True}
            code, result = self.post('/api/command', {'cmd': 'reposition', 'lat': 22.31, 'lon': 91.81})
            self.assertEqual(code, 503)
            self.assertIn('GPS', result['error'])
            fc.reposition.assert_not_called()


if __name__ == '__main__':
    unittest.main()
