import importlib.util
from pathlib import Path
import unittest
spec = importlib.util.spec_from_file_location('preflight', Path(__file__).resolve().parents[2] / 'launch/session_preflight.py')
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class PreflightTests(unittest.TestCase):
    def test_simulation_rejects_real_relay(self):
        for stage in ('1','2'):
            for script in ('mav_hub.py','rtsp_bridge.py'):
                self.assertIsNotNone(preflight.conflict(stage,['python3','/repo/gcs/'+script]))

    def test_planning_rejects_rc_bridge_but_manual_allows_it(self):
        argv=['python3','/repo/gcs/rc_to_sitl.py']
        self.assertIsNotNone(preflight.conflict('1',argv))
        self.assertIsNone(preflight.conflict('2',argv))

    def test_real_rejects_simulation(self):
        self.assertIsNotNone(preflight.conflict('3',['/repo/build/bin/px4','-d']))

    def test_unrelated_commands_and_script_text_do_not_match(self):
        for argv in (['ruby','app.rb'],['bash','-c','echo mav_hub.py'],['python3','/repo/gcs/server.py','8080']):
            for stage in ('1','2','3'):
                self.assertIsNone(preflight.conflict(stage,argv))
