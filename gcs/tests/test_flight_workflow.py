"""Protocol and operator workflow regressions. All aircraft I/O is mocked."""
import copy
import io
import json
import os
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import fc_iface as fc
import server


def record(message, counter=1):
    return {'message': message, 'status': {'time': {'counter':counter,
        'last_update':datetime.now(timezone.utc).isoformat()}}}


def addressed(**kwargs):
    return dict(target_system=fc.GCS_SYS, target_component=fc.GCS_COMP,
                mission_type={'type':'MAV_MISSION_TYPE_MISSION'}, **kwargs)


class FlightWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.fc = fc.MavlinkRestFC()
        self.get = patch.object(fc, '_get', return_value=None).start()
        self.post = patch.object(fc, '_post').start()
        self.addCleanup(patch.stopall)

    def test_foreign_and_fence_mission_ack_are_ignored(self):
        for changes in ({'target_system':255}, {'target_component':1},
                        {'mission_type':{'type':'MAV_MISSION_TYPE_FENCE'}}):
            message=addressed(mavtype={'type':'MAV_MISSION_ACCEPTED'})
            message.update(changes)
            self.get.return_value=record(message)
            self.assertIsNone(self.fc._mack())
        self.get.return_value=record(addressed(mavtype={'type':'MAV_MISSION_ACCEPTED'}))
        self.assertEqual(self.fc._mack(),(1,'MAV_MISSION_ACCEPTED'))

    def test_foreign_item_requests_are_ignored(self):
        self.get.return_value=record(dict(addressed(seq=0),target_system=255))
        self.assertIsNone(self.fc._mreq_counter())

    def test_premature_acceptance_does_not_complete_upload(self):
        with patch.object(self.fc,'_heartbeat'), patch.object(self.fc,'_mack',side_effect=[None,(1,'MAV_MISSION_ACCEPTED')]), patch.object(self.fc,'_mreq_counter',return_value=None):
            result=self.fc.upload_mission([{'command':'MAV_CMD_NAV_WAYPOINT'}])
        self.assertFalse(result['ok'])
        self.assertEqual([c.args[0]['type'] for c in self.post.call_args_list],['MISSION_COUNT'])

    def test_command_waits_for_matching_target_and_command_ack(self):
        foreign=record({'command':{'type':'MAV_CMD_COMPONENT_ARM_DISARM'}, 'result':{'type':'MAV_RESULT_ACCEPTED'},'target_system':255,'target_component':190})
        matching=record(dict(foreign['message'],target_system=fc.GCS_SYS),2)
        self.get.side_effect=[None,foreign,matching]
        with patch.object(self.fc,'_cmd') as command, patch.object(fc.time,'sleep'):
            result=self.fc.checked_command('MAV_CMD_COMPONENT_ARM_DISARM',(1,0,0,0,0,0,0))
        self.assertTrue(result['accepted'])
        command.assert_called_once()
        self.assertEqual(self.get.call_count,3)

    def test_rejected_command_is_not_success(self):
        with patch.object(self.fc,'_cmd'), patch.object(self.fc,'_wait_record',return_value=record({'result':{'type':'MAV_RESULT_DENIED'}})):
            with self.assertRaisesRegex(fc.FlightControllerError,'rejected'):
                self.fc.checked_command('MAV_CMD_DO_SET_MODE')

    def test_missing_ack_times_out_without_resending(self):
        with patch.object(self.fc,'_cmd') as command, patch.object(fc.time,'monotonic',side_effect=[0,5]):
            with self.assertRaisesRegex(fc.FlightControllerError,'timed out'):
                self.fc.checked_command('MAV_CMD_DO_SET_MODE')
        command.assert_called_once()

    def test_emergency_bypasses_regular_command_lock_and_forces_only_disarm(self):
        self.fc.command_lock.acquire()
        with patch.object(self.fc,'_cmd') as command, patch.object(self.fc,'_wait_record',return_value=record({'result':{'type':'MAV_RESULT_ACCEPTED'}})):
            self.fc.execute('emergency_stop')
        command.assert_called_once_with('MAV_CMD_COMPONENT_ARM_DISARM',(0,21196,0,0,0,0,0))
        self.assertTrue(self.fc.interrupted.is_set())
        self.fc.command_lock.release()

    def test_emergency_api_requires_explicit_confirmation(self):
        h=object.__new__(server.H); h.path='/api/flight'
        raw=json.dumps({'action':'emergency_stop'}).encode()
        h.headers={'Content-Length':str(len(raw))}; h.rfile=io.BytesIO(raw); h._send=Mock()
        with patch.object(server.FC,'execute') as execute:
            h.do_POST()
        self.assertEqual(h._send.call_args.args[0],400)
        execute.assert_not_called()

    def test_hold_without_position_sends_nothing(self):
        with patch.object(self.fc,'telemetry',return_value={'connected':True,'position_hold_available':False}):
            with self.assertRaisesRegex(fc.FlightControllerError,'Position hold unavailable'):
                self.fc.execute('hold')
        self.post.assert_not_called()

    def test_hold_keeps_the_mission(self):
        self.fc.mission_token='existing'
        with patch.object(self.fc,'telemetry',return_value={'position_hold_available':True}), patch.object(self.fc,'checked_command',return_value={'ok':True}) as command:
            self.fc.execute('hold')
        self.assertEqual(self.fc.mission_token,'existing')
        self.assertEqual(command.call_args.args,('MAV_CMD_DO_SET_MODE',(1,4,3,0,0,0,0)))

    @patch.dict(os.environ,{'GCS_MODE':'real'})
    def test_manual_takeover_requires_observed_rc(self):
        with patch.object(self.fc,'telemetry',return_value={'rc_available':False}):
            with self.assertRaisesRegex(fc.FlightControllerError,'RC input'):
                self.fc.execute('manual')
        self.post.assert_not_called()

    def test_uploads_are_serialized(self):
        self.fc.mission_lock.acquire()
        with self.assertRaisesRegex(fc.FlightControllerError,'in progress'):
            self.fc.upload_verified([{}])
        self.post.assert_not_called(); self.fc.mission_lock.release()

    def sample_mission(self):
        want={'command':'MAV_CMD_NAV_WAYPOINT','lat':22.3,'lon':91.8,'alt':45,'p1':15}
        got={'command':{'type':want['command']},'frame':{'type':'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT'},
             'x':223000000,'y':918000000,'z':45,'param1':15,'param2':0,'param3':0,'param4':0}
        self.fc.uploaded_items=[want]; self.fc.mission_token='draft-1'
        return got

    def test_readback_checks_altitude_and_hold_parameters(self):
        got=self.sample_mission()
        with patch.object(self.fc,'download_mission',return_value=([got],78)):
            self.fc.verify_uploaded()
        self.assertEqual(self.fc.mission_id,78)
        for field in ('z','param1','x'):
            bad=copy.deepcopy(got);bad[field]+=1
            with patch.object(self.fc,'download_mission',return_value=([bad],79)):
                with self.assertRaisesRegex(fc.FlightControllerError,'differs'):
                    self.fc.verify_uploaded()
            self.assertIsNone(self.fc.mission_token)

    def test_start_rechecks_readback_and_never_arms(self):
        self.sample_mission()
        with patch.object(self.fc,'telemetry',return_value={'connected':True,'position_hold_available':True}), patch.object(self.fc,'verify_uploaded') as verify, patch.object(self.fc,'_heartbeat',return_value={'message':{'base_mode':128}}), patch.object(self.fc,'checked_command',return_value={'ok':True}) as command:
            self.fc.execute('start','draft-1')
        verify.assert_called_once()
        self.assertEqual([c.args[0] for c in command.call_args_list],['MAV_CMD_DO_SET_MISSION_CURRENT','MAV_CMD_DO_SET_MODE'])

    def test_resume_does_not_reset_waypoint(self):
        self.sample_mission()
        with patch.object(self.fc,'telemetry',return_value={'connected':True,'position_hold_available':True}), patch.object(self.fc,'verify_uploaded'), patch.object(self.fc,'_heartbeat',return_value={'message':{'base_mode':128}}), patch.object(self.fc,'checked_command',return_value={'ok':True}) as command:
            self.fc.execute('resume','draft-1')
        command.assert_called_once_with('MAV_CMD_DO_SET_MODE',(1,4,4,0,0,0,0))

    def test_start_without_token_or_armed_state_is_rejected(self):
        self.sample_mission()
        with self.assertRaisesRegex(fc.FlightControllerError,'not verified'):
            self.fc.execute('start','other-draft')
        with patch.object(self.fc,'verify_uploaded'), patch.object(self.fc,'_heartbeat',return_value={'message':{'base_mode':0}}):
            with self.assertRaisesRegex(fc.FlightControllerError,'Arm separately'):
                self.fc.execute('start','draft-1')
        self.post.assert_not_called()

    def test_complete_upload_readback_and_go_protocol(self):
        records = {'HEARTBEAT': record({'base_mode':128})}
        aircraft = []
        counter = 0
        sent = []
        def update(name, message):
            nonlocal counter
            counter += 1
            records[name] = record(message, counter)
        def receive(message):
            sent.append(message['type'])
            name = message['type']
            if name == 'MISSION_COUNT':
                aircraft.clear()
                update('MISSION_REQUEST_INT', addressed(seq=0))
            elif name == 'MISSION_ITEM_INT':
                aircraft.append(dict(message))
                if len(aircraft) == 2:
                    update('MISSION_ACK', addressed(mavtype={'type':'MAV_MISSION_ACCEPTED'}))
                else: update('MISSION_REQUEST_INT', addressed(seq=1))
            elif name == 'MISSION_REQUEST_LIST':
                update('MISSION_COUNT', addressed(count=len(aircraft), opaque_id=100))
            elif name == 'MISSION_REQUEST_INT':
                item = dict(aircraft[message['seq']])
                item.update(target_system=fc.GCS_SYS, target_component=fc.GCS_COMP)
                update('MISSION_ITEM_INT', item)
            elif name == 'COMMAND_LONG':
                update('COMMAND_ACK', {'command':message['command'],
                    'result':{'type':'MAV_RESULT_ACCEPTED'},'target_system':fc.GCS_SYS,'target_component':fc.GCS_COMP})
        self.get.side_effect = lambda path, **kwargs: records.get(path.rsplit('/',1)[-1])
        self.post.side_effect = receive
        items=[{'command':'MAV_CMD_NAV_TAKEOFF','lat':22.31,'lon':91.81,'alt':45},
               {'command':'MAV_CMD_NAV_WAYPOINT','lat':22.32,'lon':91.82,'alt':45,'p1':15}]
        with patch.object(fc.time,'sleep'), patch.object(self.fc,'telemetry',return_value={'connected':True,'position_hold_available':True}):
            uploaded=self.fc.upload_verified(items)
            self.assertTrue(uploaded['verified'])
            self.assertEqual(self.fc.mission_id,100)
            started=self.fc.execute('start',uploaded['mission_token'])
        self.assertTrue(started['accepted'])
        self.assertEqual(sent.count('MISSION_REQUEST_LIST'),2)
        self.assertEqual(sent.count('MISSION_ITEM_INT'),2)
        self.assertEqual(sent.count('COMMAND_LONG'),2)

    def test_inspection_encodes_unspecified_mavlink_parameters_as_json_null(self):
        with patch.object(self.fc, '_heartbeat'), patch.object(self.fc, 'download_mission',
                return_value=([{'param4':float('nan'), 'z':45}], 12)):
            result = self.fc.inspect_mission()
        self.assertIsNone(result['items'][0]['param4'])
        json.dumps(result, allow_nan=False)

    def test_link_loss_revokes_backend_mission_authorization(self):
        self.fc.mission_token = 'old-session'
        self.fc.mission_id = 99
        self.get.return_value = {}
        self.assertEqual(self.fc.telemetry(), {'connected': False})
        self.assertIsNone(self.fc.mission_token)
        self.assertIsNone(self.fc.mission_id)
        self.assertTrue(self.fc.interrupted.is_set())

    def test_start_and_resume_reject_missing_position(self):
        for action in ('start', 'resume'):
            self.sample_mission()
            with patch.object(self.fc,'verify_uploaded'), patch.object(self.fc,'_heartbeat',return_value={'message':{'base_mode':128}}), patch.object(self.fc,'telemetry',return_value={'connected':True,'position_hold_available':False}), patch.object(self.fc,'checked_command') as command:
                with self.assertRaisesRegex(fc.FlightControllerError,'Navigation unavailable'):
                    self.fc.execute(action,'draft-1')
                command.assert_not_called()

    def test_position_loss_during_start_prevents_auto_mode(self):
        self.sample_mission()
        with patch.object(self.fc,'verify_uploaded'), patch.object(self.fc,'_heartbeat',return_value={'message':{'base_mode':128}}), patch.object(self.fc,'telemetry',side_effect=[{'connected':True,'position_hold_available':True},{'connected':True,'position_hold_available':False}]), patch.object(self.fc,'checked_command') as command:
            with self.assertRaisesRegex(fc.FlightControllerError,'Navigation unavailable'):
                self.fc.execute('start','draft-1')
        command.assert_called_once_with('MAV_CMD_DO_SET_MISSION_CURRENT',(0,1,0,0,0,0,0))

    def test_rtl_requires_position_and_valid_home(self):
        for t in ({'connected':False}, {'connected':True,'position_hold_available':True},
                  {'connected':True,'position_hold_available':True,'home_lat':float('nan'),'home_lon':91}):
            with patch.object(self.fc,'telemetry',return_value=t), patch.object(self.fc,'checked_command') as command:
                with self.assertRaises(fc.FlightControllerError): self.fc.execute('rtl')
                command.assert_not_called()
        with patch.object(self.fc,'telemetry',return_value={'connected':True,'position_hold_available':True,'home_lat':22,'home_lon':91}), patch.object(self.fc,'checked_command') as command:
            self.fc.execute('rtl')
            command.assert_called_once()

    def test_return_waypoints_preserve_destination_hold(self):
        items=server.route_to_mission([[22,91],[22.1,91.1]],40,rtl=False,hold_s=12,return_wps=[[22,91]])
        self.assertEqual(items[1]['p1'],12)
        self.assertEqual(items[2]['p1'],0)

    def test_qgc_is_never_restarted_in_any_mode(self):
        for mode in ('real','sitl'):
            with patch.dict(os.environ,{'GCS_MODE':mode,'QGC_AUTO_SYNC':'1'}), patch.object(fc.subprocess,'run') as run, patch.object(fc.subprocess,'Popen') as popen:
                self.fc.sync_qgc()
                run.assert_not_called(); popen.assert_not_called()

if __name__=='__main__': unittest.main()
