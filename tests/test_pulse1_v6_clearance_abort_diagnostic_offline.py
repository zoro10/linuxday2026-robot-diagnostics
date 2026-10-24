#!/usr/bin/env python3
"""Offline regression tests of V6 Geofence evidence captured at an abort."""
import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts'))
from pulse1_v6_safety_contract import V6GeofenceSegment, turn_clearance_detail
from pulse1_v6_clearance_abort_diagnostic import clearance_abort_snapshot


SID = 'v6-route-02f8670abf24130e7433'


def healthy_observer(rear_left=1.99):
    readings = {}
    for sensor in ('front', 'rear'):
        sectors = {k: {'clear': True, 'distance_m': None} for k in ('front', 'left', 'right')}
        readings[sensor] = {'sectors': sectors}
    readings['rear']['sectors']['left'] = {'clear': False, 'distance_m': rear_left}
    return {'source': 'avoid_observer', 'mode': 'AVOID', 'state': 'SENSOR_READY',
            'motor_commands': False,
            'odom_ok': True, 'lidar_ok': True,
            'rear_lidar_ok': True, 'imu_ok': True,
            'ages_s': {'odom': .08, 'front': .12, 'rear': .10, 'imu': .09},
            'readings': readings}


def tick_turn(observer):
    core = V6GeofenceSegment(phase='MONITOR', request_id=SID, kind='TURN',
                            waypoint_index=2, origin=(2.11, 2.83, 1.4),
                            strict_v6_guard=True, started_s=100.0)
    old = core.phase
    result = core.tick(request={'segment_id': SID}, request_age_s=.01,
                       pose=(2.11, 2.83, 1.4), pose_age_s=.02,
                       observer=observer, observer_age_s=.03,
                       now_s=100.1, guard=None, guard_age_s=None,
                       cmd=(0,0), cmd_age_s=.01,
                       world_paused=False, world_age_s=.01)
    return old, result, core


class ClearanceAbortDiagnosticTest(unittest.TestCase):
    def make_failure(self):
        observer = healthy_observer(rear_left=.3)
        old, result, core = tick_turn(observer)
        self.assertEqual(old, 'MONITOR')
        self.assertEqual(result['action'], 'PAUSE')
        self.assertEqual(result['reason'], 'OBSTACLE_CLEARANCE_LOST')
        snapshot = clearance_abort_snapshot(
            result=result, observer=observer, observer_age_s=.03,
            pose=(2.11, 2.83, 1.4), now_s=100.1)
        self.assertEqual(core.phase, 'PAUSE')
        return observer, result, snapshot

    def test_real_geofence_monitors_then_latches_pause(self):
        observer, result, snapshot = self.make_failure()
        self.assertEqual(snapshot['geofence_reason'], result['reason'])
        self.assertEqual(snapshot['segment_id'], SID)
        self.assertEqual(snapshot['kind'], 'TURN')
        self.assertEqual(snapshot['waypoint_index'], 2)
        self.assertEqual(snapshot['pose_odom'], [2.11, 2.83, 1.4])

    def test_sector_failure_detailed_and_exact(self):
        observer, _result, snapshot = self.make_failure()
        detail = turn_clearance_detail(observer)[1]
        self.assertEqual(snapshot['clearance_detail'], detail)
        self.assertIn('TURN_REAR_LEFT_TOO_CLOSE', detail)
        self.assertEqual(snapshot['sectors']['rear']['left']['distance_m'], .3)

    def test_snapshot_frozen_when_observer_later_expires(self):
        observer, _result, snapshot = self.make_failure()
        observer['state'] = 'WAIT_SENSORS'
        observer['readings']['rear']['sectors']['left']['distance_m'] = None
        observer['ages_s']['rear'] = 450.0
        self.assertEqual(snapshot['observer_state'], 'SENSOR_READY')
        self.assertEqual(snapshot['observer_sensor_ages_s']['rear'], .1)
        self.assertEqual(snapshot['sectors']['rear']['left']['distance_m'], .3)

    def test_never_fabricate_diagnostic_for_non_clearance_pause(self):
        snap = clearance_abort_snapshot(
            result={'action':'PAUSE', 'reason':'OBSERVER_STALE'},
            observer=healthy_observer(), observer_age_s=.1, pose=(0,0,0), now_s=0)
        self.assertIsNone(snap)

    def test_never_fabricate_diagnostic_for_non_abort(self):
        snap = clearance_abort_snapshot(
            result={'action':'MONITOR', 'reason':'WITHIN_SEGMENT_ENVELOPE'},
            observer=healthy_observer(), observer_age_s=.1, pose=(0,0,0), now_s=0)
        self.assertIsNone(snap)

    def test_invalid_missing_sectors_explicit_and_fail_closed(self):
        observer = healthy_observer()
        del observer['readings']['rear']['sectors']
        _old, result, _core = tick_turn(observer)
        self.assertEqual(result['reason'],'OBSTACLE_CLEARANCE_LOST')
        snap=clearance_abort_snapshot(result=result,observer=observer,
                                      observer_age_s=.1,pose=(0,0,0),now_s=1)
        self.assertEqual(snap['clearance_detail'],'TURN_REAR_SECTORS_MISSING')
        self.assertIsNone(snap['sectors']['rear']['left']['clear'])

    def test_nonfinite_values_json_safe(self):
        _obs, result, _snapshot = self.make_failure()
        observer = healthy_observer()
        observer['readings']['rear']['sectors']['left']['distance_m'] = float('nan')
        observer['ages_s']['rear'] = float('inf')
        snap=clearance_abort_snapshot(result=result,observer=observer,
                                      observer_age_s=float('nan'),
                                      pose=(0, float('inf'), 0), now_s=float('nan'))
        self.assertIsNone(snap['sectors']['rear']['left']['distance_m'])
        self.assertIsNone(snap['observer_sensor_ages_s']['rear'])
        self.assertIsNone(snap['pose_odom'])
        json.dumps(snap, allow_nan=False)

    def test_front_clearance_failure_is_recorded(self):
        result={'action':'PAUSE', 'reason':'OBSTACLE_CLEARANCE_LOST',
                'kind':'ADVANCE', 'segment_id':SID, 'waypoint_index':3}
        observer=healthy_observer()
        observer['readings']['front'] = {'distance_m':0.4,'clear':False}
        snap=clearance_abort_snapshot(result=result,observer=observer,
                                      observer_age_s=.01,pose=(0,0,0),now_s=1)
        self.assertIn('ADVANCE_FRONT_CLEARANCE_DENIED', snap['clearance_detail'])

    def test_pause_before_disk_io_in_ros_owner(self):
        # Prevent diagnostic file I/O from delaying the emergency pause.
        src = (ROOT / 'scripts/pulse1_v6_ros_safety.py').read_text()
        body = src.split('class V6Geofence(',1)[1].split('class V6MotorGuard(',1)[0]
        self.assertLess(body.index('self.start_pause()  # CRITICAL'),
                        body.index('self.publisher.publish(msg)'))
        self.assertLess(body.index('self.publisher.publish(msg)'),
                        body.index('path.write_text('))
        self.assertIn('status[\'clearance_abort_diagnostic\']', body)

    def test_one_without_lidar_returns_reason(self):
        _obs, result, _snapshot = self.make_failure()
        snap=clearance_abort_snapshot(result=result,observer=None,
                                      observer_age_s=None,pose=None,now_s=None)
        self.assertEqual(snap['clearance_detail'],'TURN_READINGS_MISSING')
        json.dumps(snap, allow_nan=False)


if __name__ == '__main__':
    unittest.main(verbosity=2)
