"""Offline regressions: pause must not mask emergencies or fabricate health."""
import unittest
from datetime import datetime, timezone
from pulse1_health_v2 import evaluate


class RobotHealthSafetyTest(unittest.TestCase):
    def data(self, sim='PAUSED', guard='BLOCKED', gf='COMPLETE',
             gf_reason='TARGET_REACHED', nominal=True, sensor='STALE',
             emergency=False, supervisor='HEALTHY', fresh=True):
        return {
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'simulation': {'state': sim},
            'nodes': {'pulse1_bridge': True, 'pulse1_v6_motor_guard': True},
            'controller_expected': 'pulse1_v6_motor_guard',
            'linux': {'cpu_percent': 1, 'memory_percent': 1,
                      'disk_percent': 1, 'cpu_cores': 2, 'load_1m': 0.1},
            'web': {'status': 'OK'},
            'topics': {k: {'status': sensor, 'rate_hz': 20.0}
                       for k in ('odometry','imu','lidar')},
            'safety': {
                'fresh': {k: fresh for k in ('guard', 'geofence', 'supervisor', 'cmd')},
                'guard': {'guard_state': guard, 'emergency_latched': emergency,
                          'nominal_stop_latched': nominal, 'segment_id': 'v6-test-001'},
                'geofence': {'action': gf, 'reason': gf_reason, 'segment_id': 'v6-test-001'},
                'supervisor': {'state': supervisor},
                'cmd': [0.0, 0.0]
            }
        }

    def test_certified_paused_no_fake_fail_score(self):
        r = evaluate(self.data())['robot_health']
        self.assertEqual(r['status'], 'PAUSED')
        self.assertIsNone(r['score'])
        self.assertEqual(r['problems'], [])
        self.assertIn('certificato', r['detail'])

    def test_unverified_paused_not_healthy(self):
        r = evaluate(self.data(fresh=False))['robot_health']
        self.assertEqual(r['status'], 'UNKNOWN')
        self.assertIsNone(r['score'])

    def test_guard_abort_while_paused_is_fail(self):
        r = evaluate(self.data(guard='ABORT', nominal=False, emergency=True,
                               gf='PAUSE', gf_reason='REQUEST_LOST_OR_CHANGED'))['robot_health']
        self.assertEqual(r['status'], 'FAIL')
        self.assertIsNone(r['score'])
        self.assertTrue(any('ABORT' in p for p in r['problems']))
        self.assertTrue(any('REQUEST_LOST_OR_CHANGED' in p for p in r['problems']))

    def test_supervisor_emergency_is_fail(self):
        r = evaluate(self.data(supervisor='EMERGENCY_PAUSE'))['robot_health']
        self.assertEqual(r['status'], 'FAIL')

    def test_initial_paused_not_certified_stop(self):
        r = evaluate(self.data(gf='WAIT', gf_reason='NOT_STARTED', nominal=False))['robot_health']
        self.assertEqual(r['status'], 'PAUSED')
        self.assertIn('nessuno STOP', r['detail'])

    def test_running_stale_fails(self):
        r = evaluate(self.data(sim='RUNNING'))['robot_health']
        self.assertEqual(r['status'], 'FAIL')
        self.assertLess(r['score'], 70)

    def test_running_fresh_good(self):
        r = evaluate(self.data(sim='RUNNING',sensor='OK'))['robot_health']
        self.assertEqual(r['status'], 'OK')
        self.assertEqual(r['score'], 100)

    def test_paused_not_nominal_after_rearm(self):
        r = evaluate(self.data(gf='WAIT', gf_reason='REARM_VERIFIED', nominal=False))['robot_health']
        self.assertEqual(r['status'], 'UNKNOWN')

    def test_absent_data_not_paused(self):
        d = self.data()
        d.pop('safety')
        r = evaluate(d)['robot_health']
        self.assertEqual(r['status'], 'UNKNOWN')

    def test_mismatched_stop_ids_not_certified(self):
        d = self.data()
        d['safety']['geofence']['segment_id'] = 'other-segment'
        r = evaluate(d)['robot_health']
        self.assertEqual(r['status'], 'UNKNOWN')
        self.assertIsNone(r['score'])

    def test_zero_command_required(self):
        d = self.data()
        d['safety']['cmd'] = [0.1, 0.0]
        r = evaluate(d)['robot_health']
        self.assertEqual(r['status'], 'UNKNOWN')

    def test_global_stale_unknown(self):
        d = self.data()
        d['timestamp_utc'] = '2020-01-01T00:00:00+00:00'
        r = evaluate(d)['robot_health']
        self.assertEqual(r['status'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
