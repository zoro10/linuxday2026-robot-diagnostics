#!/usr/bin/env python3
"""Pure offline policy validation: no ROS, no Gazebo, no rearm."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'scripts'))
from pulse1_v6_abort_resume_policy import (
    WP2_ODOM_ABORT_ID, WP2_ODOM_ABORT_REASON, WP2_ODOM_ABORT_GAZEBO,
    capture_wp2_odom_abort, verify_bootstrap,
)

G = dict(guard_state='ABORT', emergency_latched=True,
         nominal_stop_latched=False, v6_armed=True,
         cmd_vel_exclusive=True, segment_id=None,
         expected_next_segment_id=WP2_ODOM_ABORT_ID,
         permitted_linear_m_s=0.0, permitted_angular_rad_s=0.0)
F = dict(armed=True, ready_for_motion=False, action='PAUSE',
         reason=WP2_ODOM_ABORT_REASON, segment_id=WP2_ODOM_ABORT_ID,
         pause_ack=True, motor_commands=False)
S = dict(state='EMERGENCY_PAUSE', pause_ack=True, motor_commands=False)
P = WP2_ODOM_ABORT_GAZEBO

class WP2OdomAbortTest(unittest.TestCase):
    def audit(self):
        return capture_wp2_odom_abort(G, F, S, True, (0., 0.), P, now=1000.)

    def clean(self):
        g = {**G, 'guard_state': 'BLOCKED', 'emergency_latched': False,
             'expected_next_segment_id': None}
        f = {**F, 'action': 'WAIT', 'reason': 'NOT_STARTED',
             'segment_id': None, 'rearm_next_segment_id': None}
        s = {'state': 'HEALTHY'}
        return g, f, s

    def test_valid_audit_is_always_non_nominal(self):
        a = self.audit()
        self.assertEqual(a['incident'], 'MID_ADVANCE_ODOMETRY_STALE_ABORT')
        self.assertTrue(a['partial_motion_observed'])
        self.assertFalse(a['nominal_stop'])

    def test_capture_denies_wrong_guard_id(self):
        with self.assertRaises(ValueError):
            capture_wp2_odom_abort({**G, 'expected_next_segment_id': 'other'},
                                   F, S, True, (0., 0.), P, now=1000.)

    def test_capture_denies_missing_guard_latch(self):
        with self.assertRaises(ValueError):
            capture_wp2_odom_abort({**G, 'emergency_latched': False},
                                   F, S, True, (0., 0.), P, now=1000.)

    def test_capture_denies_wrong_geofence_reason(self):
        with self.assertRaises(ValueError):
            capture_wp2_odom_abort(G, {**F, 'reason': 'OBSERVER_STALE'},
                                   S, True, (0., 0.), P, now=1000.)

    def test_capture_denies_unconfirmed_pause(self):
        with self.assertRaises(ValueError):
            capture_wp2_odom_abort(G, {**F, 'pause_ack': False},
                                   S, True, (0., 0.), P, now=1000.)

    def test_capture_denies_nonzero_motion_or_unpaused(self):
        for paused, cmd in ((False, (0., 0.)), (True, (.01, 0.))):
            with self.assertRaises(ValueError):
                capture_wp2_odom_abort(G, F, S, paused, cmd, P, now=1000.)

    def test_capture_denies_pose_changed(self):
        with self.assertRaises(ValueError):
            capture_wp2_odom_abort(G, F, S, True, (0., 0.),
                                   (P[0]+.04, P[1], P[2]), now=1000.)

    def test_clean_restart_accepted(self):
        g, f, s = self.clean()
        self.assertTrue(verify_bootstrap(self.audit(), g, f, s,
                                         True, (0., 0.), P, now=1001.))

    def test_still_aborted_rejected(self):
        with self.assertRaises(ValueError):
            verify_bootstrap(self.audit(), G, F, S, True, (0., 0.), P, now=1001.)

    def test_bootstrap_rejects_old_audit(self):
        g, f, s = self.clean()
        with self.assertRaisesRegex(ValueError, 'STALE'):
            verify_bootstrap(self.audit(), g, f, s,
                             True, (0., 0.), P, now=23000.)

    def test_bootstrap_rejects_pose_changed(self):
        g, f, s = self.clean()
        with self.assertRaises(ValueError):
            verify_bootstrap(self.audit(), g, f, s,
                             True, (0., 0.), (P[0]+.02,P[1],P[2]), now=1001.)

    def test_bootstrap_rejects_nominal_forgery(self):
        g, f, s = self.clean()
        audit = self.audit(); audit['nominal_stop'] = True
        with self.assertRaises(ValueError):
            verify_bootstrap(audit, g, f, s, True, (0., 0.), P, now=1001.)

    def test_bootstrap_rejects_unfinished_next_segment(self):
        g, f, s = self.clean(); g['expected_next_segment_id'] = WP2_ODOM_ABORT_ID
        with self.assertRaises(ValueError):
            verify_bootstrap(self.audit(), g, f, s, True, (0., 0.), P, now=1001.)

if __name__ == '__main__':
    unittest.main(verbosity=2)
