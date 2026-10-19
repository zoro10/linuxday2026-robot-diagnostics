import unittest
from pulse1_s1_geofence_watchdog_v2 import trial_done, trial_drive


def request():
    return {
        'source': 'avoid_navigator',
        'mode': 'AVOID',
        'state': 'HOLD',
        'trial_phase': 'DONE',
        'trial_reason': 'TARGET_REACHED',
        'motor_commands': False,
        'proposed_linear_m_s': 0.0,
        'proposed_angular_rad_s': 0.0,
    }


class StopContractTests(unittest.TestCase):
    def test_valid_done(self):
        self.assertTrue(trial_done(request(), 0.1))

    def test_stale_done_rejected(self):
        self.assertFalse(trial_done(request(), 0.51))

    def test_missing_and_wrong_source(self):
        self.assertFalse(trial_done(None, None))
        d = request()
        d['source'] = 'untrusted'
        self.assertFalse(trial_done(d, 0.1))

    def test_drive_not_completion(self):
        d = request()
        d['trial_phase'] = 'DRIVE'
        self.assertFalse(trial_done(d, 0.1))

    def test_hold_without_target_not_completion(self):
        d = request()
        d['trial_reason'] = 'INVALID_POSE'
        self.assertFalse(trial_done(d, 0.1))

    def test_nonzero_motion_cannot_complete(self):
        d = request()
        d['proposed_linear_m_s'] = 0.1
        self.assertFalse(trial_done(d, 0.1))

    def test_bad_state_cannot_complete(self):
        d = request()
        d['state'] = 'GO_TO_GOAL'
        self.assertFalse(trial_done(d, 0.1))

    def test_done_not_drive(self):
        self.assertFalse(trial_drive(request(), 0.1))


if __name__ == '__main__':
    unittest.main()
