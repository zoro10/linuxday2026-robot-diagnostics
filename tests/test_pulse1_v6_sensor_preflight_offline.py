#!/usr/bin/env python3
"""Pure, no-ROS regression tests; do NOT arm or drive PULSE-1."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))
from pulse1_v6_sensor_preflight import check_sensor_gate
from pulse1_v6_safety_contract import observer_healthy, turn_clear, front_clear


def observer(sector_distance=3.0):
    entry = {'clear': sector_distance is None, 'distance_m': sector_distance}
    return {'source': 'avoid_observer', 'mode': 'AVOID',
            'state': 'SENSOR_READY', 'motor_commands': False,
            'odom_ok': True, 'lidar_ok': True, 'rear_lidar_ok': True, 'imu_ok': True,
            'readings': {'front': {'clear': False, 'distance_m': 4.0,
                                   'sectors': {k: dict(entry) for k in ('front','left','right')}},
                         'rear': {'sectors': {k: dict(entry) for k in ('front','left','right')}}}}


class SensorPreflightTest(unittest.TestCase):
    def test_healthy_and_turn_allowed(self):
        obs = observer(3.0)
        self.assertTrue(turn_clear(obs))
        self.assertEqual(check_sensor_gate(obs, .1, 'TURN'), (True, 'TURN_CLEAR'))

    def test_advancing_does_not_require_rear_clearance(self):
        obs = observer(3.0)
        obs['readings']['rear']['sectors']['left']['distance_m'] = 0.13
        self.assertEqual(check_sensor_gate(obs, .1, 'ADVANCE'), (True,'FRONT_CLEAR'))
        self.assertFalse(check_sensor_gate(obs, .1, 'TURN')[0])

    def test_healthy_is_not_necessarily_turn_clear(self):
        obs = observer(1.5)
        self.assertTrue(observer_healthy(obs, .1))
        self.assertFalse(turn_clear(obs))
        okay, reason = check_sensor_gate(obs, .1, 'TURN')
        self.assertFalse(okay)
        self.assertIn('TOO_CLOSE', reason)

    def test_all_infinite_rejected_without_altering_contract(self):
        obs = observer(None)
        self.assertTrue(observer_healthy(obs, .1))
        self.assertFalse(turn_clear(obs))
        self.assertEqual(check_sensor_gate(obs, .1, 'TURN'),
                         (False, 'TURN_ALL_SECTORS_NO_FINITE_RETURN'))

    def test_stale_observer_rejected(self):
        obs=observer(3.0)
        self.assertEqual(check_sensor_gate(obs, 1., 'TURN'),
                         (False, 'OBSERVER_NOT_HEALTHY'))

    def test_one_missing_sector(self):
        obs=observer(3.0)
        del obs['readings']['rear']['sectors']['left']
        self.assertEqual(check_sensor_gate(obs, .1, 'TURN'),
                         (False, 'TURN_REAR_LEFT_MISSING'))

    def test_unknown_maneuver_rejected(self):
        self.assertEqual(check_sensor_gate(observer(), .1, 'SWERVE'),
                         (False,'INVALID_MANEUVER'))

    def test_front_clearance_denied(self):
        obs=observer(3.0)
        obs['readings']['front']['distance_m'] = 1.0
        self.assertFalse(front_clear(obs))
        self.assertEqual(check_sensor_gate(obs, .1, 'ADVANCE'),
                         (False,'FRONT_CLEARANCE_DENIED'))

    def test_sensor_gate_never_overrides_contract(self):
        for dist in (None, 1.0, 1.8, 2.6, 2.7, 3.0, 20.0):
            obs=observer(dist)
            ok,_=check_sensor_gate(obs, .1, 'TURN')
            self.assertEqual(ok, observer_healthy(obs,.1) and turn_clear(obs))


if __name__ == '__main__':
    unittest.main(verbosity=2)
