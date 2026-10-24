#!/usr/bin/env python3
"""Pure policy and source-order regression; does not start ROS or Gazebo."""
import ast
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts'))
from pulse1_v6_abort_resume_policy import verify_stationary_odom


class PostPoseSensorRefresh(unittest.TestCase):
    def test_stationary(self):
        self.assertTrue(verify_stationary_odom((2.105, 2.812, 1.56),
                                               (2.106, 2.813, 1.562)))

    def test_motion_rejected(self):
        with self.assertRaisesRegex(ValueError, 'ROVER_MOVED'):
            verify_stationary_odom((2.1, 2.8, 1.56),
                                   (2.1, 2.811, 1.56))

    def test_rotation_rejected(self):
        with self.assertRaisesRegex(ValueError, 'ROVER_MOVED'):
            verify_stationary_odom((2.1, 2.8, 1.56),
                                   (2.1, 2.8, 1.58))

    def test_invalid_pose_rejected(self):
        with self.assertRaises(ValueError):
            verify_stationary_odom((2.1, 2.8, float('nan')),
                                   (2.1, 2.8, 1.56))

    def test_bootstrap_step_occurs_before_sensor_gate(self):
        text = (ROOT / 'scripts/pulse1_v6_single_segment.py').read_text()
        tree = ast.parse(text)
        self.assertIsNotNone(tree)
        bootstrap = text.index("abort_mode = os.environ.get('PULSE1_V6_ABORT_BOOTSTRAP') == 'YES'")
        refresh = text.index('dt, _ = runner.physics(250, warmup=True)',bootstrap)
        validate = text.index('verify_stationary_odom(pre_refresh_odom, post_refresh_odom)',refresh)
        gate = text.index('sensor_ok, sensor_reason = check_sensor_gate(', validate)
        publish = text.index('runner.req=request',gate)
        self.assertTrue(bootstrap < refresh < validate < gate < publish)


if __name__ == '__main__':
    unittest.main(verbosity=2)
