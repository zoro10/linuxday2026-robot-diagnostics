import unittest

from pulse1_motor_guard_logic import guarded_motion


class MotorGuardMotionTests(unittest.TestCase):

    def test_forward_motion(self):
        self.assertEqual(
            guarded_motion(0.18, 0.0, 0.1, active=True),
            (0.18, 0.0)
        )

    def test_turn_left(self):
        self.assertEqual(
            guarded_motion(0.0, 0.2, 0.1, active=True),
            (0.0, 0.2)
        )

    def test_turn_right(self):
        self.assertEqual(
            guarded_motion(0.0, -0.2, 0.1, active=True),
            (0.0, -0.2)
        )

    def test_speed_limits(self):
        self.assertEqual(
            guarded_motion(1.0, 2.0, 0.1, active=True),
            (0.18, 0.35)
        )

    def test_negative_angular_limit(self):
        self.assertEqual(
            guarded_motion(0.1, -2.0, 0.1, active=True),
            (0.1, -0.35)
        )

    def test_stale_request(self):
        self.assertEqual(
            guarded_motion(0.1, 0.2, 1.0, active=True),
            (0.0, 0.0)
        )

    def test_disarmed(self):
        self.assertEqual(
            guarded_motion(0.1, 0.2, 0.1, active=False),
            (0.0, 0.0)
        )

    def test_invalid_angular_blocks_both(self):
        self.assertEqual(
            guarded_motion(0.1, float("nan"), 0.1, active=True),
            (0.0, 0.0)
        )

    def test_invalid_linear_blocks_both(self):
        self.assertEqual(
            guarded_motion(None, 0.2, 0.1, active=True),
            (0.0, 0.0)
        )

    def test_reverse_not_yet_authorized(self):
        self.assertEqual(
            guarded_motion(-0.1, 0.2, 0.1, active=True),
            (0.0, 0.0)
        )

    def test_invalid_age(self):
        self.assertEqual(
            guarded_motion(0.1, 0.2, float("inf"), active=True),
            (0.0, 0.0)
        )


if __name__ == "__main__":
    unittest.main()
