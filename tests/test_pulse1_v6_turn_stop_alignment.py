import math
import unittest

from pulse1_v6_segment_fsm import Segment, V6SegmentGate
from pulse1_v6_turn_policy import TURN_LIMIT, turn_reached


class TestV6TurnStopAlignment(unittest.TestCase):

    def evaluate(self, direction, degrees, drift=0.0):
        origin = (0.0, 0.0, 0.0)
        sign = 1 if direction == "LEFT" else -1
        pose = (drift, 0.0, sign * math.radians(degrees))

        gate = V6SegmentGate()
        gate.current = Segment(
            1, "TURN", origin, (0.0, 2.0),
            TURN_LIMIT, direction
        )
        gate.phase = "ACTIVE"
        gate.started_at = 10.0

        phase = gate.progress(
            pose,
            now_s=10.1,
            pose_age_s=0.0
        )

        reached = turn_reached(
            pose, origin, direction, TURN_LIMIT
        )
        return phase, reached

    def test_left_stop_alignment(self):
        self.assertEqual(
            self.evaluate("LEFT", 9.9),
            ("ACTIVE", False)
        )
        self.assertEqual(
            self.evaluate("LEFT", 10.1),
            ("STOP_REQUESTED", True)
        )

    def test_right_stop_alignment(self):
        self.assertEqual(
            self.evaluate("RIGHT", 9.9),
            ("ACTIVE", False)
        )
        self.assertEqual(
            self.evaluate("RIGHT", 10.1),
            ("STOP_REQUESTED", True)
        )

    def test_position_drift_aborts(self):
        for direction in ("LEFT", "RIGHT"):
            phase, reached = self.evaluate(
                direction, 10.1, drift=0.036
            )
            self.assertEqual(phase, "ABORT")
            self.assertFalse(reached)

    def test_angle_overshoot_aborts(self):
        for direction in ("LEFT", "RIGHT"):
            phase, reached = self.evaluate(
                direction, 14.5
            )
            self.assertEqual(phase, "ABORT")
            self.assertFalse(reached)
