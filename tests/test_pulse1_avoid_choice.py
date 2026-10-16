import math
import unittest

from pulse1_avoid_choice import choose_avoid_side


class AvoidChoiceTests(unittest.TestCase):

    def test_obstacle_far(self):
        self.assertEqual(
            choose_avoid_side({
                "front": 4.202,
                "left": math.inf,
                "right": math.inf,
            }),
            "OBSERVE",
        )

    def test_choose_left(self):
        self.assertEqual(
            choose_avoid_side({
                "front": 2.3,
                "left": math.inf,
                "right": 0.8,
            }),
            "CANDIDATE_LEFT",
        )

    def test_choose_right(self):
        self.assertEqual(
            choose_avoid_side({
                "front": 2.3,
                "left": 0.8,
                "right": math.inf,
            }),
            "CANDIDATE_RIGHT",
        )

    def test_both_sides_blocked(self):
        self.assertEqual(
            choose_avoid_side({
                "front": 2.3,
                "left": 0.9,
                "right": 1.1,
            }),
            "HOLD_NO_SIDE",
        )

    def test_too_close(self):
        self.assertEqual(
            choose_avoid_side({
                "front": 1.18,
                "left": math.inf,
                "right": math.inf,
            }),
            "HOLD_TOO_CLOSE",
        )

    def test_missing_measurement(self):
        self.assertEqual(
            choose_avoid_side({
                "front": 2.3,
                "left": None,
                "right": math.inf,
            }),
            "WAIT_DATA",
        )

    def test_nan_measurement(self):
        self.assertEqual(
            choose_avoid_side({
                "front": 2.3,
                "left": math.nan,
                "right": math.inf,
            }),
            "WAIT_DATA",
        )


if __name__ == "__main__":
    unittest.main()
