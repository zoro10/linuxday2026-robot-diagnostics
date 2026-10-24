import math
import unittest

from pulse1_v6_turn_policy import (
    turn_reached,
    turn_step_count,
)
from pulse1_v6_single_segment import (
    segment_reached,
    segment_steps,
)
from pulse1_v6_segment_fsm import MIN_PROGRESS_RAD


class TestMicroTurnSharedReached(unittest.TestCase):

    def setUp(self):
        self.origin = (2.0884, 0.0, 1.535610)
        self.limit = 0.032276
        self.request = {
            "kind": "TURN",
            "direction": "LEFT",
            "limit": self.limit,
        }

    def pose(self, deg, direction="LEFT"):
        sign = 1 if direction == "LEFT" else -1
        return (
            self.origin[0],
            self.origin[1],
            self.origin[2]
            + sign * math.radians(deg),
        )

    def test_fsm_minimum_unchanged(self):
        self.assertAlmostEqual(
            math.degrees(MIN_PROGRESS_RAD),
            0.5,
            places=6
        )

    def test_no_premature_completion(self):
        for deg in (0, 0.1, 0.157563, 0.3, 0.49):
            with self.subTest(deg=deg):
                pose = self.pose(deg)

                self.assertFalse(
                    turn_reached(
                        pose, self.origin,
                        "LEFT", self.limit
                    )
                )
                self.assertFalse(
                    segment_reached(
                        pose, self.origin, self.request
                    )
                )

    def test_steps_remain_available(self):
        for deg in (0, 0.157563, 0.3, 0.49):
            with self.subTest(deg=deg):
                pose = self.pose(deg)

                count = segment_steps(
                    pose, self.origin,
                    .001, self.request
                )
                self.assertIs(type(count), int)
                self.assertGreaterEqual(count, 1)
                self.assertLessEqual(count, 40)

    def test_completion_after_minimum(self):
        for deg in (0.51, 0.6, 1.0, 1.5):
            with self.subTest(deg=deg):
                pose = self.pose(deg)

                self.assertTrue(
                    turn_reached(
                        pose, self.origin,
                        "LEFT", self.limit
                    )
                )
                self.assertTrue(
                    segment_reached(
                        pose, self.origin, self.request
                    )
                )

    def test_right_turn_is_symmetric(self):
        pose = self.pose(0.6, "RIGHT")
        self.assertTrue(
            turn_reached(
                pose, self.origin,
                "RIGHT", self.limit
            )
        )
        self.assertFalse(
            turn_reached(
                self.pose(0.1, "RIGHT"),
                self.origin, "RIGHT", self.limit
            )
        )

    def test_normal_turn_keeps_original_threshold(self):
        limit = math.radians(12)

        self.assertFalse(
            turn_reached(
                self.pose(9.5), self.origin,
                "LEFT", limit
            )
        )
        self.assertTrue(
            turn_reached(
                self.pose(10.5), self.origin,
                "LEFT", limit
            )
        )


if __name__ == "__main__":
    unittest.main()
