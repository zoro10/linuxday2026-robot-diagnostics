import math
import unittest

from pulse1_v6_sequencer import (
    WORLD, load_world, v6_corner_planning_pose
)
from pulse1_v6_route_policy import (
    next_route_request, build_route
)


class TestV6SelectedAlong(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.world = load_world(WORLD)
        cls.route = build_route(cls.world, "LEFT")
        cls.raw = (2.609480, -0.553989, 1.535610)
        cls.pose = v6_corner_planning_pose(
            cls.raw, cls.world, "LEFT"
        )

    def test_micro_turn_at_real_corner(self):
        request = next_route_request(
            self.world, self.pose,
            "v6-selected-leg-test", "LEFT"
        )
        self.assertEqual(request["kind"], "TURN")
        self.assertEqual(request["direction"], "LEFT")
        self.assertEqual(request["waypoint_index"], 1)
        self.assertAlmostEqual(
            math.degrees(request["limit"]),
            2.016028,
            delta=.03
        )

    def test_micro_turn_only_near_start(self):
        x, y, yaw = self.raw
        raw = (
            x+.10*math.cos(yaw),
            y+.10*math.sin(yaw),
            yaw
        )
        corrected = v6_corner_planning_pose(
            raw, self.world, "LEFT"
        )
        request = next_route_request(
            self.world, corrected,
            "v6-no-late-microturn", "LEFT"
        )
        self.assertEqual(request["kind"], "ADVANCE")
        self.assertEqual(request["waypoint_index"], 1)

    def test_aligned_path_can_advance(self):
        reference = self.pose
        for d in (.10, 1., 2., 2.75):
            with self.subTest(distance=d):
                p = (
                    reference[0],
                    reference[1] + d,
                    math.pi/2
                )
                request = next_route_request(
                    self.world, p,
                    "v6-aligned-leg-test",
                    "LEFT"
                )
                self.assertEqual(
                    request["kind"], "ADVANCE"
                )
                self.assertEqual(
                    request["waypoint_index"], 1
                )


if __name__ == "__main__":
    unittest.main()
