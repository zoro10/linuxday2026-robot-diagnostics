import math
import unittest

from pulse1_v6_sequencer import (
    WORLD,
    load_world,
    v6_corner_planning_pose,
)
from pulse1_v6_route_policy import (
    build_route,
    next_route_request,
)


class TestV6WP1FrameTransition(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.world = load_world(WORLD)
        route = build_route(cls.world, "LEFT")
        cls.wp0 = route["waypoints"][0]
        cls.wp1 = route["waypoints"][1]

        # Stato Gazebo validato nel Task 93.
        cls.x = 2.609480
        cls.y = -0.553989
        cls.yaw = 1.535610

    def predicted(self, distance, lateral=0.0):
        return (
            self.x
            + distance * math.cos(self.yaw)
            + lateral,
            self.y
            + distance * math.sin(self.yaw),
            self.yaw,
        )

    def corrected(self, pose):
        return v6_corner_planning_pose(
            pose, self.world, "LEFT"
        )

    def test_current_corner_unchanged(self):
        p = self.corrected(self.predicted(0))
        self.assertLess(
            math.hypot(
                p[0]-self.wp0[0],
                p[1]-self.wp0[1],
            ),
            .025
        )

    def test_earlier_corner_turn_still_supported(self):
        angle = math.radians(60)
        offset = .5542825
        x = 2.0747 + offset*(1-math.cos(angle))
        y = -offset*math.sin(angle)

        p = self.corrected((x, y, angle))
        self.assertAlmostEqual(p[0], 2.0747, places=3)
        self.assertAlmostEqual(p[1], 0, places=3)

    def test_wp1_forward_positions(self):
        for distance in (
            .03, .06, .10, .25,
            .50, 1.00
        ):
            with self.subTest(distance=distance):
                p = self.corrected(
                    self.predicted(distance)
                )
                self.assertAlmostEqual(
                    p[1], distance, delta=.002
                )

                request = next_route_request(
                    self.world,
                    p,
                    "v6-wp1-frame-test",
                    "LEFT",
                )
                self.assertEqual(
                    request["kind"], "ADVANCE"
                )
                self.assertEqual(
                    request["waypoint_index"], 1
                )

    def test_long_distance_drift_rejected(self):
        # 87.984 gradi causano deriva laterale.
        # Il planner DEVE impedire l'uscita
        # dal proprio corridoio.
        for distance in (2.0, 2.75):
            with self.subTest(distance=distance):
                with self.assertRaisesRegex(
                    ValueError,
                    "OUTSIDE_ROUTE_CORRIDOR"
                ):
                    p = self.corrected(
                        self.predicted(distance)
                    )
                    next_route_request(
                        self.world,
                        p,
                        "v6-wp1-drift-test",
                        "LEFT",
                    )

    def test_aligned_90_degree_path(self):
        # Simula il riferimento Gazebo
        # dopo un allineamento a 90 gradi.
        # Non impartisce alcun comando.
        reference = self.corrected(
            self.predicted(0)
        )
        offset = .5542825

        for distance in (
            .10, .50, 1.00, 2.00, 2.75
        ):
            with self.subTest(distance=distance):
                gz_pose = (
                    reference[0] + offset,
                    reference[1] + distance - offset,
                    math.pi / 2,
                )

                p = self.corrected(gz_pose)

                request = next_route_request(
                    self.world,
                    p,
                    "v6-wp1-aligned-test",
                    "LEFT",
                )

                self.assertEqual(
                    request["kind"], "ADVANCE"
                )
                self.assertEqual(
                    request["waypoint_index"], 1
                )

    def test_lateral_escape_rejected(self):
        with self.assertRaises(ValueError):
            self.corrected(
                self.predicted(.10, lateral=.25)
            )

    def test_beyond_wp1_rejected(self):
        with self.assertRaises(ValueError):
            self.corrected(
                self.predicted(2.95)
            )

    def test_behind_wp0_rejected(self):
        with self.assertRaises(ValueError):
            self.corrected(
                self.predicted(-.15)
            )

    def test_heading_outside_calibration_rejected(self):
        x, y, _ = self.predicted(.10)
        with self.assertRaises(ValueError):
            self.corrected(
                (x, y, math.radians(94))
            )


if __name__ == "__main__":
    unittest.main()
