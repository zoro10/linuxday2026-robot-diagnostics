import unittest

from pulse1_v6_sequencer import (
    WORLD,
    load_world,
    precision_wp0_required,
)
from pulse1_v6_route_policy import (
    build_route,
    next_route_request,
)
from pulse1_v6_single_segment import (
    precision_waypoint0_request,
)


class TestV6WP0PrecisionSetback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.world = load_world(WORLD)
        cls.x0 = build_route(
            cls.world, "LEFT"
        )["waypoints"][0][0]
        cls.sid = "v6-test-wp0-setback"

    def precision(self, gz, odom):
        return precision_waypoint0_request(
            self.world, gz, odom,
            self.sid, "LEFT"
        )

    def test_new_waypoint_coordinate(self):
        self.assertAlmostEqual(
            self.x0, 2.098566677973659,
            places=5
        )

    def test_precision_accepts_new_corner(self):
        gz = (self.x0 - .026, 0., 0.)
        odom = (self.x0 - .010, 0., 0.)

        request = next_route_request(
            self.world, gz, self.sid, "LEFT"
        )

        self.assertTrue(
            precision_wp0_required(request, gz)
        )

        corrected = self.precision(gz, odom)

        self.assertEqual(
            corrected["kind"], "ADVANCE"
        )
        self.assertEqual(
            corrected["waypoint_index"], 0
        )
        self.assertEqual(
            corrected["start_pose"], list(odom)
        )

    def test_far_before_corner_rejected(self):
        with self.assertRaises(ValueError):
            self.precision(
                (self.x0 - .055, 0., 0.),
                (self.x0 - .039, 0., 0.)
            )

    def test_already_at_corner_rejected(self):
        with self.assertRaises(ValueError):
            self.precision(
                (self.x0 - .005, 0., 0.),
                (self.x0 + .005, 0., 0.)
            )

    def test_pose_disagreement_rejected(self):
        with self.assertRaises(ValueError):
            self.precision(
                (self.x0 - .026, 0., 0.),
                (self.x0 + .030, 0., 0.)
            )

    def test_odom_not_at_turn_rejected(self):
        with self.assertRaises(ValueError):
            self.precision(
                (self.x0 - .026, 0., 0.),
                (self.x0 - .026, 0., 0.)
            )

    def test_old_corner_coordinates_rejected(self):
        with self.assertRaises(ValueError):
            self.precision(
                (2.3384, 0., 0.),
                (2.3544, 0., 0.)
            )


if __name__ == "__main__":
    unittest.main()
