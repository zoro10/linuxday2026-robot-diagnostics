import math
import unittest

from pulse1_navigation_fsm import MissionFSM
from pulse1_scan_logic import front_distance


class NavigationTests(unittest.TestCase):

    def test_goal_fisso(self):
        nav = MissionFSM("AVOID", 0, 0, 0)
        self.assertEqual((nav.goal_x, nav.goal_y), (10, 0))

        nav.update(2, 0, math.pi / 2, 4.0)
        self.assertEqual((nav.goal_x, nav.goal_y), (10, 0))

    def test_modalita_non_valida(self):
        with self.assertRaises(ValueError):
            MissionFSM("ROCKET", 0, 0, 0)

    def test_stop_avanza(self):
        nav = MissionFSM("STOP", 0, 0, 0)
        self.assertEqual(
            nav.update(0, 0, 0, 3.0).state, "DRIVE"
        )

    def test_stop_ostacolo(self):
        nav = MissionFSM("STOP", 0, 0, 0)
        self.assertEqual(
            nav.update(2, 0, 0, 1.15).state, "STOPPED"
        )

    def test_stop_soglia(self):
        nav = MissionFSM("STOP", 0, 0, 0)
        self.assertEqual(
            nav.update(2, 0, 0, 1.20).state, "STOPPED"
        )

    def test_avoid_avanza(self):
        nav = MissionFSM("AVOID", 0, 0, 0)
        self.assertEqual(
            nav.update(0, 0, 0, 3.0).state, "DRIVE"
        )

    def test_avoid_valuta(self):
        nav = MissionFSM("AVOID", 0, 0, 0)
        self.assertEqual(
            nav.update(2, 0, 0, 1.55).state, "ASSESS"
        )

    def test_avoid_troppo_vicino(self):
        nav = MissionFSM("AVOID", 0, 0, 0)
        self.assertEqual(
            nav.update(3, 0, 0, 1.10).state, "SAFE_STOP"
        )

    def test_no_lidar(self):
        nav = MissionFSM("AVOID", 0, 0, 0)
        self.assertEqual(
            nav.update(
                0, 0, 0, 5.0, lidar_ok=False
            ).state,
            "SAFE_STOP"
        )

    def test_no_odometry(self):
        nav = MissionFSM("STOP", 0, 0, 0)
        self.assertEqual(
            nav.update(
                0, 0, 0, 5.0, odom_ok=False
            ).state,
            "SAFE_STOP"
        )

    def test_no_measurement(self):
        for reading in (None, float("nan"), float("-inf"), 0.0):
            nav = MissionFSM("AVOID", 0, 0, 0)
            self.assertEqual(
                nav.update(0, 0, 0, reading).state,
                "SAFE_STOP"
            )


    def test_fsm_front_free(self):
        for mode in ("STOP", "AVOID"):
            nav = MissionFSM(mode, 0, 0, 0)
            self.assertEqual(
                nav.update(0, 0, 0, math.inf).state,
                "DRIVE"
            )

    def test_scan_clear(self):
        ranges = [math.inf] * 180
        result = front_distance(
            ranges, -math.pi / 2, math.pi / 179,
            0.12, 12.0
        )
        self.assertEqual(result, math.inf)

    def test_scan_obstacle(self):
        ranges = [math.inf] * 180
        ranges[89] = 1.17

        result = front_distance(
            ranges, -math.pi / 2, math.pi / 179,
            0.12, 12.0
        )
        self.assertAlmostEqual(result, 1.17)

    def test_scan_invalid(self):
        ranges = [math.inf] * 180
        ranges[89] = math.nan

        result = front_distance(
            ranges, -math.pi / 2, math.pi / 179,
            0.12, 12.0
        )
        self.assertIsNone(result)

    def test_scan_missing_front(self):
        result = front_distance(
            [math.inf] * 10,
            0.8, 0.05, 0.12, 12.0
        )
        self.assertIsNone(result)

    def test_arrived(self):
        nav = MissionFSM("AVOID", 0, 0, 0)
        result = nav.update(9.7, 0, 0, 5.0)
        self.assertEqual(result.state, "ARRIVED")

        # Non riavvia il movimento dopo il completamento.
        self.assertEqual(
            nav.update(8, 0, 0, 5.0).state,
            "ARRIVED"
        )


if __name__ == "__main__":
    unittest.main()
