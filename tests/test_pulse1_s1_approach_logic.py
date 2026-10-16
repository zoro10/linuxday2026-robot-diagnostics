import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from pulse1_s1_approach_logic import (
    approach_step,
    SHADOW_SPEED_M_S,
    LIDAR_TO_FRONT_BUMPER_M,
)

ROOT = Path(__file__).resolve().parents[1]


def decide(**changes):
    kwargs = {
        "front_distance_m": 4.202,
        "pose": (0.0, 0.0, 0.0),
        "sensors_ok": True,
        "observer_age_s": 0.2,
    }
    kwargs.update(changes)
    return approach_step(**kwargs)


class ApproachTests(unittest.TestCase):

    def test_initial_approach(self):
        r = decide()
        self.assertEqual(r["state"], "APPROACH")
        self.assertEqual(
            r["shadow_linear_m_s"],
            SHADOW_SPEED_M_S,
        )

    def test_stop_at_front_threshold(self):
        r = decide(front_distance_m=1.60)
        self.assertEqual(r["state"], "STOP")
        self.assertEqual(
            r["reason"], "FRONT_DISTANCE_LIMIT"
        )

    def test_stop_at_position_limit(self):
        r = decide(pose=(2.60, 0.0, 0.0))
        self.assertEqual(r["state"], "STOP")

    def test_missing_sensor(self):
        self.assertEqual(
            decide(sensors_ok=False)["state"],
            "HOLD",
        )

    def test_stale_observer(self):
        self.assertEqual(
            decide(observer_age_s=1.8)["state"],
            "HOLD",
        )

    def test_invalid_distance(self):
        for distance in (None, -1, float("inf")):
            with self.subTest(distance=distance):
                self.assertEqual(
                    decide(
                        front_distance_m=distance
                    )["state"],
                    "HOLD",
                )

    def test_lateral_error(self):
        self.assertEqual(
            decide(pose=(1, 0.20, 0))["state"],
            "HOLD",
        )

    def test_heading_error(self):
        self.assertEqual(
            decide(pose=(1, 0, 0.20))["state"],
            "HOLD",
        )

    def test_never_commands_motors(self):
        cases = (
            decide(),
            decide(front_distance_m=1.0),
            decide(sensors_ok=False),
        )
        for result in cases:
            self.assertIs(
                result["motor_commands"], False
            )
            self.assertEqual(
                result["proposed_linear_m_s"], 0
            )
            self.assertEqual(
                result["proposed_angular_rad_s"], 0
            )

    def test_geometry_matches_sdf(self):
        sdf = ET.parse(
            ROOT / "simulation/models/pulse1/model_demo2.sdf"
        ).getroot()

        chassis = sdf.find(".//link[@name='chassis']")
        chassis_x = float(
            chassis.findtext("pose").split()[0]
        )
        length = float(
            chassis.findtext(
                "./collision/geometry/box/size"
            ).split()[0]
        )
        lidar_x = float(
            chassis.find(
                "./sensor[@name='pulse1_lidar']/pose"
            ).text.split()[0]
        )

        nose_x = chassis_x + length / 2
        sensor_x = chassis_x + lidar_x

        self.assertAlmostEqual(
            nose_x - sensor_x,
            LIDAR_TO_FRONT_BUMPER_M,
            places=4,
        )

        # Ostacolo: prima faccia a X=4.5.
        expected_scan = 4.5 - sensor_x

        self.assertAlmostEqual(
            expected_scan, 4.202, places=2
        )

    def test_stopping_margin(self):
        remaining_gap = (
            1.60 - LIDAR_TO_FRONT_BUMPER_M
        )
        self.assertGreater(remaining_gap, 1.0)


if __name__ == "__main__":
    unittest.main()
