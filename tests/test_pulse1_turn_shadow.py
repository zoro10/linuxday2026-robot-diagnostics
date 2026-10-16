import unittest
from pathlib import Path

from pulse1_turn_shadow import (
    observer_ready,
    clearance_from_observer,
)


def example(distance=4.5):
    sector = {"clear": False, "distance_m": distance}
    readings = {}

    for sensor in ("front", "rear"):
        readings[sensor] = {
            "sectors": {
                name: dict(sector)
                for name in ("front", "left", "right")
            }
        }

    return {
        "source": "avoid_observer",
        "mode": "AVOID",
        "state": "SENSOR_READY",
        "motor_commands": False,
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
        "readings": readings,
    }


class TurnShadowTests(unittest.TestCase):

    def test_ready(self):
        self.assertTrue(observer_ready(example(), 0.2))

    def test_stale_observer(self):
        self.assertFalse(observer_ready(example(), 2.0))

    def test_missing_rear_lidar(self):
        data = example()
        data["rear_lidar_ok"] = False
        self.assertFalse(observer_ready(data, 0.1))

    def test_clearance(self):
        self.assertAlmostEqual(
            clearance_from_observer(example()), 3.4
        )

    def test_missing_sector(self):
        data = example()
        del data["readings"]["rear"]["sectors"]["left"]
        self.assertIsNone(clearance_from_observer(data))

    def test_no_motor_topics(self):
        source = (
            Path(__file__).resolve().parent.parent
            / "scripts/pulse1_turn_shadow.py"
        ).read_text()
        self.assertNotIn('"/pulse1/avoid_request"', source)
        self.assertNotIn('"/model/pulse1_proto/cmd_vel"', source)


if __name__ == "__main__":
    unittest.main()
