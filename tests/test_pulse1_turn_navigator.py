import json
import time
import unittest
from types import SimpleNamespace

from pulse1_turn_logic import TARGET_RAD
from pulse1_turn_navigator import TurnNavigator


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


def fake_navigator(armed=False, x=0.0):
    now = time.monotonic()
    sector = {"clear": False, "distance_m": 4.5}

    observer = {
        "source": "avoid_observer",
        "mode": "AVOID",
        "state": "SENSOR_READY",
        "motor_commands": False,
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
        "readings": {
            name: {
                "sectors": {
                    k: dict(sector)
                    for k in ("front", "left", "right")
                }
            }
            for name in ("front", "rear")
        },
    }

    return SimpleNamespace(
        armed=armed,
        observer=observer,
        observer_at=now,
        pose=(x, 0.0, 0.0),
        pose_at=now,
        imu_at=now,
        gyro_z=0.0,
        gyro_integral=0.0,
        start_pose=None,
        started_at=None,
        finished=None,
        output=Capture(),
    )


def control(node):
    TurnNavigator.control(node)
    return node.output.messages[-1]


class TurnNavigatorTests(unittest.TestCase):

    def test_disarmed_holds(self):
        result = control(fake_navigator())
        self.assertEqual(result["state"], "HOLD")
        self.assertEqual(result["proposed_angular_rad_s"], 0.0)

    def test_armed_initial_turn(self):
        result = control(fake_navigator(armed=True))
        self.assertEqual(result["state"], "AVOIDING")
        self.assertAlmostEqual(
            result["proposed_angular_rad_s"], 0.10
        )
        self.assertEqual(result["proposed_linear_m_s"], 0.0)

    def test_unsafe_start_pose(self):
        result = control(fake_navigator(armed=True, x=3.056))
        self.assertEqual(result["state"], "HOLD")
        self.assertEqual(
            result["turn_reason"], "UNSAFE_START_POSE"
        )

    def test_missing_sensor(self):
        node = fake_navigator(armed=True)
        node.observer["imu_ok"] = False
        result = control(node)
        self.assertEqual(result["state"], "HOLD")
        self.assertEqual(result["proposed_angular_rad_s"], 0.0)

    def test_target_reached_stops(self):
        node = fake_navigator(armed=True)
        node.started_at = time.monotonic() - 3.0
        node.start_pose = (0.0, 0.0, 0.0)
        node.pose = (0.0, 0.0, TARGET_RAD)
        node.gyro_integral = TARGET_RAD
        node.gyro_z = 0.05

        result = control(node)
        self.assertEqual(result["state"], "HOLD")
        self.assertEqual(result["turn_state"], "DONE")
        self.assertEqual(result["proposed_angular_rad_s"], 0.0)

    def test_timeout_stops(self):
        node = fake_navigator(armed=True)
        node.started_at = time.monotonic() - 7.0
        node.start_pose = (0.0, 0.0, 0.0)

        result = control(node)
        self.assertEqual(result["state"], "HOLD")
        self.assertEqual(result["turn_state"], "ABORT")
        self.assertEqual(result["proposed_angular_rad_s"], 0.0)


if __name__ == "__main__":
    unittest.main()
