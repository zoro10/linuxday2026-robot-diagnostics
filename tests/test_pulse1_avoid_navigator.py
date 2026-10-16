import unittest

from pulse1_avoid_navigator import hold_request
from pulse1_motor_guard_logic import authorized_request


def ready_observer():
    return {
        "source": "avoid_observer",
        "mode": "AVOID",
        "state": "SENSOR_READY",
        "motor_commands": False,
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
    }


class AvoidNavigatorTests(unittest.TestCase):

    def test_all_sensors_ready_still_holds(self):
        result = hold_request(ready_observer(), 0.2)
        self.assertEqual(result["state"], "HOLD")
        self.assertEqual(result["reason"], "ZERO_MOTION_INTEGRATION")
        self.assertTrue(result["imu_ok"])
        self.assertEqual(result["proposed_linear_m_s"], 0)
        self.assertEqual(result["proposed_angular_rad_s"], 0)
        self.assertFalse(authorized_request(result, "AVOID"))

    def test_missing_imu(self):
        observer = ready_observer()
        observer["imu_ok"] = False
        result = hold_request(observer, 0.2)
        self.assertFalse(result["imu_ok"])
        self.assertEqual(result["reason"], "WAIT_SENSORS")

    def test_observer_expired(self):
        result = hold_request(ready_observer(), 3.0)
        self.assertFalse(result["odom_ok"])
        self.assertEqual(result["state"], "HOLD")

    def test_wrong_source(self):
        observer = ready_observer()
        observer["source"] = "unknown"
        result = hold_request(observer, 0.1)
        self.assertFalse(result["lidar_ok"])

    def test_invalid_age(self):
        result = hold_request(ready_observer(), None)
        self.assertEqual(result["state"], "HOLD")
        self.assertFalse(result["rear_lidar_ok"])

    def test_missing_observer(self):
        result = hold_request(None, None)
        self.assertEqual(result["state"], "HOLD")
        self.assertFalse(authorized_request(result, "AVOID"))


if __name__ == "__main__":
    unittest.main()
