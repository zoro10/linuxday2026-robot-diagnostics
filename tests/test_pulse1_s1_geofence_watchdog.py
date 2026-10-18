import unittest

from pulse1_s1_geofence_watchdog import trial_drive


def valid():
    return {
        "source": "avoid_navigator",
        "mode": "AVOID",
        "state": "GO_TO_GOAL",
        "trial_phase": "DRIVE",
        "trial_reason": "LIMITED_FORWARD_TRIAL",
        "motor_commands": False,
        "proposed_linear_m_s": 0.10,
        "proposed_angular_rad_s": 0.0,
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
    }


class WatchdogRequestTests(unittest.TestCase):

    def test_valid_trial(self):
        self.assertTrue(trial_drive(valid(), 0.1))

    def test_disarmed_hold(self):
        data = valid()
        data["state"] = "HOLD"
        self.assertFalse(trial_drive(data, 0.1))

    def test_stale_request(self):
        self.assertFalse(trial_drive(valid(), 0.8))

    def test_other_source(self):
        data = valid()
        data["source"] = "turn_shadow"
        self.assertFalse(trial_drive(data, 0.1))

    def test_excess_speed(self):
        data = valid()
        data["proposed_linear_m_s"] = 0.11
        self.assertFalse(trial_drive(data, 0.1))

    def test_missing_sensor(self):
        for flag in (
            "odom_ok", "lidar_ok",
            "rear_lidar_ok", "imu_ok"
        ):
            with self.subTest(flag=flag):
                data = valid()
                data[flag] = False
                self.assertFalse(trial_drive(data, 0.1))


if __name__ == "__main__":
    unittest.main()
