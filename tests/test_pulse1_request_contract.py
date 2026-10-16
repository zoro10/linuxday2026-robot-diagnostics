import unittest

from pulse1_motor_guard_logic import authorized_request


STOP = {
    "source": "stop_dryrun",
    "mode": "STOP",
    "state": "DRIVE",
    "odom_ok": True,
    "lidar_ok": True,
    "motor_commands": False,
}

AVOID = {
    "source": "avoid_navigator",
    "mode": "AVOID",
    "state": "AVOIDING",
    "odom_ok": True,
    "lidar_ok": True,
    "rear_lidar_ok": True,
    "imu_ok": True,
    "motor_commands": False,
}


class RequestContractTests(unittest.TestCase):

    def test_stop_authorized(self):
        self.assertTrue(authorized_request(STOP, "STOP"))

    def test_avoid_authorized(self):
        self.assertTrue(authorized_request(AVOID, "AVOID"))

    def test_avoid_denied_in_stop_mode(self):
        self.assertFalse(authorized_request(AVOID, "STOP"))

    def test_stop_denied_in_avoid_mode(self):
        self.assertFalse(authorized_request(STOP, "AVOID"))

    def test_avoid_without_imu(self):
        request = dict(AVOID, imu_ok=False)
        self.assertFalse(authorized_request(request, "AVOID"))

    def test_avoid_without_rear_lidar(self):
        request = dict(AVOID, rear_lidar_ok=False)
        self.assertFalse(authorized_request(request, "AVOID"))

    def test_avoid_without_front_lidar(self):
        request = dict(AVOID, lidar_ok=False)
        self.assertFalse(authorized_request(request, "AVOID"))

    def test_avoid_without_odometry(self):
        request = dict(AVOID, odom_ok=False)
        self.assertFalse(authorized_request(request, "AVOID"))

    def test_avoid_unknown_state(self):
        request = dict(AVOID, state="UNKNOWN")
        self.assertFalse(authorized_request(request, "AVOID"))

    def test_avoid_wrong_source(self):
        request = dict(AVOID, source="stop_dryrun")
        self.assertFalse(authorized_request(request, "AVOID"))

    def test_unselected_mode(self):
        self.assertFalse(authorized_request(AVOID, "UNKNOWN"))

    def test_invalid_message(self):
        self.assertFalse(authorized_request(None, "AVOID"))


if __name__ == "__main__":
    unittest.main()
