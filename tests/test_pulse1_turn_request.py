import math
import unittest

from pulse1_motor_guard_logic import authorized_request
from pulse1_turn_request import make_turn_request


def sensors(**changes):
    data = {
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
    }
    data.update(changes)
    return data


def turning(angle=0.10):
    return {
        "state": "TURNING",
        "linear_m_s": 0.0,
        "angular_rad_s": angle,
    }


class TurnRequestTests(unittest.TestCase):

    def test_left_turn(self):
        r = make_turn_request(turning(), sensors(), armed=True)
        self.assertEqual(r["state"], "AVOIDING")
        self.assertAlmostEqual(r["proposed_angular_rad_s"], 0.10)
        self.assertTrue(authorized_request(r, "AVOID"))

    def test_right_turn(self):
        r = make_turn_request(
            turning(-0.10), sensors(), armed=True
        )
        self.assertAlmostEqual(r["proposed_angular_rad_s"], -0.10)

    def test_disarmed(self):
        r = make_turn_request(turning(), sensors())
        self.assertEqual(r["state"], "HOLD")
        self.assertEqual(r["proposed_angular_rad_s"], 0.0)
        self.assertFalse(authorized_request(r, "AVOID"))

    def test_done_stops(self):
        d = turning()
        d["state"] = "DONE"
        r = make_turn_request(d, sensors(), armed=True)
        self.assertEqual(r["proposed_angular_rad_s"], 0.0)

    def test_abort_stops(self):
        d = turning()
        d["state"] = "ABORT"
        r = make_turn_request(d, sensors(), armed=True)
        self.assertEqual(r["state"], "HOLD")

    def test_missing_imu(self):
        r = make_turn_request(
            turning(), sensors(imu_ok=False), armed=True
        )
        self.assertEqual(r["proposed_angular_rad_s"], 0.0)

    def test_missing_rear_lidar(self):
        r = make_turn_request(
            turning(), sensors(rear_lidar_ok=False),
            armed=True,
        )
        self.assertEqual(r["state"], "HOLD")

    def test_excessive_angular_speed(self):
        r = make_turn_request(
            turning(0.35), sensors(), armed=True
        )
        self.assertEqual(r["proposed_angular_rad_s"], 0.0)

    def test_invalid_angular_speed(self):
        r = make_turn_request(
            turning(math.nan), sensors(), armed=True
        )
        self.assertEqual(r["state"], "HOLD")

    def test_linear_motion_denied(self):
        d = turning()
        d["linear_m_s"] = 0.18
        r = make_turn_request(d, sensors(), armed=True)
        self.assertEqual(r["proposed_linear_m_s"], 0.0)
        self.assertEqual(r["proposed_angular_rad_s"], 0.0)


if __name__ == "__main__":
    unittest.main()
