import math
import unittest

from pulse1_s1_approach_logic import approach_step
from pulse1_s1_approach_request import (
    make_approach_request,
    MAX_TEST_X_M,
)
from pulse1_motor_guard_logic import (
    authorized_request,
    guarded_motion,
)


def decision():
    return approach_step(
        front_distance_m=4.202,
        pose=(0.0, 0.0, 0.0),
        sensors_ok=True,
        observer_age_s=0.1,
    )


def sensors():
    return {
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
    }


def request(
    *,
    armed=True,
    pose=(0.0, 0.0, 0.0),
    dec=None,
    flags=None,
):
    return make_approach_request(
        decision() if dec is None else dec,
        sensors() if flags is None else flags,
        pose,
        armed=armed,
    )


class ApproachRequestTests(unittest.TestCase):

    def test_disarmed_never_moves(self):
        r = request(armed=False)
        self.assertEqual(r["state"], "HOLD")
        self.assertEqual(r["proposed_linear_m_s"], 0)
        self.assertFalse(
            authorized_request(r, "AVOID")
        )

    def test_valid_armed_request(self):
        r = request()
        self.assertEqual(r["state"], "GO_TO_GOAL")
        self.assertEqual(r["proposed_linear_m_s"], 0.10)
        self.assertEqual(r["proposed_angular_rad_s"], 0)
        self.assertTrue(
            authorized_request(r, "AVOID")
        )

    def test_motor_guard_accepts_safe_speed(self):
        r = request()
        linear, angular = guarded_motion(
            r["proposed_linear_m_s"],
            r["proposed_angular_rad_s"],
            0.1,
            active=authorized_request(r, "AVOID"),
        )
        self.assertEqual((linear, angular), (0.1, 0.0))

    def test_position_limit(self):
        for x in (MAX_TEST_X_M, 0.30, -0.10):
            with self.subTest(x=x):
                r = request(pose=(x, 0.0, 0.0))
                self.assertEqual(r["state"], "HOLD")
                self.assertEqual(
                    r["proposed_linear_m_s"], 0
                )

    def test_lateral_limit(self):
        r = request(pose=(0.1, 0.11, 0.0))
        self.assertEqual(r["state"], "HOLD")

    def test_heading_limit(self):
        r = request(pose=(0.1, 0.0, 0.09))
        self.assertEqual(r["state"], "HOLD")

    def test_invalid_position(self):
        for pose in (
            None,
            (0.0, 0.0),
            (math.nan, 0.0, 0.0),
            (math.inf, 0.0, 0.0),
        ):
            with self.subTest(pose=pose):
                self.assertEqual(
                    request(pose=pose)["state"],
                    "HOLD",
                )

    def test_sensor_failure(self):
        for flag in sensors():
            with self.subTest(flag=flag):
                flags = sensors()
                flags[flag] = False
                r = request(flags=flags)
                self.assertEqual(r["state"], "HOLD")
                self.assertFalse(
                    authorized_request(r, "AVOID")
                )

    def test_stop_decision_denied(self):
        dec = approach_step(
            front_distance_m=1.60,
            pose=(0.0, 0.0, 0.0),
            sensors_ok=True,
            observer_age_s=0.1,
        )
        self.assertEqual(dec["state"], "STOP")
        self.assertEqual(
            request(dec=dec)["state"], "HOLD"
        )

    def test_excess_speed_denied(self):
        dec = decision()
        dec["shadow_linear_m_s"] = 0.11
        self.assertEqual(
            request(dec=dec)["state"], "HOLD"
        )

    def test_invalid_speed_denied(self):
        for value in (
            0.0, -0.1, math.nan, math.inf, True
        ):
            with self.subTest(value=value):
                dec = decision()
                dec["shadow_linear_m_s"] = value
                self.assertEqual(
                    request(dec=dec)["state"],
                    "HOLD",
                )

    def test_motor_contract_tampering_denied(self):
        for field, value in (
            ("motor_commands", True),
            ("proposed_linear_m_s", 0.1),
            ("proposed_angular_rad_s", 0.1),
        ):
            with self.subTest(field=field):
                dec = decision()
                dec[field] = value
                self.assertEqual(
                    request(dec=dec)["state"],
                    "HOLD",
                )

    def test_never_direct_motor_commands(self):
        for r in (request(), request(armed=False)):
            self.assertIs(r["motor_commands"], False)
            self.assertEqual(
                r["proposed_angular_rad_s"], 0
            )


if __name__ == "__main__":
    unittest.main()
