import math
import unittest

from pulse1_turn_logic import turn_step, TARGET_RAD


def step(**changes):
    data = {
        "start_pose": (0.0, 0.0, 0.0),
        "pose": (0.0, 0.0, 0.0),
        "integrated_gyro_rad": 0.0,
        "gyro_z": 0.0,
        "elapsed_s": 0.0,
        "sensors_ok": True,
        "clearance_m": 4.0,
        "direction": "LEFT",
    }
    data.update(changes)
    return turn_step(**data)


class TurnLogicTests(unittest.TestCase):

    def test_initial_left_rotation(self):
        r = step()
        self.assertEqual(r["state"], "TURNING")
        self.assertAlmostEqual(r["angular_rad_s"], 0.10)
        self.assertEqual(r["linear_m_s"], 0.0)

    def test_initial_right_rotation(self):
        r = step(direction="RIGHT")
        self.assertEqual(r["state"], "TURNING")
        self.assertAlmostEqual(r["angular_rad_s"], -0.10)

    def test_target_reached(self):
        r = step(
            pose=(0, 0, TARGET_RAD),
            integrated_gyro_rad=TARGET_RAD,
            gyro_z=0.05,
            elapsed_s=3.0,
        )
        self.assertEqual(r["state"], "DONE")
        self.assertEqual(r["angular_rad_s"], 0.0)

    def test_slowdown_near_target(self):
        r = step(
            pose=(0, 0, 0.20),
            integrated_gyro_rad=0.20,
            gyro_z=0.05,
            elapsed_s=2.0,
        )
        self.assertEqual(r["state"], "TURNING")
        self.assertAlmostEqual(r["angular_rad_s"], 0.05)

    def test_missing_sensor(self):
        r = step(sensors_ok=False)
        self.assertEqual(r["state"], "ABORT")
        self.assertEqual(r["angular_rad_s"], 0.0)

    def test_too_close_to_obstacle(self):
        r = step(clearance_m=1.4)
        self.assertEqual(
            r["reason"], "INSUFFICIENT_CLEARANCE"
        )

    def test_time_limit(self):
        r = step(elapsed_s=6.1)
        self.assertEqual(r["state"], "ABORT")

    def test_imu_odometry_disagreement(self):
        r = step(
            pose=(0, 0, 0.02),
            integrated_gyro_rad=0.25,
            elapsed_s=3.0,
        )
        self.assertEqual(
            r["reason"], "ODOM_IMU_DISAGREE"
        )

    def test_wrong_direction(self):
        r = step(
            pose=(0, 0, -0.08),
            integrated_gyro_rad=-0.08,
            elapsed_s=1.0,
        )
        self.assertEqual(
            r["reason"], "WRONG_TURN_DIRECTION"
        )

    def test_unexpected_translation(self):
        r = step(pose=(0.30, 0.0, 0.0))
        self.assertEqual(
            r["reason"], "UNEXPECTED_TRANSLATION"
        )

    def test_invalid_measurement(self):
        r = step(gyro_z=math.nan)
        self.assertEqual(
            r["reason"], "INVALID_MEASUREMENT"
        )

    def test_wrapped_angle(self):
        initial = math.pi - 0.10
        current = -math.pi + 0.12
        r = step(
            start_pose=(0, 0, initial),
            pose=(0, 0, current),
            integrated_gyro_rad=0.22,
            gyro_z=0.05,
            elapsed_s=2.5,
        )
        self.assertEqual(r["state"], "TURNING")
        self.assertAlmostEqual(r["angular_rad_s"], 0.05)


if __name__ == "__main__":
    unittest.main()
