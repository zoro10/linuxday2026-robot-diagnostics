import math
import unittest

from pulse1_s1_approach_logic import approach_step
from pulse1_s1_trial_gate import TrialGate
from pulse1_motor_guard_logic import authorized_request


FLAGS = (
    "odom_ok", "lidar_ok", "rear_lidar_ok", "imu_ok"
)


def tick(
    gate, *, pose=(0.0, 0.0, 0.0),
    elapsed=0.0, armed=True, distance=4.202,
    bad_sensor=None, speed=None,
):
    decision = approach_step(
        front_distance_m=distance,
        pose=pose,
        sensors_ok=True,
        observer_age_s=0.1,
    )

    if speed is not None:
        decision["shadow_linear_m_s"] = speed

    sensors = {name: True for name in FLAGS}

    if bad_sensor is not None:
        sensors[bad_sensor] = False

    return gate.step(
        decision, sensors, pose, elapsed,
        armed=armed,
    )


class TrialGateTests(unittest.TestCase):

    def test_disarmed_initially(self):
        r = tick(TrialGate(), armed=False)
        self.assertEqual(r["state"], "HOLD")
        self.assertFalse(authorized_request(r, "AVOID"))

    def test_valid_drive(self):
        r = tick(TrialGate())
        self.assertEqual(r["state"], "GO_TO_GOAL")
        self.assertEqual(r["trial_phase"], "DRIVE")
        self.assertEqual(r["proposed_linear_m_s"], 0.10)
        self.assertEqual(r["proposed_angular_rad_s"], 0)
        self.assertTrue(authorized_request(r, "AVOID"))

    def test_stop_and_permanent_latch(self):
        gate = TrialGate()
        tick(gate)
        r = tick(gate, pose=(0.20, 0, 0), elapsed=2)
        self.assertEqual(r["trial_phase"], "DONE")
        self.assertEqual(r["proposed_linear_m_s"], 0)
        self.assertFalse(authorized_request(r, "AVOID"))
        r = tick(gate, pose=(0, 0, 0), elapsed=3)
        self.assertEqual(r["state"], "HOLD")

    def test_timeout_latches(self):
        gate = TrialGate()
        tick(gate)
        r = tick(gate, elapsed=8.0)
        self.assertEqual(r["trial_reason"], "TIME_LIMIT")
        self.assertEqual(tick(gate)["state"], "HOLD")

    def test_invalid_times(self):
        for value in (-1, None, math.nan, math.inf):
            with self.subTest(value=value):
                r = tick(TrialGate(), elapsed=value)
                self.assertEqual(r["state"], "HOLD")
                self.assertEqual(r["trial_reason"], "TIME_LIMIT")

    def test_unsafe_initial_position(self):
        for pose in (
            (0.04, 0, 0),
            (0, 0.04, 0),
            (0, 0, 0.04),
        ):
            with self.subTest(pose=pose):
                r = tick(TrialGate(), pose=pose)
                self.assertEqual(
                    r["trial_reason"], "UNSAFE_START_POSE"
                )

    def test_invalid_pose(self):
        for pose in (None, (0, 0), (math.nan, 0, 0)):
            with self.subTest(pose=pose):
                r = tick(TrialGate(), pose=pose)
                self.assertEqual(r["state"], "HOLD")

    def test_arm_loss_latches(self):
        gate = TrialGate()
        tick(gate)
        r = tick(gate, armed=False, elapsed=1)
        self.assertEqual(r["trial_reason"], "ARM_LOST")
        self.assertEqual(tick(gate)["state"], "HOLD")

    def test_sensor_failure_latches(self):
        for flag in FLAGS:
            with self.subTest(flag=flag):
                gate = TrialGate()
                tick(gate)
                r = tick(gate, bad_sensor=flag, elapsed=1)
                self.assertEqual(
                    r["trial_reason"], "REQUEST_DENIED"
                )
                self.assertEqual(tick(gate)["state"], "HOLD")

    def test_obstacle_stop_denies_motion(self):
        gate = TrialGate()
        tick(gate)
        r = tick(gate, distance=1.60, elapsed=1)
        self.assertEqual(r["state"], "HOLD")
        self.assertFalse(authorized_request(r, "AVOID"))

    def test_lateral_deviation(self):
        gate = TrialGate()
        tick(gate)
        r = tick(gate, pose=(0.1, 0.11, 0), elapsed=1)
        self.assertEqual(r["trial_reason"], "REQUEST_DENIED")

    def test_heading_deviation(self):
        gate = TrialGate()
        tick(gate)
        r = tick(gate, pose=(0.1, 0, 0.09), elapsed=1)
        self.assertEqual(r["trial_reason"], "REQUEST_DENIED")

    def test_excessive_speed(self):
        gate = TrialGate()
        tick(gate)
        r = tick(gate, elapsed=1, speed=0.11)
        self.assertEqual(r["trial_reason"], "REQUEST_DENIED")
        self.assertEqual(r["proposed_linear_m_s"], 0)


if __name__ == "__main__":
    unittest.main()
