import json
import math
import time
import unittest
from types import SimpleNamespace

from std_msgs.msg import String
from pulse1_motor_guard_live import MotorGuardLive


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def fake_guard(mode):
    now = time.monotonic()
    return SimpleNamespace(
        selected_mode=mode,
        sensor_at={
            name: now
            for name in ("odom", "front", "rear", "imu")
        } if mode == "AVOID" else {},
        request_allowed=False,
        request_at=None,
        request_speed=None,
        request_angular=0.0,
        upstream_state=None,
        output=Capture(),
        motor=Capture(),
    )


def request(mode, **changes):
    data = {
        "source": (
            "avoid_navigator"
            if mode == "AVOID" else "stop_dryrun"
        ),
        "mode": mode,
        "state": (
            "AVOIDING" if mode == "AVOID" else "DRIVE"
        ),
        "motor_commands": False,
        "odom_ok": True,
        "lidar_ok": True,
        "rear_lidar_ok": True,
        "imu_ok": True,
        "proposed_linear_m_s": 0.0,
        "proposed_angular_rad_s": 0.15,
    }
    data.update(changes)
    msg = String()
    msg.data = json.dumps(data)
    return msg


def run_control(guard):
    MotorGuardLive.control(guard)
    status = json.loads(guard.output.messages[-1].data)
    cmd = guard.motor.messages[-1]
    return status, cmd


class TestMotorGuardLive(unittest.TestCase):

    def test_avoid_rotation_authorized(self):
        guard = fake_guard("AVOID")
        MotorGuardLive.on_request(
            guard, request("AVOID")
        )
        status, cmd = run_control(guard)
        self.assertEqual(status["guard_state"], "AUTHORIZED")
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertAlmostEqual(cmd.angular.z, 0.15)

    def test_missing_imu_blocks(self):
        guard = fake_guard("AVOID")
        MotorGuardLive.on_request(
            guard, request("AVOID", imu_ok=False)
        )
        status, cmd = run_control(guard)
        self.assertEqual(status["guard_state"], "BLOCKED")
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0, 0))

    def test_invalid_front_scan_blocks(self):
        guard = fake_guard("AVOID")
        MotorGuardLive.on_request(
            guard, request("AVOID")
        )
        invalid_scan = SimpleNamespace(
            ranges=[math.nan] * 10,
            angle_min=-1.0,
            angle_increment=0.2,
            range_min=0.1,
            range_max=12.0,
        )
        MotorGuardLive.on_sensor(
            guard, "front", invalid_scan
        )
        self.assertNotIn("front", guard.sensor_at)
        status, cmd = run_control(guard)
        self.assertEqual(status["guard_state"], "BLOCKED")
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0, 0))

    def test_expired_sensor_blocks(self):
        guard = fake_guard("AVOID")
        MotorGuardLive.on_request(
            guard, request("AVOID")
        )
        guard.sensor_at["imu"] = time.monotonic() - 3.0
        status, cmd = run_control(guard)
        self.assertEqual(status["guard_state"], "BLOCKED")
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0, 0))

    def test_stop_cannot_rotate(self):
        guard = fake_guard("STOP")
        MotorGuardLive.on_request(
            guard, request(
                "STOP",
                proposed_linear_m_s=0.18,
                proposed_angular_rad_s=0.3,
            )
        )
        status, cmd = run_control(guard)
        self.assertEqual(status["guard_state"], "AUTHORIZED")
        self.assertAlmostEqual(cmd.linear.x, 0.18)
        self.assertEqual(cmd.angular.z, 0.0)


if __name__ == "__main__":
    unittest.main()
