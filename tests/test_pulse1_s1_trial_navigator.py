import json
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from pulse1_s1_trial_gate import TrialGate
from pulse1_s1_trial_navigator import (
    TrialNavigator, MAX_INPUT_AGE_S,
)


FLAGS = (
    "odom_ok", "lidar_ok",
    "rear_lidar_ok", "imu_ok",
)


def fixture(armed=True):
    now = time.monotonic()

    observer = {
        "source": "avoid_observer",
        "mode": "AVOID",
        "state": "SENSOR_READY",
        "motor_commands": False,
        **{flag: True for flag in FLAGS},
    }

    shadow = {
        "source": "s1_approach_shadow",
        "mode": "AVOID",
        "state": "HOLD",
        "sensors_ok": True,
        "motor_commands": False,
        "proposed_linear_m_s": 0.0,
        "proposed_angular_rad_s": 0.0,
        "shadow_state": "APPROACH",
        "shadow_reason": "SHADOW_APPROACH",
        "shadow_linear_m_s": 0.10,
        "pose": (0.0, 0.0, 0.0),
    }

    node = SimpleNamespace(
        armed=armed,
        gate=TrialGate(),
        created_at=now,
        started_at=None,
        observer=observer,
        shadow=shadow,
        observer_at=now,
        shadow_at=now,
        diagnostic=Mock(),
        request_pub=Mock() if armed else None,
    )

    node.fresh = lambda stamp, instant: (
        stamp is not None
        and 0 <= instant - stamp <= MAX_INPUT_AGE_S
    )

    return node


def run(node):
    TrialNavigator.control(node)

    status = json.loads(
        node.diagnostic.publish.call_args.args[0].data
    )

    request = None
    if node.request_pub is not None:
        request = json.loads(
            node.request_pub.publish.call_args.args[0].data
        )

    return status, request


class TrialNavigatorTests(unittest.TestCase):

    def test_disarmed(self):
        node = fixture(armed=False)
        status, request = run(node)
        self.assertEqual(status["request_state"], "HOLD")
        self.assertIsNone(request)
        self.assertFalse(node.gate.started)

    def test_armed_forward(self):
        node = fixture()
        status, request = run(node)
        self.assertEqual(request["state"], "GO_TO_GOAL")
        self.assertEqual(request["trial_phase"], "DRIVE")
        self.assertEqual(request["proposed_linear_m_s"], 0.10)
        self.assertEqual(request["proposed_angular_rad_s"], 0)
        self.assertTrue(node.gate.started)

    def test_stop_at_20cm_and_latch(self):
        node = fixture()
        run(node)

        node.shadow["pose"] = (0.20, 0.0, 0.0)
        status, request = run(node)

        self.assertEqual(request["state"], "HOLD")
        self.assertEqual(request["trial_phase"], "DONE")
        self.assertEqual(request["proposed_linear_m_s"], 0)
        self.assertEqual(request["trial_reason"], "TARGET_REACHED")

        node.shadow["pose"] = (0.0, 0.0, 0.0)
        status, request = run(node)

        self.assertEqual(request["state"], "HOLD")
        self.assertEqual(request["trial_phase"], "DONE")

    def test_stale_before_start(self):
        node = fixture()
        node.shadow_at = None

        status, request = run(node)

        self.assertEqual(request["state"], "HOLD")
        self.assertIsNone(node.started_at)

    def test_sensor_loss_after_start(self):
        node = fixture()
        run(node)

        node.observer_at = None
        status, request = run(node)

        self.assertEqual(request["state"], "HOLD")
        self.assertEqual(request["trial_reason"], "REQUEST_DENIED")

        node.observer_at = time.monotonic()
        status, request = run(node)

        self.assertEqual(request["state"], "HOLD")

    def test_startup_timeout(self):
        node = fixture()
        node.created_at = time.monotonic() - 11
        node.shadow_at = None

        status, request = run(node)

        self.assertEqual(request["state"], "HOLD")
        self.assertEqual(
            request["trial_reason"], "STARTUP_TIMEOUT"
        )

    def test_unsafe_start_pose(self):
        node = fixture()
        node.shadow["pose"] = (0.04, 0.0, 0.0)

        status, request = run(node)

        self.assertEqual(request["state"], "HOLD")
        self.assertEqual(
            request["trial_reason"], "UNSAFE_START_POSE"
        )


if __name__ == "__main__":
    unittest.main()
