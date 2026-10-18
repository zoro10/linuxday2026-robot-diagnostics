import json
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from pulse1_motor_guard_live import MotorGuardLive


def ready():
    return dict(
        source="s1_geofence_watchdog",
        mode="AVOID",
        armed=True,
        ready_for_motion=True,
        trial_active=True,
        action="MONITOR",
        reason="WITHIN_LIMITS",
        motor_commands=False,
    )


def check(speed=0.10, angular=0.0, watchdog=None, age=0.1):
    now = time.monotonic()
    node = SimpleNamespace(
        selected_mode="AVOID",
        request_at=now - 0.1,
        request_speed=speed,
        request_angular=angular,
        request_allowed=True,
        request_data=dict(
            state="GO_TO_GOAL",
            trial_phase="DRIVE",
            trial_reason="LIMITED_FORWARD_TRIAL",
        ),
        upstream_state="GO_TO_GOAL",
        watchdog_status=watchdog,
        watchdog_at=now-age if watchdog is not None else None,
        sensor_at={
            name: now-0.1
            for name in ("odom", "front", "rear", "imu")
        },
        output=Mock(),
        motor=Mock(),
    )
    MotorGuardLive.control(node)
    status = json.loads(node.output.publish.call_args.args[0].data)
    cmd = node.motor.publish.call_args.args[0]
    return status["guard_state"], cmd.linear.x, cmd.angular.z


class GuardIntegrationTests(unittest.TestCase):
    def test_forward_authorized(self):
        self.assertEqual(check(watchdog=ready()), ("AUTHORIZED", 0.1, 0.0))

    def test_watchdog_missing(self):
        self.assertEqual(check(), ("BLOCKED", 0.0, 0.0))

    def test_watchdog_stale(self):
        self.assertEqual(check(watchdog=ready(), age=0.5),
                         ("BLOCKED", 0.0, 0.0))

    def test_watchdog_disarmed(self):
        data = ready()
        data["armed"] = False
        self.assertEqual(check(watchdog=data), ("BLOCKED", 0.0, 0.0))

    def test_watchdog_alarm(self):
        data = ready()
        data["action"] = "PAUSE"
        self.assertEqual(check(watchdog=data), ("BLOCKED", 0.0, 0.0))

    def test_invalid_speeds(self):
        for speed in (-0.01, 0.11):
            with self.subTest(speed=speed):
                self.assertEqual(check(speed=speed, watchdog=ready()),
                                 ("BLOCKED", 0.0, 0.0))

    def test_forward_with_rotation(self):
        self.assertEqual(check(angular=0.05, watchdog=ready()),
                         ("BLOCKED", 0.0, 0.0))

    def test_previous_rotation_compatible(self):
        self.assertEqual(check(speed=0.0, angular=0.15),
                         ("AUTHORIZED", 0.0, 0.15))
