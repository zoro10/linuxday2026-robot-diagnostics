import math
import unittest

from pulse1_s1_geofence_logic import Geofence


def inputs(**changes):
    data = {
        "active": True,
        "pose": (0.10, 0.0, 0.0),
        "pose_age_s": 0.1,
        "guard": {
            "source": "motor_guard_live",
            "mode": "AVOID",
            "motor_commands": True,
            "guard_state": "AUTHORIZED",
            "permitted_linear_m_s": 0.10,
            "permitted_angular_rad_s": 0.0,
        },
        "guard_age_s": 0.1,
        "cmd": (0.10, 0.0),
        "cmd_age_s": 0.1,
        "elapsed_s": 2.0,
    }
    data.update(changes)
    return data


class GeofenceTests(unittest.TestCase):

    def test_wait_before_start(self):
        self.assertEqual(
            Geofence().step()["action"], "WAIT"
        )

    def test_normal_motion(self):
        self.assertEqual(
            Geofence().step(**inputs())["action"],
            "MONITOR",
        )

    def test_emergency_position(self):
        for x in (0.22, 0.25, -0.06):
            with self.subTest(x=x):
                r = Geofence().step(
                    **inputs(pose=(x, 0, 0))
                )
                self.assertEqual(r["action"], "PAUSE")

    def test_lateral_deviation(self):
        r = Geofence().step(
            **inputs(pose=(0.1, 0.11, 0))
        )
        self.assertEqual(r["reason"], "LATERAL_LIMIT")

    def test_unexpected_rotation(self):
        r = Geofence().step(
            **inputs(pose=(0.1, 0, 0.09))
        )
        self.assertEqual(r["reason"], "HEADING_LIMIT")

    def test_excessive_speed(self):
        r = Geofence().step(
            **inputs(cmd=(0.11, 0))
        )
        self.assertEqual(r["reason"], "UNEXPECTED_MOTION")

    def test_angular_motion(self):
        r = Geofence().step(
            **inputs(cmd=(0.1, 0.01))
        )
        self.assertEqual(r["reason"], "UNEXPECTED_MOTION")

    def test_blocked_guard_cannot_move(self):
        guard = dict(inputs()["guard"])
        guard["guard_state"] = "BLOCKED"

        r = Geofence().step(
            **inputs(guard=guard)
        )
        self.assertEqual(r["reason"], "BLOCKED_BUT_MOVING")

    def test_invalid_guard(self):
        r = Geofence().step(
            **inputs(guard={})
        )
        self.assertEqual(r["reason"], "INVALID_GUARD")

    def test_stale_telemetry(self):
        for field in (
            "pose_age_s", "guard_age_s", "cmd_age_s"
        ):
            with self.subTest(field=field):
                r = Geofence().step(
                    **inputs(**{field: 1.2})
                )
                self.assertEqual(r["action"], "PAUSE")

    def test_timeout(self):
        r = Geofence().step(
            **inputs(elapsed_s=8.0)
        )
        self.assertEqual(r["reason"], "TRIAL_TIMEOUT")

    def test_invalid_pose(self):
        r = Geofence().step(
            **inputs(pose=(math.nan, 0, 0))
        )
        self.assertEqual(r["reason"], "INVALID_POSE")

    def test_alarm_is_permanent(self):
        gate = Geofence()
        gate.step(
            **inputs(pose=(0.23, 0, 0))
        )

        r = gate.step(**inputs())
        self.assertEqual(r["action"], "PAUSE")
        self.assertEqual(r["reason"], "POSITION_LIMIT")

    def test_deactivation_after_start(self):
        gate = Geofence()
        gate.step(**inputs())

        r = gate.step(**inputs(active=False))
        self.assertEqual(r["reason"], "TRIAL_DEACTIVATED")

    def test_blocked_zero_command_is_valid(self):
        guard = dict(inputs()["guard"])
        guard["guard_state"] = "BLOCKED"
        guard["permitted_linear_m_s"] = 0.0

        r = Geofence().step(
            **inputs(guard=guard, cmd=(0.0, 0.0))
        )
        self.assertEqual(r["action"], "MONITOR")


if __name__ == "__main__":
    unittest.main()
