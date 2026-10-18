#!/usr/bin/env python3
"""Geofence S1: logica pura per pausa d'emergenza."""

import math

EMERGENCY_X_M = 0.22
MIN_X_M = -0.05
MAX_Y_M = 0.10
MAX_YAW_RAD = 0.08
MAX_SPEED_M_S = 0.10
MAX_AGE_S = 0.8
MAX_TRIAL_S = 8.0


def finite(value):
    return (
        type(value) in (int, float)
        and math.isfinite(value)
    )


class Geofence:
    """Il primo allarme rimane memorizzato."""

    def __init__(self):
        self.engaged = False
        self.alarm = None

    def step(
        self, *,
        active=False,
        pose=None,
        pose_age_s=None,
        guard=None,
        guard_age_s=None,
        cmd=None,
        cmd_age_s=None,
        elapsed_s=None,
    ):
        def pause(reason):
            self.alarm = reason
            return {"action": "PAUSE", "reason": reason}

        if self.alarm is not None:
            return {"action": "PAUSE", "reason": self.alarm}

        if active is True:
            self.engaged = True

        if not self.engaged:
            return {"action": "WAIT", "reason": "NOT_STARTED"}

        if active is not True:
            return pause("TRIAL_DEACTIVATED")

        if not finite(elapsed_s) or not 0 <= elapsed_s < MAX_TRIAL_S:
            return pause("TRIAL_TIMEOUT")

        for name, age in (
            ("POSE", pose_age_s),
            ("GUARD", guard_age_s),
            ("CMD", cmd_age_s),
        ):
            if not finite(age) or not 0 <= age <= MAX_AGE_S:
                return pause(name + "_STALE")

        if (
            not isinstance(pose, (tuple, list))
            or len(pose) != 3
            or not all(finite(v) for v in pose)
        ):
            return pause("INVALID_POSE")

        x, y, yaw = pose

        if x >= EMERGENCY_X_M or x < MIN_X_M:
            return pause("POSITION_LIMIT")

        if abs(y) > MAX_Y_M:
            return pause("LATERAL_LIMIT")

        if abs(yaw) > MAX_YAW_RAD:
            return pause("HEADING_LIMIT")

        if (
            not isinstance(guard, dict)
            or guard.get("source") != "motor_guard_live"
            or guard.get("mode") != "AVOID"
            or guard.get("motor_commands") is not True
            or guard.get("guard_state")
               not in ("AUTHORIZED", "BLOCKED")
        ):
            return pause("INVALID_GUARD")

        if (
            not isinstance(cmd, (tuple, list))
            or len(cmd) != 2
            or not all(finite(v) for v in cmd)
        ):
            return pause("INVALID_CMD")

        speed, angular = cmd
        permitted_speed = guard.get("permitted_linear_m_s")
        permitted_angular = guard.get("permitted_angular_rad_s")

        if not finite(permitted_speed) or not finite(permitted_angular):
            return pause("INVALID_PERMISSION")

        if (
            speed < 0
            or speed > MAX_SPEED_M_S + 1e-6
            or abs(angular) > 1e-6
            or permitted_speed < 0
            or permitted_speed > MAX_SPEED_M_S + 1e-6
            or abs(permitted_angular) > 1e-6
        ):
            return pause("UNEXPECTED_MOTION")

        if (
            guard["guard_state"] == "BLOCKED"
            and (abs(speed) > 1e-6 or abs(angular) > 1e-6)
        ):
            return pause("BLOCKED_BUT_MOVING")

        return {"action": "MONITOR", "reason": "WITHIN_LIMITS"}
