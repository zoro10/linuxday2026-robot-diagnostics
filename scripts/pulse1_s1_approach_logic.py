#!/usr/bin/env python3
"""S1 APPROACH/STOP: logica pura, nessun comando motore."""

import math

SHADOW_SPEED_M_S = 0.10
STOP_FRONT_LIDAR_M = 1.60
MAX_APPROACH_X_M = 2.60
MAX_LATERAL_ERROR_M = 0.15
MAX_HEADING_ERROR_RAD = 0.12
MAX_OBSERVER_AGE_S = 1.50

LIDAR_TO_FRONT_BUMPER_M = 0.55571


def finite(value):
    return (
        type(value) in (int, float)
        and math.isfinite(value)
    )


def approach_step(
    *,
    front_distance_m,
    pose,
    sensors_ok,
    observer_age_s,
):
    """Decisione shadow. Mai autorizzabile direttamente."""

    def result(state, reason, speed=0.0, gap=None):
        return {
            "state": state,
            "reason": reason,
            "shadow_linear_m_s": speed,
            "proposed_linear_m_s": 0.0,
            "proposed_angular_rad_s": 0.0,
            "front_bumper_gap_m": gap,
            "motor_commands": False,
        }

    if sensors_ok is not True:
        return result("HOLD", "SENSORS_UNAVAILABLE")

    if (
        not finite(observer_age_s)
        or not 0 <= observer_age_s <= MAX_OBSERVER_AGE_S
    ):
        return result("HOLD", "STALE_OBSERVER")

    if (
        not isinstance(pose, (list, tuple))
        or len(pose) != 3
        or not all(finite(v) for v in pose)
    ):
        return result("HOLD", "INVALID_POSE")

    x, y, yaw = pose

    if (
        x < -0.15
        or abs(y) > MAX_LATERAL_ERROR_M
        or abs(yaw) > MAX_HEADING_ERROR_RAD
    ):
        return result("HOLD", "UNEXPECTED_POSE")

    if (
        not finite(front_distance_m)
        or front_distance_m <= 0
    ):
        return result("HOLD", "INVALID_FRONT_DISTANCE")

    gap = max(
        0.0,
        front_distance_m - LIDAR_TO_FRONT_BUMPER_M,
    )

    if front_distance_m <= STOP_FRONT_LIDAR_M:
        return result(
            "STOP", "FRONT_DISTANCE_LIMIT", gap=gap
        )

    if x >= MAX_APPROACH_X_M:
        return result(
            "STOP", "POSITION_LIMIT", gap=gap
        )

    return result(
        "APPROACH",
        "SHADOW_APPROACH",
        speed=SHADOW_SPEED_M_S,
        gap=gap,
    )
