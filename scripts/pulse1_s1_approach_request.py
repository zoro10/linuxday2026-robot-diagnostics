#!/usr/bin/env python3
"""Richieste AVOID per il primo avvicinamento limitato.

Logica pura: nessun ROS, nessuna pubblicazione motore.
"""

import math

MAX_TEST_X_M = 0.25
MAX_TEST_SPEED_M_S = 0.10
MAX_LATERAL_M = 0.10
MAX_YAW_RAD = 0.08

SENSOR_FLAGS = (
    "odom_ok",
    "lidar_ok",
    "rear_lidar_ok",
    "imu_ok",
)


def finite(value):
    return (
        type(value) in (int, float)
        and math.isfinite(value)
    )


def make_approach_request(
    decision,
    sensors,
    pose,
    *,
    armed=False,
):
    flags = {
        name: (
            isinstance(sensors, dict)
            and sensors.get(name) is True
        )
        for name in SENSOR_FLAGS
    }

    valid_pose = (
        isinstance(pose, (tuple, list))
        and len(pose) == 3
        and all(finite(v) for v in pose)
    )

    position_ok = (
        valid_pose
        and -0.05 <= pose[0] < MAX_TEST_X_M
        and abs(pose[1]) <= MAX_LATERAL_M
        and abs(pose[2]) <= MAX_YAW_RAD
    )

    valid_decision = (
        isinstance(decision, dict)
        and decision.get("state") == "APPROACH"
        and decision.get("motor_commands") is False
        and decision.get("proposed_linear_m_s") == 0
        and decision.get("proposed_angular_rad_s") == 0
    )

    speed = (
        decision.get("shadow_linear_m_s")
        if isinstance(decision, dict)
        else None
    )

    speed_ok = (
        finite(speed)
        and 0 < speed <= MAX_TEST_SPEED_M_S
    )

    allowed = (
        armed is True
        and all(flags.values())
        and position_ok
        and valid_decision
        and speed_ok
    )

    return {
        "source": "avoid_navigator",
        "mode": "AVOID",
        "state": "GO_TO_GOAL" if allowed else "HOLD",
        "reason": (
            "LIMITED_APPROACH_25CM"
            if allowed
            else "MOTION_DENIED"
        ),
        **flags,
        "proposed_linear_m_s": (
            float(speed) if allowed else 0.0
        ),
        "proposed_angular_rad_s": 0.0,
        "motor_commands": False,
        "approach_state": (
            decision.get("state")
            if isinstance(decision, dict)
            else "HOLD"
        ),
    }
