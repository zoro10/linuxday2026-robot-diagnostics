#!/usr/bin/env python3
"""Converte una decisione di rotazione in una richiesta AVOID.

Nessuna dipendenza ROS. Nessuna pubblicazione motore.
"""

import math

FLAGS = (
    "odom_ok",
    "lidar_ok",
    "rear_lidar_ok",
    "imu_ok",
)

MAX_TEST_ANGULAR = 0.10


def make_turn_request(decision, sensors, armed=False):
    flags = {
        name: (
            isinstance(sensors, dict)
            and sensors.get(name) is True
        )
        for name in FLAGS
    }

    angle = (
        decision.get("angular_rad_s")
        if isinstance(decision, dict)
        else None
    )

    linear = (
        decision.get("linear_m_s")
        if isinstance(decision, dict)
        else None
    )

    allowed = all((
        armed is True,
        isinstance(decision, dict),
        decision.get("state") == "TURNING"
        if isinstance(decision, dict) else False,
        all(flags.values()),
        type(linear) in (int, float),
        linear == 0,
        type(angle) in (int, float),
        math.isfinite(angle)
        if type(angle) in (int, float) else False,
        0 < abs(angle) <= MAX_TEST_ANGULAR
        if type(angle) in (int, float)
        and math.isfinite(angle) else False,
    ))

    return {
        "source": "avoid_navigator",
        "mode": "AVOID",
        "state": "AVOIDING" if allowed else "HOLD",
        "reason": (
            "CONTROLLED_TURN" if allowed else "MOTION_DENIED"
        ),
        **flags,
        "proposed_linear_m_s": 0.0,
        "proposed_angular_rad_s": (
            float(angle) if allowed else 0.0
        ),
        "motor_commands": False,
    }
