#!/usr/bin/env python3
"""Controllo di una rotazione breve AVOID, senza ROS e senza motori."""

import math

TARGET_RAD = math.radians(15.0)
MAX_ANGULAR_RAD_S = 0.10
SLOW_ANGULAR_RAD_S = 0.05
MAX_DURATION_S = 6.0
MIN_CLEARANCE_M = 2.10
MAX_POSITION_CHANGE_M = 0.25
MAX_YAW_DISAGREEMENT_RAD = 0.18
TARGET_TOLERANCE_RAD = 0.025


def finite_number(value):
    return (
        type(value) in (int, float)
        and math.isfinite(value)
    )


def angle_difference(current, initial):
    return math.atan2(
        math.sin(current - initial),
        math.cos(current - initial),
    )


def turn_step(
    *,
    start_pose,
    pose,
    integrated_gyro_rad,
    gyro_z,
    elapsed_s,
    sensors_ok,
    clearance_m,
    direction="LEFT",
):
    """Restituisce stato e velocità angolare, mai velocità lineare."""

    def result(state, angular=0.0, reason="", progress=0.0):
        return {
            "state": state,
            "angular_rad_s": angular,
            "linear_m_s": 0.0,
            "reason": reason,
            "progress_rad": progress,
        }

    if direction not in ("LEFT", "RIGHT"):
        return result("ABORT", reason="INVALID_DIRECTION")

    if sensors_ok is not True:
        return result("ABORT", reason="SENSOR_UNAVAILABLE")

    if (
        not isinstance(start_pose, (tuple, list))
        or not isinstance(pose, (tuple, list))
        or len(start_pose) != 3
        or len(pose) != 3
    ):
        return result("ABORT", reason="INVALID_POSE")

    values = (
        *start_pose, *pose,
        integrated_gyro_rad,
        gyro_z,
        elapsed_s,
        clearance_m,
    )

    if not all(finite_number(v) for v in values):
        return result("ABORT", reason="INVALID_MEASUREMENT")

    if elapsed_s < 0 or elapsed_s >= MAX_DURATION_S:
        return result("ABORT", reason="TIME_LIMIT")

    if clearance_m < MIN_CLEARANCE_M:
        return result("ABORT", reason="INSUFFICIENT_CLEARANCE")

    displacement = math.hypot(
        pose[0] - start_pose[0],
        pose[1] - start_pose[1],
    )

    if displacement > MAX_POSITION_CHANGE_M:
        return result("ABORT", reason="UNEXPECTED_TRANSLATION")

    sign = 1 if direction == "LEFT" else -1

    progress = sign * angle_difference(
        pose[2], start_pose[2]
    )

    gyro_progress = sign * integrated_gyro_rad
    signed_rate = sign * gyro_z

    if abs(gyro_z) > 0.7:
        return result("ABORT", reason="EXCESSIVE_ANGULAR_RATE")

    if progress < -0.05 or gyro_progress < -0.07:
        return result("ABORT", reason="WRONG_TURN_DIRECTION")

    if (
        progress > TARGET_RAD + 0.10
        or gyro_progress > TARGET_RAD + 0.10
    ):
        return result("ABORT", reason="ANGLE_OVERSHOOT")

    if (
        abs(progress - gyro_progress)
        > MAX_YAW_DISAGREEMENT_RAD
    ):
        return result("ABORT", reason="ODOM_IMU_DISAGREE")

    if elapsed_s > 1.0 and signed_rate < -0.04:
        return result("ABORT", reason="REVERSE_ANGULAR_RATE")

    if progress >= TARGET_RAD - TARGET_TOLERANCE_RAD:
        return result("DONE", reason="TARGET_REACHED",
                      progress=progress)

    speed = (
        SLOW_ANGULAR_RAD_S
        if progress >= TARGET_RAD * 0.6
        else MAX_ANGULAR_RAD_S
    )

    return result(
        "TURNING",
        angular=sign * speed,
        reason="CONTROLLED_TURN",
        progress=progress,
    )
