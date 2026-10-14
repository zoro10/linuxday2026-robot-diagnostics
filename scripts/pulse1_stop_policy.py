#!/usr/bin/env python3
"""Politica di velocità per la missione STOP."""

import math

from pulse1_navigation_math import (
    GOAL_TOLERANCE,
    STOP_DISTANCE,
)

MAX_SPEED = 0.18
MAX_ODOM_AGE = 2.5
MAX_LIDAR_AGE = 1.0
MAX_HEADING_ERROR = 0.15


def stop_velocity(decision, front_m, odom_age, lidar_age, *, mode):
    """Restituisce la velocità consentita, oppure zero."""

    if mode != "STOP":
        return 0.0

    for age, limit in (
        (odom_age, MAX_ODOM_AGE),
        (lidar_age, MAX_LIDAR_AGE),
    ):
        if (
            not isinstance(age, (int, float))
            or not math.isfinite(age)
            or age < 0
            or age > limit
        ):
            return 0.0

    if decision.state != "DRIVE":
        return 0.0

    distance = decision.goal_distance_m
    heading = decision.heading_error_rad

    if (
        distance is None
        or heading is None
        or not math.isfinite(distance)
        or not math.isfinite(heading)
    ):
        return 0.0

    if distance <= GOAL_TOLERANCE:
        return 0.0

    if abs(heading) > MAX_HEADING_ERROR:
        return 0.0

    if (
        not isinstance(front_m, (int, float))
        or math.isnan(front_m)
        or front_m <= STOP_DISTANCE
    ):
        return 0.0

    # +inf è ammesso soltanto se proviene
    # da una scansione già validata.
    return MAX_SPEED
