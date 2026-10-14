#!/usr/bin/env python3
"""Validazione delle richieste di velocità al Motor Guard."""

import math

MAX_SPEED = 0.18
MAX_REQUEST_AGE_S = 0.7


def guarded_velocity(speed, age_s, *, active):
    """Restituisce una velocità sicura oppure zero."""

    if active is not True:
        return 0.0

    if type(speed) not in (int, float):
        return 0.0

    if type(age_s) not in (int, float):
        return 0.0

    if not math.isfinite(speed):
        return 0.0

    if not math.isfinite(age_s):
        return 0.0

    if speed < 0:
        return 0.0

    if age_s < 0 or age_s > MAX_REQUEST_AGE_S:
        return 0.0

    return min(float(speed), MAX_SPEED)
