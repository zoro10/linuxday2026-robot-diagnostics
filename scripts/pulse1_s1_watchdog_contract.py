#!/usr/bin/env python3
"""Contratto fail-closed tra watchdog S1 e Motor Guard."""

import math

MAX_WATCHDOG_AGE_S = 0.35


def watchdog_allows_trial(status, age_s):
    """True soltanto con watchdog armato, pronto e recente."""

    return (
        isinstance(status, dict)
        and type(age_s) in (int, float)
        and math.isfinite(age_s)
        and 0 <= age_s <= MAX_WATCHDOG_AGE_S
        and status.get("source") == "s1_geofence_watchdog"
        and status.get("mode") == "AVOID"
        and status.get("armed") is True
        and status.get("ready_for_motion") is True
        and status.get("trial_active") is True
        and status.get("action") == "MONITOR"
        and status.get("reason") == "WITHIN_LIMITS"
        and status.get("motor_commands") is False
    )
