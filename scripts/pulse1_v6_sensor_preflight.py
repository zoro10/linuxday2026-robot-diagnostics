#!/usr/bin/env python3
"""Pure sensor preflight with explicit diagnostics; NEVER grants motor authority.

Uses the unchanged safety contract as the sole permission predicate. Diagnostics
explain rejection but cannot make a rejected request pass.
"""
from pulse1_v6_safety_contract import (
    observer_healthy, front_clear, turn_clearance_detail,
)


def check_sensor_gate(observer, observer_age_s, kind):
    """Return (ready, reason) for a prospective ADVANCE or TURN.

    A True here is merely a prerequisite for independent Guard/Geofence
    authorization, NOT permission to command motors.
    """
    if kind not in ('ADVANCE', 'TURN'):
        return (False, 'INVALID_MANEUVER')
    if not observer_healthy(observer, observer_age_s):
        return (False, 'OBSERVER_NOT_HEALTHY')
    if kind == 'ADVANCE':
        return ((True, 'FRONT_CLEAR') if front_clear(observer)
                else (False, 'FRONT_CLEARANCE_DENIED'))
    # Diagnostics and admission are from the SAME policy used by the independent
    # Guard and Geofence. Never allow a successful diagnostic to override it.
    return turn_clearance_detail(observer)
