"""Verifica indipendente del riarmo V6. Nessuna attuazione ROS."""

import math
from pulse1_v6_safety_contract import stopped_independently


def fresh(value, maximum):
    return (
        type(value) in (int, float)
        and math.isfinite(value)
        and 0 <= value <= maximum
    )


def verify_rearm(
    *, previous_id, next_id, operator_authorized,
    previous_executor_exited, watchdog, watchdog_age_s,
    guard, guard_age_s, cmd, cmd_age_s,
    world_paused, world_age_s,
):
    if operator_authorized is not True:
        return False
    if previous_executor_exited is not True:
        return False

    if (
        type(previous_id) is not str
        or not 8 <= len(previous_id) <= 64
        or type(next_id) is not str
        or not 8 <= len(next_id) <= 64
        or next_id == previous_id
    ):
        return False

    if not fresh(watchdog_age_s, 0.6):
        return False
    if not isinstance(watchdog, dict):
        return False

    if not all((
        watchdog.get("source") == "v6_geofence_watchdog",
        watchdog.get("armed") is True,
        watchdog.get("segment_id") == previous_id,
        watchdog.get("action") == "COMPLETE",
        watchdog.get("reason") == "TARGET_REACHED",
        watchdog.get("pause_ack") is True,
    )):
        return False

    return stopped_independently(
        world_paused=world_paused,
        world_age_s=world_age_s,
        guard=guard,
        guard_age_s=guard_age_s,
        cmd=cmd,
        cmd_age_s=cmd_age_s,
        strict_v6=True,
        nominal_segment_id=previous_id,
    )
