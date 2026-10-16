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


# Limite iniziale per le prove AVOID.
MAX_ANGULAR_SPEED_RAD_S = 0.35


def guarded_motion(linear_m_s, angular_rad_s, age_s, *, active):
    """Valida insieme velocità lineare e angolare.

    In caso di dato non valido entrambe diventano zero.
    Per ora non autorizza la retromarcia.
    """

    if active is not True:
        return 0.0, 0.0

    values = (linear_m_s, angular_rad_s, age_s)

    for value in values:
        if type(value) not in (int, float):
            return 0.0, 0.0
        if not math.isfinite(value):
            return 0.0, 0.0

    if age_s < 0 or age_s > MAX_REQUEST_AGE_S:
        return 0.0, 0.0

    if linear_m_s < 0:
        return 0.0, 0.0

    linear = min(float(linear_m_s), MAX_SPEED)

    angular = max(
        -MAX_ANGULAR_SPEED_RAD_S,
        min(float(angular_rad_s), MAX_ANGULAR_SPEED_RAD_S)
    )

    return linear, angular


AVOID_MOVING_STATES = frozenset((
    "GO_TO_GOAL",
    "AVOIDING",
    "REJOINING_GOAL",
))


def authorized_request(data, selected_mode):
    """Verifica il contratto di una richiesta di movimento.

    La modalità deve essere selezionata prima della missione.
    Il controllo numerico delle velocità rimane a guarded_motion().
    """

    if not isinstance(data, dict):
        return False

    if data.get("motor_commands") is not False:
        return False

    if data.get("odom_ok") is not True:
        return False

    if data.get("lidar_ok") is not True:
        return False

    if selected_mode == "STOP":
        return all((
            data.get("source") == "stop_dryrun",
            data.get("mode") == "STOP",
            data.get("state") == "DRIVE",
        ))

    if selected_mode == "AVOID":
        return all((
            data.get("source") == "avoid_navigator",
            data.get("mode") == "AVOID",
            data.get("state") in AVOID_MOVING_STATES,
            data.get("imu_ok") is True,
            data.get("rear_lidar_ok") is True,
        ))

    return False
