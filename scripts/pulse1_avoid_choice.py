#!/usr/bin/env python3
"""Scelta preliminare del lato AVOID. Nessun comando motore."""

import math

APPROACH_DISTANCE_M = 2.8
MIN_TURN_START_M = 1.6
MIN_SIDE_DISTANCE_M = 2.0


def valid_distance(value):
    return (
        type(value) in (int, float)
        and not math.isnan(value)
        and value > 0
    )


def choose_avoid_side(sectors):
    """Valuta un candidato; NON autorizza una traiettoria.

    sectors: dict con front, left, right.
    inf = nessun ritorno LiDAR.
    None o valori non validi = dato sconosciuto.
    """

    if not isinstance(sectors, dict):
        return "WAIT_DATA"

    for name in ("front", "left", "right"):
        if not valid_distance(sectors.get(name)):
            return "WAIT_DATA"

    front = sectors["front"]
    left = sectors["left"]
    right = sectors["right"]

    if front > APPROACH_DISTANCE_M:
        return "OBSERVE"

    if front < MIN_TURN_START_M:
        return "HOLD_TOO_CLOSE"

    left_possible = left >= MIN_SIDE_DISTANCE_M
    right_possible = right >= MIN_SIDE_DISTANCE_M

    if not left_possible and not right_possible:
        return "HOLD_NO_SIDE"

    if left_possible and not right_possible:
        return "CANDIDATE_LEFT"

    if right_possible and not left_possible:
        return "CANDIDATE_RIGHT"

    # A parita' di letture, convenzione iniziale: sinistra.
    # La scelta definitiva richiedera' verifica della traiettoria.
    if left >= right:
        return "CANDIDATE_LEFT"

    return "CANDIDATE_RIGHT"
