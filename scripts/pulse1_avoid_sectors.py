#!/usr/bin/env python3
"""Percezione LiDAR a settori, senza comandi motore."""

import math

SECTORS = {
    "right": (-1.25, -0.35),
    "front": (-0.35, 0.35),
    "left": (0.35, 1.25),
}


def sector_distances(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
):
    """Restituisce le distanze minime per settore.

    None = settore non valutabile.
    inf = nessun ostacolo rilevato entro la portata.
    Valore finito = distanza minima misurata.
    """

    unknown = {name: None for name in SECTORS}

    metadata = (
        angle_min, angle_increment, range_min, range_max
    )

    if not all(
        type(v) in (int, float) and math.isfinite(v)
        for v in metadata
    ):
        return unknown

    if (
        angle_increment <= 0
        or range_min <= 0
        or range_max <= range_min
    ):
        return unknown

    if ranges is None or len(ranges) < 5:
        return unknown

    scan_max = angle_min + (
        len(ranges) - 1
    ) * angle_increment

    result = {}

    for name, (low, high) in SECTORS.items():

        # Il settore deve ricadere nel campo visivo.
        if (
            angle_min > low
            or scan_max < high
        ):
            result[name] = None
            continue

        values = []

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment

            if not low <= angle <= high:
                continue

            values.append(distance)

        if len(values) < 5:
            result[name] = None
            continue

        hits = []
        valid = True

        for distance in values:
            if type(distance) not in (int, float):
                valid = False
                break

            if distance == math.inf:
                continue

            if (
                not math.isfinite(distance)
                or not range_min <= distance <= range_max
            ):
                valid = False
                break

            hits.append(float(distance))

        if not valid:
            result[name] = None
        elif hits:
            result[name] = min(hits)
        else:
            result[name] = math.inf

    return result
