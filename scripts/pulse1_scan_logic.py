#!/usr/bin/env python3
"""Validazione prudenziale del settore LiDAR frontale."""

import math


def front_distance(
    ranges, angle_min, angle_increment,
    range_min, range_max, half_angle=0.35
):
    metadata = (
        angle_min, angle_increment, range_min, range_max
    )

    if not all(math.isfinite(x) for x in metadata):
        return None

    if angle_increment <= 0 or range_min <= 0:
        return None

    if range_max <= range_min:
        return None

    hits = []
    beams = 0

    for i, distance in enumerate(ranges):
        angle = angle_min + i * angle_increment

        if abs(angle) > half_angle:
            continue

        beams += 1

        # Nessun ritorno entro la portata.
        if distance == math.inf:
            continue

        # Qualsiasi misura frontale anomala
        # impedisce di dichiarare sicuro il settore.
        if (
            not math.isfinite(distance)
            or distance < range_min
            or distance > range_max
        ):
            return None

        hits.append(distance)

    if beams < 5:
        return None

    return min(hits) if hits else math.inf
