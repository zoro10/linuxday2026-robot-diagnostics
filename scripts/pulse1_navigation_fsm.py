#!/usr/bin/env python3
"""PULSE-1: decisioni di navigazione, senza comandi motore."""

import math
from dataclasses import dataclass

from pulse1_navigation_math import (
    mission_goal,
    goal_error,
    STOP_DISTANCE,
    AVOID_TRIGGER_DISTANCE,
    GOAL_TOLERANCE,
)


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    goal_distance_m: float | None
    heading_error_rad: float | None


class MissionFSM:
    def __init__(self, mode, start_x, start_y, start_yaw):
        mode = mode.upper()

        if mode not in ("STOP", "AVOID"):
            raise ValueError("Modalita ammesse: STOP, AVOID")

        if not all(map(math.isfinite, (
            start_x, start_y, start_yaw
        ))):
            raise ValueError("Posizione iniziale non valida")

        self.mode = mode
        self.goal_x, self.goal_y = mission_goal(
            start_x, start_y, start_yaw
        )
        self.state = "IDLE"

    def decide(self, state, reason, distance, heading):
        self.state = state
        return Decision(state, reason, distance, heading)

    def update(
        self, x, y, yaw, front_m,
        odom_ok=True, lidar_ok=True
    ):
        # Una missione completata rimane completata.
        if self.state == "ARRIVED":
            return self.decide(
                "ARRIVED", "Meta gia raggiunta", 0.0, 0.0
            )

        if not odom_ok or not all(map(math.isfinite, (x, y, yaw))):
            return self.decide(
                "SAFE_STOP", "Odometria assente o non valida",
                None, None
            )

        distance, heading = goal_error(
            x, y, yaw, self.goal_x, self.goal_y
        )

        if not lidar_ok:
            return self.decide(
                "SAFE_STOP", "LiDAR non disponibile",
                distance, heading
            )

        # None, NaN e inf non sono prove di spazio libero.
        if (
            front_m is None
            or not math.isfinite(front_m)
            or front_m <= 0
        ):
            return self.decide(
                "SAFE_STOP", "Distanza LiDAR non determinabile",
                distance, heading
            )

        if distance <= GOAL_TOLERANCE:
            return self.decide(
                "ARRIVED", "Destinazione raggiunta",
                distance, heading
            )

        if self.mode == "STOP":
            if front_m <= STOP_DISTANCE:
                return self.decide(
                    "STOPPED", "Ostacolo rilevato",
                    distance, heading
                )

            return self.decide(
                "DRIVE", "Percorso frontale disponibile",
                distance, heading
            )

        # Modalita AVOID: nessuna manovra e ancora autorizzata.
        if front_m <= STOP_DISTANCE:
            return self.decide(
                "SAFE_STOP",
                "Ostacolo troppo vicino per manovrare",
                distance, heading
            )

        if front_m <= AVOID_TRIGGER_DISTANCE:
            return self.decide(
                "ASSESS",
                "Ostacolo rilevato: valutare un passaggio",
                distance, heading
            )

        return self.decide(
            "DRIVE", "Avanzamento consentito dalla logica base",
            distance, heading
        )
