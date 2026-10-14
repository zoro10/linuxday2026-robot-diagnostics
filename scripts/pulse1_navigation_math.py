#!/usr/bin/env python3
"""Geometria e funzioni pure per PULSE-1 Navigation."""

import math

# Dimensioni note dal modello SDF, in metri.
CHASSIS_X = -0.151427
CHASSIS_LENGTH = 2.01142
CHASSIS_HALF_WIDTH = 0.5

WHEEL_Y = 0.625029
WHEEL_RADIUS = 0.3

# Posizione LiDAR rispetto all'origine del modello.
LIDAR_X = CHASSIS_X + 0.45

# Parametri prudenziali iniziali.
SAFETY_MARGIN = 0.20
STOP_DISTANCE = 1.20
AVOID_TRIGGER_DISTANCE = 1.70
GOAL_DISTANCE = 10.0
GOAL_TOLERANCE = 0.50


def wrap_angle(angle):
    """Normalizza un angolo nell'intervallo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def turning_envelope():
    """Raggio conservativo per una rotazione sul posto."""
    front = CHASSIS_X + CHASSIS_LENGTH / 2
    rear = CHASSIS_X - CHASSIS_LENGTH / 2

    half_length = max(abs(front), abs(rear))
    half_width = max(
        CHASSIS_HALF_WIDTH,
        WHEEL_Y + WHEEL_RADIUS
    )

    radius = math.hypot(half_length, half_width)
    return radius + SAFETY_MARGIN


def mission_goal(start_x, start_y, start_yaw):
    """Meta fissa a 10 m davanti alla posa iniziale."""
    return (
        start_x + GOAL_DISTANCE * math.cos(start_yaw),
        start_y + GOAL_DISTANCE * math.sin(start_yaw)
    )


def goal_error(x, y, yaw, goal_x, goal_y):
    """Restituisce distanza e angolo verso la meta."""
    dx = goal_x - x
    dy = goal_y - y

    distance = math.hypot(dx, dy)
    heading = math.atan2(dy, dx)

    return distance, wrap_angle(heading - yaw)


def sector_minimum(
    ranges,
    angle_min,
    angle_increment,
    center_deg,
    half_width_deg,
    range_min,
    range_max
):
    """Distanza minima valida in un settore LiDAR.

    None significa misura insufficiente, NON spazio libero.
    """
    distances = []

    for index, distance in enumerate(ranges):
        angle = math.degrees(
            angle_min + index * angle_increment
        )

        if abs(angle - center_deg) > half_width_deg:
            continue

        if (
            math.isfinite(distance)
            and range_min <= distance <= range_max
        ):
            distances.append(distance)

    return min(distances) if distances else None
