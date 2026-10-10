#!/usr/bin/env python3
"""Separazione Robot Health / Platform Health."""

import json
from datetime import datetime, timezone
from pathlib import Path

from pulse1_health_score import evaluate as evaluate_v03


def evaluate(data, now=None):
    now = now or datetime.now(timezone.utc)

    try:
        timestamp = datetime.fromisoformat(
            data['timestamp_utc']
        )
        if timestamp.tzinfo is None:
            raise ValueError('Timestamp senza timezone')
        age = (now - timestamp).total_seconds()
    except (KeyError, TypeError, ValueError):
        age = None

    fresh = age is not None and -2 <= age < 10

    telemetry = {
        'status': 'FRESH' if fresh else 'STALE',
        'age_seconds': (
            round(max(0, age), 2)
            if age is not None else None
        ),
    }

    if not fresh:
        return {
            'robot_health': {
                'score': None,
                'status': 'UNKNOWN',
                'problems': ['Telemetria non aggiornata'],
            },
            'platform_health': {
                'score': 0,
                'status': 'FAIL',
                'problems': ['Collector non aggiornato'],
            },
            'telemetry': telemetry,
        }

    old = evaluate_v03(data)
    parts = old['components']

    # Robot: 25 Linux + 30 ROS + 35 sensori
    robot_points = sum(
        parts[name]['score']
        for name in ('linux', 'ros2', 'sensors')
    )
    robot_score = round(robot_points * 100 / 90)

    robot_status = (
        'OK' if robot_score == 100
        else 'DEGRADED' if robot_score >= 70
        else 'FAIL'
    )

    robot_problems = [
        p for p in old['problems']
        if not p.startswith('Servizio web')
    ]

    # Piattaforma: collector 40, web 40, API 20
    web_ok = data.get('web', {}).get('status') == 'OK'
    web_fail = data.get('web', {}).get('status') == 'FAIL'

    platform_score = 40 + (40 if web_ok else 0)

    if web_fail:
        platform_status = 'DEGRADED'
        platform_problems = ['Servizio web non disponibile']
    else:
        platform_status = 'PARTIAL'
        platform_problems = []

    platform_problems.append(
        'Diagnostic API non ancora verificata'
    )

    return {
        'robot_health': {
            'score': robot_score,
            'status': robot_status,
            'problems': robot_problems,
        },
        'platform_health': {
            'score': platform_score,
            'status': platform_status,
            'components': {
                'collector': {'score': 40, 'max': 40},
                'web': {
                    'score': 40 if web_ok else 0,
                    'max': 40,
                },
                'api': {'score': 0, 'max': 20},
            },
            'problems': platform_problems,
        },
        'telemetry': telemetry,
    }


if __name__ == '__main__':
    path = (
        Path(__file__).resolve().parent.parent
        / 'runtime' / 'pulse1_status.json'
    )
    data = json.loads(path.read_text())
    print(json.dumps(evaluate(data), indent=2))
