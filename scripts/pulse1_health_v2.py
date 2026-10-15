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
    nodes = data.get('nodes', {})
    web_status = data.get('web', {}).get('status')

    # Robot: controller 30 + sensori 70.
    expected = data.get('controller_expected', 'pulse1_controller')
    controller_ok = nodes.get(expected) is True
    controller_points = 30 if controller_ok else 0
    sensor_points = parts['sensors']['score'] * 2

    robot_score = controller_points + sensor_points

    robot_prefixes = (
        'Sensore non aggiornato:',
        'Frequenza assente:',
        'Frequenza bassa:',
        'Frequenza critica:',
    )

    robot_problems = [
        problem for problem in old['problems']
        if problem.startswith(robot_prefixes)
        or problem == f'Nodo ROS assente: {expected}'
    ]

    if not controller_ok:
        robot_status = 'FAIL'
    elif robot_score == 100 and not robot_problems:
        robot_status = 'OK'
    elif robot_score >= 70:
        robot_status = 'DEGRADED'
    else:
        robot_status = 'FAIL'

    # Piattaforma: Linux 25, collector 25,
    # web 20, bridge 10, API 20.
    linux_points = parts['linux']['score']
    collector_points = 25
    bridge_ok = nodes.get('pulse1_bridge') is True
    bridge_points = 10 if bridge_ok else 0
    web_points = 20 if web_status == 'OK' else 0

    platform_problems = [
        problem for problem in old['problems']
        if problem not in robot_problems
    ]

    if web_status not in ('OK', 'FAIL'):
        platform_problems.append('Servizio web non verificato')

    platform_problems.append(
        'Diagnostic API non ancora verificata'
    )

    platform_score = (
        linux_points + collector_points
        + bridge_points + web_points
    )

    return {
        'robot_health': {
            'score': robot_score,
            'status': robot_status,
            'problems': robot_problems,
        },
        'platform_health': {
            'score': platform_score,
            'status': 'PARTIAL',
            'components': {
                'linux': {
                    'score': linux_points, 'max': 25
                },
                'collector': {
                    'score': collector_points, 'max': 25
                },
                'web': {
                    'score': web_points, 'max': 20
                },
                'bridge': {
                    'score': bridge_points, 'max': 10
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
