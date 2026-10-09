#!/usr/bin/env python3
"""Health Score diagnostico di PULSE-1."""

import json
from pathlib import Path

LIMITS = {
    'linux': 25,
    'ros2': 30,
    'sensors': 35,
    'web': 10,
}


def evaluate(data):
    scores = {key: 0 for key in LIMITS}
    problems = []
    linux = data.get('linux', {})

    # Linux: CPU, massimo 10 punti
    cpu = linux.get('cpu_percent')
    if cpu is None:
        problems.append('CPU: metrica assente')
    elif cpu < 70:
        scores['linux'] += 10
    elif cpu < 85:
        scores['linux'] += 7
        problems.append(f'CPU elevata: {cpu}%')
    elif cpu < 95:
        scores['linux'] += 3
        problems.append(f'CPU molto elevata: {cpu}%')
    else:
        problems.append(f'CPU critica: {cpu}%')

    # Linux: memoria e disco, 6 punti ciascuno
    for key, label in [
        ('memory_percent', 'RAM'),
        ('disk_percent', 'Disco'),
    ]:
        value = linux.get(key)
        if value is None:
            problems.append(f'{label}: metrica assente')
        elif value < 80:
            scores['linux'] += 6
        elif value < 90:
            scores['linux'] += 3
            problems.append(f'{label} elevato: {value}%')
        else:
            problems.append(f'{label} critico: {value}%')

    # Linux: load average, massimo 3 punti
    load = linux.get('load_1m')
    cores = linux.get('cpu_cores')
    if load is not None and cores:
        ratio = load / cores
        if ratio <= 1.5:
            scores['linux'] += 3
        elif ratio <= 2.5:
            scores['linux'] += 1
            problems.append(f'Load elevato: {load}')
        else:
            problems.append(f'Load critico: {load}')
    else:
        problems.append('Load: metrica assente')

    # ROS 2: 15 punti per ogni nodo
    nodes = data.get('nodes', {})
    for name in ('pulse1_bridge', 'pulse1_controller'):
        if nodes.get(name) is True:
            scores['ros2'] += 15
        else:
            problems.append(f'Nodo ROS assente: {name}')

    # Sensori: freschezza e frequenza
    targets = {
        'odometry': (10, 1.0),
        'imu': (10, 10.0),
        'lidar': (15, 5.0),
    }

    for name, (points, nominal_hz) in targets.items():
        info = data.get('topics', {}).get(name, {})

        if info.get('status') != 'OK':
            problems.append(f'Sensore non aggiornato: {name}')
            continue

        rate = info.get('rate_hz')
        if rate is None:
            problems.append(f'Frequenza assente: {name}')
        elif rate >= nominal_hz * 0.75:
            scores['sensors'] += points
        elif rate >= nominal_hz * 0.45:
            scores['sensors'] += points // 2
            problems.append(f'Frequenza bassa: {name} ({rate} Hz)')
        else:
            problems.append(f'Frequenza critica: {name} ({rate} Hz)')

    # Web: nessun punto senza una verifica HTTP reale
    web_status = data.get('web', {}).get('status', 'NOT_CHECKED')
    if web_status == 'OK':
        scores['web'] = 10
    elif web_status == 'FAIL':
        problems.append('Servizio web non disponibile')

    total = sum(scores.values())

    if web_status not in ('OK', 'FAIL'):
        status = 'PARTIAL'
    elif total >= 90 and not problems:
        status = 'OK'
    elif total >= 70:
        status = 'DEGRADED'
    else:
        status = 'FAIL'

    return {
        'robot': data.get('robot', 'PULSE-1'),
        'score': total,
        'max_score': 100,
        'status': status,
        'web_checked': web_status in ('OK', 'FAIL'),
        'components': {
            key: {'score': scores[key], 'max': maximum}
            for key, maximum in LIMITS.items()
        },
        'problems': problems,
    }


if __name__ == '__main__':
    path = (
        Path(__file__).resolve().parent.parent
        / 'runtime' / 'pulse1_status.json'
    )
    data = json.loads(path.read_text())
    print(json.dumps(evaluate(data), indent=2, ensure_ascii=False))
