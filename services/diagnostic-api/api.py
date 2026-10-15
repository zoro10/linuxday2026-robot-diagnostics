#!/usr/bin/env python3
"""API diagnostica PULSE-1, indipendente da ROS 2."""

import json
from collections import deque
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from pulse1_health_v2 import evaluate

DATA = Path(os.getenv(
    'PULSE1_STATUS_PATH', '/runtime/pulse1_status.json'
))


EVENTS = DATA.with_name('pulse1_events.jsonl')
WORLD = DATA.with_name('pulse1_world.json')


def get_events(limit=50):
    events = deque(maxlen=limit)

    try:
        with EVENTS.open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    event = json.loads(line)
                except ValueError:
                    continue

                if not isinstance(event, dict):
                    continue

                if event.get('type') not in ('FAULT', 'RECOVERY'):
                    continue

                if not isinstance(event.get('component'), str):
                    continue

                if not isinstance(event.get('timestamp_utc'), str):
                    continue

                events.append(event)
    except FileNotFoundError:
        return []

    return list(reversed(events))


def get_diagnostics():
    try:
        data = json.loads(DATA.read_text())
    except (OSError, ValueError):
        data = {}

    result = evaluate(data)
    platform = result['platform_health']

    # L'API attiva vale 20 punti.
    platform['score'] = min(100, platform['score'] + 20)
    platform.setdefault('components', {})['api'] = {
        'score': 20, 'max': 20
    }
    platform['problems'] = [
        p for p in platform['problems']
        if p != 'Diagnostic API non ancora verificata'
    ]

    if result['telemetry']['status'] != 'FRESH':
        platform['status'] = 'FAIL'
    elif data.get('web', {}).get('status') not in ('OK', 'FAIL'):
        platform['status'] = 'PARTIAL'
    elif platform['score'] == 100 and not platform['problems']:
        platform['status'] = 'OK'
    elif platform['score'] < 70:
        platform['status'] = 'FAIL'
    else:
        platform['status'] = 'DEGRADED'

    return data, result


class Handler(BaseHTTPRequestHandler):
    def respond(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path

        if path == '/api/health':
            self.respond(200, {
                'service': 'pulse1-diagnostic-api',
                'status': 'OK'
            })
            return

        if path == '/api/world':
            try:
                payload = json.loads(
                    WORLD.read_text(encoding='utf-8')
                )
                self.respond(200, payload)
            except (OSError, ValueError):
                self.respond(503, {
                    'error': 'World metadata unavailable'
                })
            return

        if path == '/api/events':
            try:
                events = get_events()
                self.respond(200, {
                    'events': events,
                    'count': len(events)
                })
            except OSError:
                self.respond(500, {
                    'error': 'Event journal unavailable'
                })
            return

        if path not in (
            '/api/status', '/api/robot', '/api/platform'
        ):
            self.respond(404, {'error': 'Not found'})
            return

        try:
            data, result = get_diagnostics()
            if path == '/api/robot':
                payload = {
                    'robot_health': result['robot_health'],
                    'telemetry': result['telemetry'],
                    'position': data.get('position'),
                    'topics': data.get('topics', {}),
                    'lidar_front_distance_m':
                        data.get('lidar_front_distance_m')
                }
            elif path == '/api/platform':
                payload = {
                    'platform_health': result['platform_health'],
                    'telemetry': result['telemetry']
                }
            else:
                payload = {
                    'robot': data.get('robot', 'PULSE-1'),
                    'diagnostics': result,
                    'linux': data.get('linux', {}),
                    'web': data.get('web', {}),
                    'nodes': data.get('nodes', {}),
                    'navigation_mode': data.get('navigation_mode'),
                    'controller_expected': data.get('controller_expected'),
                    'timestamp_utc': data.get('timestamp_utc'),
                    'lidar_front_distance_m': data.get('lidar_front_distance_m'),
                    'position': data.get('position'),
                    'topics': data.get('topics', {})
                }
            self.respond(200, payload)
        except Exception:
            self.respond(500, {'error': 'Diagnostic API error'})


if __name__ == '__main__':
    server = ThreadingHTTPServer(
        ('0.0.0.0', 8000), Handler
    )
    print('PULSE-1 Diagnostic API listening on :8000',
          flush=True)
    server.serve_forever()
