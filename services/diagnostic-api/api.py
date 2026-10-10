#!/usr/bin/env python3
"""API diagnostica PULSE-1, indipendente da ROS 2."""

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from pulse1_health_v2 import evaluate

DATA = Path(os.getenv(
    'PULSE1_STATUS_PATH', '/runtime/pulse1_status.json'
))


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
    elif platform['score'] == 100 and not platform['problems']:
        platform['status'] = 'OK'
    elif data.get('web', {}).get('status') == 'FAIL':
        platform['status'] = 'DEGRADED'
    else:
        platform['status'] = 'PARTIAL'

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
