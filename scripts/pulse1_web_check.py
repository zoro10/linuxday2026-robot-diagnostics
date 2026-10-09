#!/usr/bin/env python3
"""Controllo HTTP, PHP e database per PULSE-1."""

import json
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

HEALTH_URL = 'http://127.0.0.1:8080/health.php'


def check_web(url=HEALTH_URL, timeout=3):
    started = time.monotonic()

    result = {
        'status': 'FAIL',
        'http_code': None,
        'response_ms': None,
        'db_response_ms': None,
        'error': None,
    }

    try:
        request = Request(
            url,
            headers={'Accept': 'application/json'}
        )

        with urlopen(request, timeout=timeout) as response:
            result['http_code'] = response.status
            body = response.read(16384)

        payload = json.loads(body)

        if not isinstance(payload, dict):
            raise ValueError('Risposta JSON non valida')

        result['db_response_ms'] = payload.get(
            'db_response_ms'
        )

        expected = {
            'status': 'OK',
            'db': 'ok',
            'query': 'ok',
        }

        failed = [
            key for key, value in expected.items()
            if payload.get(key) != value
        ]

        if result['http_code'] == 200 and not failed:
            result['status'] = 'OK'
        else:
            result['error'] = (
                'Controlli falliti: ' + ', '.join(failed)
            )

    except HTTPError as exc:
        result['http_code'] = exc.code
        result['error'] = f'HTTP {exc.code}'

    except (URLError, TimeoutError, ValueError) as exc:
        result['error'] = str(exc)[:200]

    finally:
        elapsed = time.monotonic() - started
        result['response_ms'] = round(elapsed * 1000, 1)

    return result


if __name__ == '__main__':
    result = check_web()
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['status'] == 'OK' else 1)
