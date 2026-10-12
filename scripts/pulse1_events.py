#!/usr/bin/env python3
"""Storico persistente dei guasti ROS 2."""

import json
from datetime import datetime, timezone
from pathlib import Path


class EventJournal:
    def __init__(self, path):
        self.path = Path(path)
        self.active = {}
        self.pending = {}

        if self.path.exists():
            for line in self.path.read_text().splitlines():
                try:
                    event = json.loads(line)
                    key = event['component']
                    if event['type'] == 'FAULT':
                        self.active[key] = event
                    elif event['type'] == 'RECOVERY':
                        self.active.pop(key, None)
                except (ValueError, KeyError, TypeError):
                    continue

    def update(self, nodes, topics, now=None):
        now = now or datetime.now(timezone.utc)
        observations = {}

        for name, present in nodes.items():
            if isinstance(present, bool):
                observations['node/' + name] = (
                    not present,
                    'Nodo ROS assente: ' + name
                )

        for name, data in topics.items():
            status = data.get('status')
            if status in ('OK', 'STALE'):
                observations['sensor/' + name] = (
                    status == 'STALE',
                    'Sensore non aggiornato: ' + name
                )

        emitted = []

        for key, (fault, message) in observations.items():
            already_faulty = key in self.active

            if fault == already_faulty:
                self.pending.pop(key, None)
                continue

            previous, count = self.pending.get(
                key, (None, 0)
            )
            count = count + 1 if previous == fault else 1

            if count < 3:
                self.pending[key] = (fault, count)
                continue

            self.pending.pop(key, None)

            event = {
                'timestamp_utc': now.isoformat(),
                'type': 'FAULT' if fault else 'RECOVERY',
                'component': key,
                'message': message if fault else 'Componente ripristinato'
            }

            if fault:
                self.active[key] = event
            else:
                started = datetime.fromisoformat(
                    self.active[key]['timestamp_utc']
                )
                event['duration_seconds'] = round(
                    max(0, (now - started).total_seconds()), 2
                )
                self.active.pop(key)

            with self.path.open('a', encoding='utf-8') as f:
                f.write(json.dumps(event, ensure_ascii=False) + '\n')

            emitted.append(event)

        return emitted
