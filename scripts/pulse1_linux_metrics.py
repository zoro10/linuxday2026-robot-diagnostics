#!/usr/bin/env python3
"""Metriche Linux per il diagnostico PULSE-1."""

import json
import os
import shutil
import time


class LinuxMetrics:
    def __init__(self):
        self.previous_cpu = None

    def cpu_percent(self):
        with open('/proc/stat') as f:
            values = [
                int(x) for x in
                f.readline().split()[1:9]
            ]

        total = sum(values)
        idle = values[3] + values[4]
        previous = self.previous_cpu
        self.previous_cpu = (total, idle)

        if previous is None:
            return None

        delta_total = total - previous[0]
        delta_idle = idle - previous[1]

        if delta_total <= 0:
            return None

        usage = 100 * (1 - delta_idle / delta_total)
        return round(max(0, min(100, usage)), 1)

    def collect(self):
        memory = {}

        with open('/proc/meminfo') as f:
            for line in f:
                key, value = line.split(':', 1)
                memory[key] = int(value.strip().split()[0])

        mem_total = memory['MemTotal']
        mem_available = memory['MemAvailable']
        swap_total = memory['SwapTotal']
        swap_free = memory['SwapFree']

        disk = shutil.disk_usage('/')
        load1, load5, load15 = os.getloadavg()

        return {
            'cpu_percent': self.cpu_percent(),
            'cpu_cores': os.cpu_count(),
            'load_1m': round(load1, 2),
            'load_5m': round(load5, 2),
            'load_15m': round(load15, 2),
            'memory_percent': round(
                100 * (mem_total - mem_available) / mem_total, 1
            ),
            'memory_available_mib': round(
                mem_available / 1024
            ),
            'swap_percent': round(
                100 * (swap_total - swap_free) / swap_total, 1
            ) if swap_total else 0.0,
            'disk_percent': round(
                100 * disk.used / disk.total, 1
            ),
            'disk_free_gib': round(
                disk.free / (1024 ** 3), 1
            )
        }


if __name__ == '__main__':
    monitor = LinuxMetrics()
    monitor.collect()
    time.sleep(1)
    print(json.dumps(monitor.collect(), indent=2))
