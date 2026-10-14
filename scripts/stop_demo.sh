#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="pulse1"

GZ_PATTERN="^gz sim -s -r --headless-rendering ${ROOT}/simulation/worlds/pulse1(_s1)?[.]sdf -v 3$"

set +u
source /opt/ros/jazzy/setup.bash
set -u

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[1/4] Arresto controller..."
    tmux kill-window -t "$SESSION:controller" 2>/dev/null || true

    echo "[2/4] Invio velocita' zero..."
    timeout 5s ros2 topic pub --once \
      /model/pulse1_proto/cmd_vel \
      geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}' \
      >/dev/null 2>&1 || true
fi

echo "[3/4] Arresto Gazebo con SIGINT..."

if pgrep -f "$GZ_PATTERN" >/dev/null; then
    pkill -INT -f "$GZ_PATTERN" || true

    for i in {1..12}; do
        if ! pgrep -f "$GZ_PATTERN" >/dev/null; then
            break
        fi
        sleep 1
    done
fi

if pgrep -f "$GZ_PATTERN" >/dev/null; then
    echo "Gazebo ancora attivo: invio SIGTERM..."
    pkill -TERM -f "$GZ_PATTERN" || true
    sleep 2
fi

if pgrep -f "$GZ_PATTERN" >/dev/null; then
    echo "[3b/4] Gazebo resistente: arresto forzato verificato..."

    python3 - "$ROOT" <<'PULSE1_FORCE_STOP'
import os
import signal
import sys
from pathlib import Path

root = Path(sys.argv[1])

allowed_worlds = {
    str(root / "simulation/worlds/pulse1.sdf"),
    str(root / "simulation/worlds/pulse1_s1.sdf"),
}

for proc in Path("/proc").iterdir():
    if not proc.name.isdecimal():
        continue

    try:
        args = [
            part.decode()
            for part in (proc / "cmdline").read_bytes().split(b"\0")
            if part
        ]
    except (OSError, UnicodeDecodeError):
        continue

    if (
        len(args) != 8
        or args[:5] != [
            "gz", "sim", "-s", "-r", "--headless-rendering"
        ]
        or args[5] not in allowed_worlds
        or args[6:] != ["-v", "3"]
    ):
        continue

    try:
        os.kill(int(proc.name), signal.SIGKILL)
        print(f"SIGKILL inviato a Gazebo PID {proc.name}")
    except ProcessLookupError:
        pass
PULSE1_FORCE_STOP

    sleep 2
fi

echo "[4/4] Chiusura tmux..."
tmux kill-session -t "$SESSION" 2>/dev/null || true

if pgrep -f "$GZ_PATTERN" >/dev/null; then
    echo "ERRORE: Gazebo ancora in esecuzione!"
    exit 1
fi

# Nessun mondo attivo: non mostrare una mappa precedente.
rm -f "$ROOT/runtime/pulse1_world.json"

echo "Demo arrestata correttamente."
