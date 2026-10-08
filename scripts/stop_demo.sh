#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="pulse1"

GZ_PATTERN="^gz sim -s -r --headless-rendering ${ROOT}/simulation/worlds/pulse1[.]sdf -v 3$"

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

echo "[4/4] Chiusura tmux..."
tmux kill-session -t "$SESSION" 2>/dev/null || true

if pgrep -f "$GZ_PATTERN" >/dev/null; then
    echo "ERRORE: Gazebo ancora in esecuzione!"
    exit 1
fi

echo "Demo arrestata correttamente."
