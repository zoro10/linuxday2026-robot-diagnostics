#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="diagnostics"

if tmux has-session -t "=$SESSION" 2>/dev/null; then
    echo "Sessione collector gia' attiva: $SESSION"
    exit 0
fi

if pgrep -f '[p]ulse1_collector.py' >/dev/null; then
    echo "Collector gia' attivo fuori tmux."
    echo "Non avvio una seconda istanza."
    exit 0
fi

tmux new-session -d -s "$SESSION" -n collector \
  "bash -lc 'source /opt/ros/jazzy/setup.bash; cd \"$ROOT\"; exec python3 scripts/pulse1_collector.py'"

echo "Collector avviato nella sessione tmux: $SESSION"
