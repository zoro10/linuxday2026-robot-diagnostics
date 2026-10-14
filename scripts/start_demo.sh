#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="pulse1"

SCENARIO="${1:-demo1}"

case "$SCENARIO" in
    demo1)
        WORLD_FILE="$ROOT/simulation/worlds/pulse1.sdf"
        ;;
    s1)
        WORLD_FILE="$ROOT/simulation/worlds/pulse1_s1.sdf"
        ;;
    *)
        echo "Scenario non riconosciuto: $SCENARIO"
        echo "Scenari disponibili: demo1, s1"
        exit 2
        ;;
esac

if [[ ! -f "$WORLD_FILE" ]]; then
    echo "Mondo Gazebo non trovato: $WORLD_FILE"
    exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Demo gia' attiva nella sessione $SESSION"
    exit 1
fi

if pgrep -af '^(/usr/bin/)?gz sim([[:space:]]|$)' >/dev/null; then
    echo "ATTENZIONE: Gazebo e' gia' in esecuzione."
    echo "Ferma prima la simulazione precedente."
    exit 1
fi

trap 'echo "Avvio fallito: arresto la sessione"; tmux kill-session -t "$SESSION" 2>/dev/null || true' ERR

echo "[1/4] Avvio Gazebo..."
tmux new-session -d -s "$SESSION" -n gazebo \
  "bash -c 'source /opt/ros/jazzy/setup.bash; exec gz sim -s -r --headless-rendering \"$WORLD_FILE\" -v 3'"

# Mantieni la finestra se Gazebo termina.
tmux set-window-option   -t "$SESSION:gazebo" remain-on-exit on

# Salva l'output di Gazebo su disco.
tmux pipe-pane -o   -t "$SESSION:gazebo"   "cat >> $ROOT/runtime/gazebo.log"

echo "[2/4] Attendo il mondo..."
ready=0
for i in {1..30}; do
    if gz service -l 2>/dev/null | grep -q '/world/empty/create'; then
        ready=1
        break
    fi
    sleep 1
done

if [[ "$ready" -ne 1 ]]; then
    echo "Gazebo non pronto."
    tmux kill-session -t "$SESSION"
    exit 1
fi

ros2 run ros_gz_sim create \
  --world empty \
  --file "$ROOT/simulation/models/pulse1/model.sdf" \
  --name pulse1_proto \
  -x 0 -y 0 -z 0.5

echo "[2b/4] Pubblicazione geometria scenario..."
python3 "$ROOT/scripts/pulse1_world_metadata.py" "$SCENARIO"

echo "[3/4] Avvio bridge ROS/Gazebo..."
tmux new-window -t "$SESSION" -n bridge \
  "bash \"$ROOT/scripts/pulse1_bridge.sh\" main"

echo "[3b/4] Avvio bridge LiDAR..."
tmux new-window -t "$SESSION" -n lidar_bridge \
  "bash \"$ROOT/scripts/pulse1_bridge.sh\" lidar"

echo "[4/4] Avvio controller..."
tmux new-window -t "$SESSION" -n controller \
  "bash -c 'source /opt/ros/jazzy/setup.bash; cd \"$ROOT\"; exec python3 scripts/pulse1_controller.py'"

trap - ERR
echo "Demo avviata. Sessione tmux: $SESSION"
tmux list-windows -t "$SESSION"
