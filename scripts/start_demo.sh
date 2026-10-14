#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="pulse1"

SCENARIO="${1:-demo1}"

case "$SCENARIO" in
    demo1)
        WORLD_FILE="$ROOT/simulation/worlds/pulse1.sdf"
        MODEL_FILE="$ROOT/simulation/models/pulse1/model.sdf"
        ;;
    s1)
        WORLD_FILE="$ROOT/simulation/worlds/pulse1_s1.sdf"
        MODEL_FILE="$ROOT/simulation/models/pulse1/model_demo2.sdf"
        ;;
    *)
        echo "Scenario non riconosciuto: $SCENARIO"
        echo "Scenari disponibili: demo1, s1"
        exit 2
        ;;
esac

if (( $# > 2 )); then
    echo "Uso: $0 [demo1 | s1 [STOP|AVOID]]" >&2
    exit 2
fi

if [[ "$SCENARIO" == "s1" ]]; then
    NAV_MODE="${2:-STOP}"

    case "$NAV_MODE" in
        STOP|AVOID) ;;
        *)
            echo "Modalita non valida: $NAV_MODE" >&2
            exit 2
            ;;
    esac

    if [[ ! -f "$ROOT/scripts/pulse1_navigation_shadow.py" ]]; then
        echo "Navigatore shadow non trovato" >&2
        exit 1
    fi
elif (( $# > 1 )); then
    echo "Demo 1 non accetta una modalita di missione" >&2
    exit 2
fi

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
  --file "$MODEL_FILE" \
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

if [[ "$SCENARIO" == "s1" ]]; then
    echo "[3c/4] Avvio bridge LiDAR posteriore..."
    tmux new-window -t "$SESSION" -n lidar_rear_bridge \
      "bash \"$ROOT/scripts/pulse1_bridge.sh\" rear"
fi

echo "[4/4] Avvio controller..."
tmux new-window -t "$SESSION" -n controller \
  "bash -c 'source /opt/ros/jazzy/setup.bash; cd \"$ROOT\"; exec python3 scripts/pulse1_controller.py'"

if [[ "$SCENARIO" == "s1" ]]; then
    echo "[4b/4] Avvio navigatore shadow: $NAV_MODE"

    tmux new-window -t "$SESSION" -n nav_shadow \
      "bash -c 'source /opt/ros/jazzy/setup.bash; cd \"$ROOT\"; exec python3 scripts/pulse1_navigation_shadow.py --mode \"$NAV_MODE\" --samples 0'"
fi

trap - ERR
echo "Demo avviata. Sessione tmux: $SESSION"
tmux list-windows -t "$SESSION"
