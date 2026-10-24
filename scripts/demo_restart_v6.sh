#!/usr/bin/env bash
# Reset completo della sola simulazione PULSE-1 V6. Non avvia movimenti.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ $# -ne 1 || "$1" != "--reset" ]]; then
  echo "Uso: ./scripts/demo_restart_v6.sh --reset"
  echo "ATTENZIONE: azzera la posizione della simulazione; conserva le evidenze."
  exit 2
fi

for command in tmux gz ros2 curl timeout flock pgrep python3; do
  command -v "$command" >/dev/null || { echo "STOP: comando mancante: $command"; exit 1; }
done
[[ -r /opt/ros/jazzy/setup.bash ]] || { echo 'STOP: ROS 2 Jazzy non trovato'; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
set -u

mkdir -p runtime/evidence
# Il lock resta nel processo wrapper, NON nei nodi tmux/ROS avviati qui.
if [[ "${PULSE1_DEMO_LOCK_HELD:-}" != YES ]]; then
  exec flock -n --close "$ROOT/runtime/.pulse1_demo.lock" \
    env PULSE1_DEMO_LOCK_HELD=YES bash "${BASH_SOURCE[0]}" "$@"
fi

# Non interrompere un executor eventualmente ancora in moto.
if pgrep -f '^/usr/bin/python3 -u scripts/pulse1_v6_(single_segment|sequencer)[.]py' >/dev/null \
  || pgrep -f '^python3 -u scripts/pulse1_v6_(single_segment|sequencer)[.]py' >/dev/null; then
  echo 'STOP: executor V6 attivo; non eseguo il reset'
  exit 1
fi

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ARCHIVE="$ROOT/runtime/evidence/demo_reset_${STAMP}"
mkdir -p "$ARCHIVE"
GAZEBO_PATTERN='^gz sim -s --headless-rendering .*/pulse1_s1[.]sdf'

# Snapshot obbligatorio prima di distruggere una sessione preesistente.
if tmux has-session -t pulse1 2>/dev/null || pgrep -f "$GAZEBO_PATTERN" >/dev/null; then
  echo '=== ARCHIVIAZIONE STATO PRECEDENTE ==='
  if ! curl -fsS --max-time 8 http://127.0.0.1:8080/api/status > "$ARCHIVE/api_status.json"; then
    echo 'STOP: API non disponibile; non posso attestare lo stato precedente'
    exit 1
  fi
  if ! /usr/bin/python3 -c '
import json,sys
from datetime import datetime,timezone
s=json.load(open(sys.argv[1],encoding="utf-8"))
now=datetime.now(timezone.utc)
try:
 t=datetime.fromisoformat(s["timestamp_utc"].replace("Z","+00:00"))
except (KeyError,ValueError,TypeError):
 sys.exit("STOP: timestamp sicurezza non valido")
sec=(now-t).total_seconds()
v=s.get("safety") or {}; f=v.get("fresh") or {}
if not (0 <= sec <= 5 and s.get("simulation",{}).get("state")=="PAUSED"
        and v.get("cmd")==[0.0,0.0]
        and all(f.get(k) is True for k in ("guard","geofence","supervisor","cmd"))):
 sys.exit("STOP: PAUSED / motori zero / telemetria fresca NON verificati")
print("SNAPSHOT_QUIESCENT=OK")
' "$ARCHIVE/api_status.json"; then
    echo "SNAPSHOT=$ARCHIVE/api_status.json"
    exit 1
  fi
  if ! timeout 12s gz model -m pulse1_proto -p > "$ARCHIVE/gazebo_pose.txt" \
    || ! grep -Fq 'Name: pulse1_proto' "$ARCHIVE/gazebo_pose.txt"; then
    echo 'STOP: posa Gazebo non verificabile; reset rifiutato'
    exit 1
  fi
  cp -p runtime/pulse1_status.json "$ARCHIVE/collector_status.json" 2>/dev/null || true
  printf 'Reset manualmente autorizzato: %s\n' "$STAMP" > "$ARCHIVE/operator_reset.txt"
  echo "EVIDENZE=$ARCHIVE"
fi

# Terminazione della vecchia sessione; nessuna azione su Docker o sui log storici.
echo '=== ARRESTO VECCHIA SESSIONE ==='
if tmux has-session -t pulse1 2>/dev/null; then tmux kill-session -t pulse1; fi
sleep 2
mapfile -t old_gz < <(pgrep -f "$GAZEBO_PATTERN" || true)
for pid in "${old_gz[@]}"; do
  kill -INT "$pid" 2>/dev/null || true
done

for _ in $(seq 1 24); do
  if ! pgrep -f "$GAZEBO_PATTERN" >/dev/null; then break; fi
  sleep 0.5
done
if pgrep -f "$GAZEBO_PATTERN" >/dev/null; then
  echo 'STOP: Gazebo residuo ancora presente (nessun SIGKILL automatico)'
  exit 1
fi
if pgrep -f '^python3 (-u )?scripts/pulse1_(v6_ros_safety|v6_supervisor|collector)[.]py' >/dev/null; then
  echo 'STOP: processi V6 residui; controllarli prima di ricreare la sessione'
  exit 1
fi

# Avvio mondo simulato e spawn a coordinate iniziali.
echo '=== AVVIO GAZEBO S1 ==='
tmux new-session -d -s pulse1 -n gazebo \
  "bash -lc 'source /opt/ros/jazzy/setup.bash; exec gz sim -s --headless-rendering \"$ROOT/simulation/worlds/pulse1_s1.sdf\" -v 3'"

world_ready=0
for _ in $(seq 1 20); do
  if timeout 4s gz service -l 2>/dev/null | grep -Fxq '/world/empty/entity/system/add'; then
    world_ready=1; break
  fi
  sleep 1
done
if [[ $world_ready -ne 1 ]]; then
  echo 'STOP: mondo Gazebo non disponibile (nessun movimento eseguito)'
  exit 1
fi

echo '=== SPAWN PULSE-1 ==='
timeout 25s ros2 run ros_gz_sim create --world empty \
  --file "$ROOT/simulation/models/pulse1/model_demo2.sdf" \
  --name pulse1_proto -x 0 -y 0 -z 0.5
if ! timeout 12s gz model -m pulse1_proto -p | grep -Fq 'Name: pulse1_proto'; then
  echo 'STOP: modello PULSE-1 non verificato'
  exit 1
fi

# I bridge vengono avviati PRIMA dei sistemi di sicurezza.
echo '=== AVVIO BRIDGE E OBSERVER ==='
for mode in main stats rear lidar; do
  case "$mode" in
    main) name=bridge ;;
    stats) name=stats_bridge ;;
    rear) name=lidar_rear_bridge ;;
    lidar) name=lidar_bridge ;;
  esac
  tmux new-window -d -t pulse1 -n "$name" \
    "bash -lc 'cd \"$ROOT\" && source /opt/ros/jazzy/setup.bash && exec bash scripts/pulse1_bridge.sh $mode'"
done

tmux new-window -d -t pulse1 -n control_bridge \
  "bash -lc 'source /opt/ros/jazzy/setup.bash; exec ros2 run ros_gz_bridge parameter_bridge /world/empty/control@ros_gz_interfaces/srv/ControlWorld --ros-args -r __node:=pulse1_control_bridge'"
tmux new-window -d -t pulse1 -n avoid_observer \
  "bash -lc 'cd \"$ROOT\" && source /opt/ros/jazzy/setup.bash && exec python3 scripts/pulse1_avoid_observer.py --continuous'"

bridges_ready=0
for _ in $(seq 1 15); do
  topics="$(timeout 6s ros2 topic list 2>/dev/null || true)"
  services="$(timeout 6s ros2 service list 2>/dev/null || true)"
  ok=1
  for topic in /model/pulse1_proto/odometry /pulse1/imu /pulse1/scan /pulse1/scan_rear /pulse1/avoid_observer /world/empty/stats; do
    if ! grep -Fxq "$topic" <<< "$topics"; then ok=0; break; fi
  done
  if [[ $ok -eq 1 ]] && grep -Fxq /world/empty/control <<< "$services"; then
    bridges_ready=1; break
  fi
  sleep 2
done
if [[ $bridges_ready -ne 1 ]]; then
  echo 'STOP: topic o servizio ROS assenti; non avvio Guard/Geofence'
  exit 1
fi

# Sicurezza completa V6: NON disabilitare protezioni.
echo '=== AVVIO SICUREZZA V6 E COLLECTOR ==='
tmux new-window -d -t pulse1 -n v6_geofence \
  "bash -lc 'cd \"$ROOT\" && source /opt/ros/jazzy/setup.bash && exec env PULSE1_V6_GEOFENCE_ARM=YES PULSE1_V6_REARM_ENABLE=YES python3 -u scripts/pulse1_v6_ros_safety.py geofence --armed'"
tmux new-window -d -t pulse1 -n v6_guard \
  "bash -lc 'cd \"$ROOT\" && source /opt/ros/jazzy/setup.bash && exec env PULSE1_V6_MOTOR_ARM=YES PULSE1_V6_REARM_ENABLE=YES python3 -u scripts/pulse1_v6_ros_safety.py guard --armed'"
tmux new-window -d -t pulse1 -n collector \
  "bash -lc 'cd \"$ROOT\" && source /opt/ros/jazzy/setup.bash && exec python3 scripts/pulse1_collector.py'"

echo '=== ATTESA GUARD E GEOFENCE STABILI ==='
guard_ready=0
guard_streak=0

for _ in $(seq 1 30); do
  if curl -fsS --max-time 3 http://127.0.0.1:8080/api/status 2>/dev/null |
    /usr/bin/python3 -c '
import sys,json
try: d=json.load(sys.stdin)
except (ValueError,TypeError): sys.exit(1)
s=d.get("safety") or {}
g=s.get("guard") or {}
f=s.get("geofence") or {}
fresh=s.get("fresh") or {}
ok=(
 d.get("diagnostics",{}).get("telemetry",{}).get("status")=="FRESH"
 and d.get("simulation",{}).get("state")=="PAUSED"
 and g.get("guard_state")=="BLOCKED"
 and g.get("emergency_latched") is False
 and g.get("cmd_vel_exclusive") is True
 and g.get("v6_armed") is True
 and g.get("expected_next_segment_id") is None
 and f.get("action")=="WAIT"
 and f.get("reason")=="NOT_STARTED"
 and s.get("cmd")==[0.0,0.0]
 and all(fresh.get(k) is True for k in ("guard","geofence","cmd"))
)
sys.exit(0 if ok else 1)
'; then
    guard_streak=$((guard_streak+1))
    if [[ $guard_streak -ge 3 ]]; then
      guard_ready=1
      break
    fi
  else
    guard_streak=0
  fi
  sleep 1
done

if [[ $guard_ready -ne 1 ]]; then
  echo 'STOP: Guard/Geofence non pronti. Supervisor NON avviato.'
  exit 1
fi

echo 'OK: Guard e Geofence stabili'
echo '=== AVVIO SUPERVISOR ==='

tmux new-window -d -t pulse1 -n v6_supervisor \
  "bash -lc 'cd \"$ROOT\" && source /opt/ros/jazzy/setup.bash && exec env PULSE1_V6_SUPERVISOR_ARM=YES python3 -u scripts/pulse1_v6_supervisor.py --armed'"

ready=0
for _ in $(seq 1 30); do
  if curl -fsS --max-time 3 http://127.0.0.1:8080/api/status 2>/dev/null | /usr/bin/python3 -c '
import json,sys
try: d=json.load(sys.stdin)
except (ValueError,TypeError): sys.exit(1)
s=d.get("safety") or {};g=s.get("guard") or {}; f=s.get("geofence") or {}; v=s.get("supervisor") or {}; fresh=s.get("fresh") or {}
ok=(d.get("diagnostics",{}).get("telemetry",{}).get("status")=="FRESH"
 and d.get("simulation",{}).get("state")=="PAUSED"
 and g.get("guard_state")=="BLOCKED" and g.get("emergency_latched") is False
 and g.get("cmd_vel_exclusive") is True and g.get("expected_next_segment_id") is None
 and f.get("action")=="WAIT" and f.get("reason")=="NOT_STARTED"
 and v.get("state")=="HEALTHY" and s.get("cmd")==[0.0,0.0]
 and all(fresh.get(k) is True for k in ("guard","geofence","supervisor","cmd")))
sys.exit(0 if ok else 1)
'; then
    ready=1; break
  fi
  sleep 2
done
if [[ $ready -ne 1 ]]; then
  echo 'STOP: preflight V6 NON superato. Non eseguire segmenti.'
  echo 'Controllare: curl -s http://127.0.0.1:8080/api/status'
  exit 1
fi

echo '=== RESET COMPLETATO ==='
echo 'PRONTO=YES | GAZEBO=PAUSED | GUARD=BLOCKED | GEOFENCE=WAIT | SUPERVISOR=HEALTHY | CMD=0'
echo 'Nessun movimento eseguito. Usare: ./scripts/demo_segmenti_v6.sh 5'
