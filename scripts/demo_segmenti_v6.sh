#!/usr/bin/env bash
# Esegue fino a N segmenti certificati, fermandosi al primo errore.
# Per la demo pubblica si ferma alla prima curva (prima di ADVANCE WP1).
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ $# -ne 1 || ! "$1" =~ ^[0-9]+$ ]] || (( 10#${1:-0} < 1 || 10#${1:-0} > 60 )); then
  echo 'Uso: ./scripts/demo_segmenti_v6.sh NUMERO  (1..60)'
  exit 2
fi
N=$((10#$1))
for command in flock curl gz timeout python3; do
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

# Solo una sessione completamente nuova, ferma, può utilizzare il primo segmento.
initial_session() {
  curl -fsS --max-time 5 http://127.0.0.1:8080/api/status 2>/dev/null |
  /usr/bin/python3 -c '
import json,sys
try: d=json.load(sys.stdin)
except (TypeError,ValueError): sys.exit(1)
s=d.get("safety") or {}; g=s.get("guard") or {}; gf=s.get("geofence") or {}; v=s.get("supervisor") or {}; fresh=s.get("fresh") or {}
ok=(d.get("diagnostics",{}).get("telemetry",{}).get("status")=="FRESH"
 and d.get("simulation",{}).get("state")=="PAUSED"
 and g.get("guard_state")=="BLOCKED" and g.get("emergency_latched") is False
 and g.get("expected_next_segment_id") is None and g.get("cmd_vel_exclusive") is True
 and gf.get("action")=="WAIT" and gf.get("reason")=="NOT_STARTED"
 and v.get("state")=="HEALTHY" and s.get("cmd")==[0.0,0.0]
 and all(fresh.get(k) is True for k in ("guard","geofence","supervisor","cmd")))
sys.exit(0 if ok else 1)
' || return 1
  # Non usare il primo segmento se il robot non è veramente all'origine.
  timeout 12s gz model -m pulse1_proto -p 2>/dev/null |
  /usr/bin/python3 -c '
import re,sys
s=sys.stdin.read()
m=re.search(r"Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[\s*([^\]]+)\]",s)
try:
 if not m: raise ValueError("no pose")
 x,y,z=map(float,m.group(1).split())
 ok=(abs(x)<0.03 and abs(y)<0.03 and z>=0)
except (ValueError,TypeError): ok=False
sys.exit(0 if ok else 1)
'
}

completed=0
for (( i=1; i<=N; i++ )); do
  log="runtime/evidence/v6_public_demo_$(date -u +%Y%m%dT%H%M%SZ)_${i}_$$.log"
  echo "=== SEGMENTO $i/$N ==="

  if initial_session; then
    echo 'PIANO: FIRST ADVANCE WP0, 0.1 m (sessione nuova)'
    if PULSE1_V6_EXECUTE=YES PYTHONPATH="$ROOT/scripts:${PYTHONPATH:-}" \
      /usr/bin/python3 -u scripts/pulse1_v6_single_segment.py --execute > "$log" 2>&1; then
      rc=0
    else
      rc=$?
    fi
    marker='FIRST'
  else
    plan="$(mktemp /tmp/pulse1_demo_plan.XXXXXX)"
    if PYTHONPATH="$ROOT/scripts:${PYTHONPATH:-}" \
      /usr/bin/python3 -u scripts/pulse1_v6_sequencer.py --plan > "$plan" 2>&1; then
      rc_plan=0
    else
      rc_plan=$?
    fi
    next="$(grep '^NEXT:' "$plan" | tail -n 1 || true)"
    if [[ $rc_plan -ne 0 ]] || ! grep -q '^VERIFIED STOP:' "$plan"; then
      echo 'STOP: nessuno STOP certificato o piano non valido; NESSUN riarmo.'
      tail -n 6 "$plan"
      rm -f "$plan"
      exit 1
    fi
    rm -f "$plan"

    if [[ "$next" == NEXT:\ TURN\ WAYPOINT:\ 2\ * ]]; then
      echo "STOP_PRIMA_CURVA_WP2: prossimo piano = $next"
      echo "SEGMENTI_ESEGUITI=$completed (richiesti $N)"
      exit 0
    elif [[ "$next" =~ ^NEXT:\ ADVANCE\ WAYPOINT:\ [01]\ LIMIT:\ 0[.](0[0-9]*[1-9][0-9]*|1(0*))$ ]] || \
         [[ "$next" =~ ^NEXT:\ TURN\ WAYPOINT:\ 1\ LIMIT:\ [0-9]+([.][0-9]+)?$ ]]; then
      echo "PIANO: $next"
    else
      echo "STOP: piano inatteso ($next); NESSUN riarmo"
      exit 1
    fi

    if PULSE1_V6_SEQUENCE_ARM=YES PYTHONPATH="$ROOT/scripts:${PYTHONPATH:-}" \
      /usr/bin/python3 -u scripts/pulse1_v6_sequencer.py --once > "$log" 2>&1; then
      rc=0
    else
      rc=$?
    fi
    marker='SEQUENCER'
  fi

  if [[ $rc -ne 0 ]] || ! grep -q 'SUCCESS V6: pose=' "$log" || \
     ! grep -q 'GF COMPLETE, GUARD NOMINAL STOP, PAUSED' "$log" || \
     { [[ "$marker" == SEQUENCER ]] && ! grep -q 'SUCCESS: SEGMENT + INDEPENDENT STOP' "$log"; }; then
    echo "STOP: SEGMENTO $i FALLITO (exit=$rc); NON RIPETERE / NON RIARMARE"
    grep -E 'ABORT V6:|STOP SEQUENCER:|RuntimeError:|SUCCESS V6:' "$log" | tail -n 8 || true
    echo "LOG=$log"
    echo "SEGMENTI_ESEGUITI=$completed"
    exit 1
  fi

  completed=$i
  grep 'SUCCESS V6: pose=' "$log" | tail -n 1
  echo "LOG=$log"
done

echo "SEGMENTI_ESEGUITI=$completed"
echo 'STOP certificato nell’ultimo log; nessun riarmo aggiuntivo.'
