#!/usr/bin/env python3
"""PULSE-1 S1: prova STOP nominale 20 cm, a blocchi ControlWorld."""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import pulse1_first_bounded_motion_v4 as v4

ROOT = Path(__file__).resolve().parent.parent
TRACE = Path('/tmp/pulse1_nominal_stop_v5.jsonl')
LOG = Path('/tmp/pulse1_nominal_stop_v5_trial.log')
MAX_X = 0.215            # abort prudenziale prima del limite 0.22
STOP_X = 0.20
MAX_WALL = 32.0         # startup + DRIVE 16 + STOPPING 2 + margine


def step_budget(x, dt):
    """Non chiedere mai un blocco che possa superare 0.215 m alla velocita' massima."""
    if not math.isfinite(x) or not math.isfinite(dt) or not 0 < dt <= 0.0011:
        raise RuntimeError(f'Posizione/step invalidi: x={x}, dt={dt}')
    remaining = MAX_X - x
    if remaining <= 0:
        raise RuntimeError(f'Limite V5 raggiunto: x={x:.5f}')
    # Mantieni blocchi sufficienti a produrre nuova odometria.
    # Il margine conservativo 1.5 e il limite MAX_X restano.
    n = min(50, int(remaining / (0.10 * dt * 1.5)))
    if n < 1:
        raise RuntimeError('Margine insufficiente per un altro step')
    return n


class ProbeV5(v4.Probe):
    def safety(self, first_x):
        gf = self.msg.get('geofence', {})
        if self.limits_breached or gf.get('action') == 'PAUSE':
            raise RuntimeError(f'Geofence PAUSE: {gf.get("reason")}')
        if self.pose is not None:
            x, y = self.pose
            if not all(math.isfinite(v) for v in (x, y)):
                raise RuntimeError('Odometria non finita')
            if x >= MAX_X or x < -0.05 or abs(y) >= 0.06:
                raise RuntimeError(f'Limite V5: x={x:.5f} y={y:.5f}')


def fresh(probe, key, limit=0.8):
    return probe.age(key) <= limit


def require_new_odometry(before, after, age):
    values = (before, after, age)
    if not all(
        type(v) in (int, float) and math.isfinite(v)
        for v in values
    ):
        raise RuntimeError("Timestamp odometria non valido")

    if after <= before:
        raise RuntimeError("Odometria non aggiornata dopo lo step")

    if not 0 <= age <= 0.5:
        raise RuntimeError("Nuova odometria scaduta")


def require_stopped(probe, *, geofence_wait=False):
    probe.require_paused()
    if probe.stats_stamp is None or time.monotonic() - probe.stats_stamp > 0.8:
        raise RuntimeError('Statistiche mondo obsolete')
    gf = probe.msg.get('geofence', {})
    guard = probe.msg.get('guard', {})
    if not fresh(probe, 'geofence') or not fresh(probe, 'guard'):
        raise RuntimeError('Heartbeat Geofence/Guard scaduto')
    if geofence_wait and not (gf.get('action') == 'WAIT'
                              and gf.get('reason') == 'NOT_STARTED'
                              and gf.get('armed') is True
                              and gf.get('drive_timeout_s') == 16.0):
        raise RuntimeError(f'Geofence V5 non pronta: {gf}')
    if (guard.get('permitted_linear_m_s') != 0
            or guard.get('permitted_angular_rad_s') != 0):
        raise RuntimeError('Guard non a zero')
    if (probe.cmd is None or probe.cmd_stamp is None
            or time.monotonic() - probe.cmd_stamp > 0.5
            or abs(probe.cmd[0]) > 1e-8 or abs(probe.cmd[1]) > 1e-8):
        raise RuntimeError('cmd_vel non-zero/scaduto')



def terminal_step_stopped(probe, error, before_odom, requested_n):
    """Riconosce solo uno STOP nominale che interrompe ControlWorld.

    Non autorizza nuovi step e non modifica il V4.
    """
    pattern = (
        r"Timeout ControlWorld: target=([0-9]+), "
        r"ACK=True, PAUSED_TARGET=False, "
        r"stats=[(]([0-9]+), True, ([0-9]+(?:[.][0-9]+)?)[)]"
    )

    match = re.fullmatch(pattern, str(error))

    if match is None:
        return False

    target = int(match.group(1))
    observed = int(match.group(2))
    reported_time = float(match.group(3))

    # Solo un blocco parzialmente completato, mai un overrun.
    if not (
        isinstance(requested_n, int)
        and 1 <= requested_n <= 50
        and 0 < target - observed <= requested_n
    ):
        return False

    if probe.stats is None or len(probe.stats) != 3:
        return False

    iterations, paused, simulated_time = probe.stats

    if iterations != observed or paused is not True:
        return False

    if (
        not math.isfinite(simulated_time)
        or abs(simulated_time - reported_time) > 1e-6
    ):
        return False

    # Deve esserci odometria nuova, successiva allo step.
    if probe.pose is None or probe.pos_stamp is None:
        return False

    if before_odom is None or probe.pos_stamp <= before_odom:
        return False

    x, y = probe.pose

    if not all(math.isfinite(v) for v in (x, y)):
        return False

    if not (STOP_X <= x < MAX_X and abs(y) < 0.06):
        return False

    if probe.limits_breached:
        return False

    trial = probe.msg.get("trial", {})
    gf = probe.msg.get("geofence", {})
    guard = probe.msg.get("guard", {})

    if not (
        trial.get("trial_phase") == "DONE"
        and trial.get("trial_reason") == "TARGET_REACHED"
        and trial.get("request_state") == "HOLD"
    ):
        return False

    if not (
        gf.get("action") == "COMPLETE"
        and gf.get("reason") == "TARGET_REACHED"
        and gf.get("trial_active") is False
    ):
        return False

    if guard.get("guard_state") not in (
        "BLOCKED", "STALE_REQUEST"
    ):
        return False

    # Evidenza di movimento prima dell'arresto.
    if not (
        probe.guard_authorized
        and probe.watchdog_monitor
        and probe.positive_cmd
    ):
        return False

    # Riusa tutte le verifiche originali:
    # Gazebo PAUSED, heartbeat freschi e comandi a zero.
    try:
        require_stopped(probe)
    except RuntimeError:
        return False

    return True

def run(probe):
    world = json.loads((ROOT / 'runtime/pulse1_world.json').read_text())
    if (world.get('scenario'), world.get('navigation_mode')) != ('s1', 'AVOID'):
        raise RuntimeError('Scenario non S1/AVOID')
    deadline = time.monotonic() + 8.0

    while time.monotonic() < deadline:
        probe.spin(0.10)

        gf = probe.msg.get('geofence', {})
        guard = probe.msg.get('guard', {})
        cmd = probe.cmd
        now = time.monotonic()

        if gf.get('action') == 'PAUSE' or probe.limits_breached:
            raise RuntimeError(
                f'Allarme Geofence nel preflight: {gf}'
            )

        if cmd is not None and (
            not all(math.isfinite(v) for v in cmd)
            or any(abs(v) > 1e-8 for v in cmd)
        ):
            raise RuntimeError('cmd_vel non-zero nel preflight')

        if guard and (
            guard.get('permitted_linear_m_s') != 0
            or guard.get('permitted_angular_rad_s') != 0
        ):
            raise RuntimeError('Guard non a zero nel preflight')

        ready = (
            probe.stats is not None
            and probe.stats[1] is True
            and probe.stats_stamp is not None
            and 0 <= now - probe.stats_stamp <= 0.8
            and len(probe.publishers(
                '/model/pulse1_proto/cmd_vel'
            )) == 1
            and not probe.publishers('/pulse1/avoid_request')
            and not probe.publishers('/pulse1/s1_trial_status')
            and gf.get('drive_timeout_s') == 16.0
            and gf.get('action') == 'WAIT'
            and gf.get('reason') == 'NOT_STARTED'
            and gf.get('armed') is True
            and guard.get('guard_state') in (
                'BLOCKED', 'STALE_REQUEST'
            )
            and fresh(probe, 'geofence', 0.5)
            and fresh(probe, 'guard', 0.5)
            and cmd is not None
            and probe.cmd_stamp is not None
            and 0 <= now - probe.cmd_stamp <= 0.5
        )

        if ready:
            require_stopped(probe, geofence_wait=True)
            print(
                'OK: Geofence, Guard e cmd_vel '
                'sincronizzati nel preflight',
                flush=True,
            )
            break

    else:
        raise RuntimeError(
            'Timeout sincronizzazione DDS: '
            f'geofence_age={probe.age("geofence"):.2f}s '
            f'guard_age={probe.age("guard"):.2f}s '
            f'cmd_presente={probe.cmd is not None}'
        )
    if probe.pose is not None and probe.pose[0] >= 0.015:
        raise RuntimeError('Posizione iniziale non ripristinata')
    baseline = probe.stats[0]
    probe.spin(0.5)
    if probe.stats[0] != baseline or probe.stats[1] is not True:
        raise RuntimeError('World non stabile in PAUSE')

    print('=== WARMUP SENZA TRIAL ===', flush=True)
    dt = None
    for i in range(2):
        require_stopped(probe, geofence_wait=True)
        dt = probe.step(1000)
        probe.spin(0.15)
        o = probe.msg.get('observer', {})
        sh = probe.msg.get('shadow', {})
        g = probe.msg.get('guard', {})
        ready = (o.get('state') == 'SENSOR_READY'
                 and sh.get('sensors_ok') is True
                 and g.get('avoid_sensor_ready') is True
                 and fresh(probe, 'observer', 0.5)
                 and fresh(probe, 'shadow', 0.4)
                 and fresh(probe, 'guard', 0.4)
                 and probe.pos_stamp is not None
                 and time.monotonic() - probe.pos_stamp <= 0.5)
        print(f'warmup {i+1}: ready={ready} x={probe.pose}', flush=True)
        if ready:
            break
    else:
        raise RuntimeError('Warmup V5 non pronto')
    if dt is None or not 0 < dt <= 0.0011:
        raise RuntimeError(f'Tempo fisica inatteso: {dt}')

    x0 = probe.pose[0]
    if not -0.03 <= x0 <= 0.015:
        raise RuntimeError(f'Posizione iniziale fuori limite: {x0}')
    probe.start_x = x0
    require_stopped(probe, geofence_wait=True)

    env = os.environ.copy()
    env['PULSE1_S1_TRIAL_ARM'] = 'YES'
    env['PULSE1_S1_V5_DRIVE_TIMEOUT'] = '16'
    log = LOG.open('w')
    try:
        probe.trial_proc = subprocess.Popen(
            [sys.executable, '-u', str(ROOT / 'scripts/pulse1_s1_trial_navigator.py'), '--armed'],
            cwd=str(ROOT), env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    finally:
        log.close()

    print(f'=== TRIAL V5 x0={x0:.5f} dt={dt:.6f} ===', flush=True)
    deadline = time.monotonic() + MAX_WALL
    target_seen = False
    for i in range(160):
        if time.monotonic() >= deadline:
            raise RuntimeError('Scaduto limite globale V5')
        if probe.trial_proc.poll() is not None:
            raise RuntimeError(f'Trial terminato: {probe.trial_proc.returncode}')
        probe.safety(x0)
        trial = probe.msg.get('trial', {})
        gf = probe.msg.get('geofence', {})
        reason = trial.get('trial_reason')
        if reason in ('TIME_LIMIT', 'STARTUP_TIMEOUT', 'REQUEST_DENIED',
                      'INVALID_POSE', 'ARM_LOST', 'UNSAFE_START_POSE',
                      'NAVIGATOR_SHUTDOWN'):
            raise RuntimeError(f'Trial abortito: {reason}')
        if trial.get('drive_timeout_s') not in (None, 16.0):
            raise RuntimeError('Trial non usa timeout V5')
        if reason == 'TARGET_REACHED' and trial.get('trial_phase') == 'DONE':
            target_seen = True
        if gf.get('action') in ('STOPPING', 'COMPLETE') or target_seen:
            break                     # Non inviare ulteriori step dopo STOPPING
        if gf.get('action') not in ('WAIT', 'MONITOR'):
            raise RuntimeError(f'Geofence inattesa: {gf}')
        if probe.pose is None or probe.pos_stamp is None:
            raise RuntimeError('Posizione assente')
        if time.monotonic() - probe.pos_stamp > 0.75:
            raise RuntimeError('Odometria scaduta prima dello step')
        if not fresh(probe, 'guard', 0.65) or not fresh(probe, 'geofence', 0.65):
            raise RuntimeError('Telemetria di sicurezza scaduta')
        before_odom = probe.pos_stamp
        n = step_budget(probe.pose[0], dt)

        try:
            probe.step(n, x0)
        except RuntimeError as exc:
            if not terminal_step_stopped(
                probe, exc, before_odom, n
            ):
                raise

            probe.log("CONTROL_TERMINAL_STOP", {
                "requested_steps": n,
                "observed_iterations": probe.stats[0],
                "x": probe.pose[0],
                "y": probe.pose[1],
                "reason": "TARGET_REACHED",
            })

            print(
                "OK V5: ControlWorld interrotto dallo "
                "STOP nominale confermato",
                flush=True,
            )

            target_seen = True
            break

        probe.spin(0.035)

        trial_after = probe.msg.get('trial', {})
        gf_after = probe.msg.get('geofence', {})

        terminal = (
            gf_after.get('action') in ('STOPPING', 'COMPLETE')
            or (
                trial_after.get('trial_phase') == 'DONE'
                and trial_after.get('trial_reason') == 'TARGET_REACHED'
            )
        )

        if not terminal:
            odom_deadline = time.monotonic() + 0.30

            while (
                (probe.pos_stamp is None
                 or probe.pos_stamp <= before_odom)
                and time.monotonic() < odom_deadline
            ):
                probe.spin(0.02)
                probe.safety(x0)

            require_new_odometry(
                before_odom,
                probe.pos_stamp,
                (
                    time.monotonic() - probe.pos_stamp
                    if probe.pos_stamp is not None else None
                ),
            )
        if i % 5 == 0:
            print(f'blocco={i+1} x={probe.pose[0]:.5f} '
                  f'trial={probe.msg.get("trial", {}).get("trial_phase")} '
                  f'gf={probe.msg.get("geofence", {}).get("action")}', flush=True)
    else:
        raise RuntimeError('Limite massimo blocchi V5')

    print('=== ATTESA STOPPING / COMPLETE, SENZA NUOVI STEP ===', flush=True)
    stopped_deadline = time.monotonic() + 2.5
    while time.monotonic() < stopped_deadline:
        probe.spin(0.05)
        probe.safety(x0)
        trial = probe.msg.get('trial', {})
        gf = probe.msg.get('geofence', {})
        target_seen |= (trial.get('trial_phase') == 'DONE'
                        and trial.get('trial_reason') == 'TARGET_REACHED')
        if gf.get('action') == 'COMPLETE' and target_seen:
            break
    else:
        raise RuntimeError(f'COMPLETE non confermato: gf={gf} trial={trial}')

    probe.spin(0.25)
    require_stopped(probe)
    trial = probe.msg.get('trial', {})
    gf = probe.msg.get('geofence', {})
    x = probe.pose[0] if probe.pose else None
    if not (target_seen and gf.get('action') == 'COMPLETE'
            and gf.get('reason') == 'TARGET_REACHED'
            and x is not None and STOP_X <= x < 0.22
            and probe.guard_authorized and probe.watchdog_monitor
            and probe.positive_cmd):
        raise RuntimeError('Evidenza STOP nominale incompleta')
    print(f'=== SUCCESSO: x={x:.5f}; GF COMPLETE; GUARD ZERO ===', flush=True)
    return True


def terminate_trial(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig, timeout_s in ((signal.SIGINT, 2), (signal.SIGTERM, 1), (signal.SIGKILL, 1)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--plan', action='store_true')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--self-test', action='store_true')
    args = p.parse_args()
    if args.self_test:
        assert step_budget(0.0, 0.001) == 50
        assert step_budget(0.190, 0.001) == 50
        assert step_budget(0.198, 0.001) == 50
        assert 0.212 + step_budget(0.212, 0.001) * 0.001 * 0.1 * 1.5 <= MAX_X + 1e-10
        print('OK: limiti step V5')
        return
    if args.plan:
        print('V5: S1/AVOID, timeout DRIVE 16 s, STOPPING 2 s, '
              'STOP 0.20 m, abort 0.215 m, geofence 0.22 m, '
              'blocchi <=50; nessun movimento in --plan')
        return
    if not args.execute or os.getenv('PULSE1_S1_V5_EXECUTE') != 'YES':
        raise SystemExit('Nessun movimento: servono --execute e PULSE1_S1_V5_EXECUTE=YES')
    v4.TRACE_PATH = TRACE
    rclpy.init()
    probe = ProbeV5()
    probe.trial_proc = None
    ok = False
    try:
        ok = run(probe)
    except Exception as exc:
        print(f'STOP V5: {exc}', flush=True)
    finally:
        terminate_trial(probe.trial_proc)
        try:
            v4.service('pause: true', timeout_s=8)
        except Exception as exc:
            print(f'ALLARME: PAUSE non confermato: {exc}', flush=True)
            ok = False
        probe.spin(1.5)
        gf = probe.msg.get('geofence', {})
        paused = probe.stats is not None and probe.stats[1] is True
        zero = (probe.cmd is not None
                and abs(probe.cmd[0]) < 1e-8
                and abs(probe.cmd[1]) < 1e-8
                and probe.msg.get('guard', {}).get('permitted_linear_m_s') == 0)
        print(f'FINALE: paused={paused} cmd_guard_zero={zero} '
              f'x={probe.pose} geofence={gf.get("action")}', flush=True)
        if not (paused and zero and gf.get('action') == 'COMPLETE'):
            ok = False
        probe.node.destroy_node()
        probe.trace.close()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
