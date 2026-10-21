#!/usr/bin/env python3
"""PULSE-1: primo movimento simulato entro un budget millimetrico.

Solo su VM di simulazione ROS 2 Jazzy / Gazebo Harmonic, world empty.
Non disabilita nessun interblocco; se i prerequisiti non sono soddisfatti esce.
Codice predisposto offline, NON collaudato sul sistema dell'utente.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import WorldStatistics
from std_msgs.msg import String

ROOT = Path.home() / "vmdiag-app"
WORLD_CONTROL = "/world/empty/control"
MAX_X_M = 0.025        # arresto del test ben prima dei 0,22 m del geofence
GOAL_DELTA_M = 0.002  # consideriamo dimostrato il primo avanzamento da 2 mm
WALL_DEADLINE_S = 14  # include l'attesa dell'handshake; trial startup <=10s


def service(req, timeout_s=8):
    p = subprocess.run(
        ["timeout", f"{timeout_s}s", "gz", "service", "-s", WORLD_CONTROL,
         "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
         "--timeout", str(max(1000, (timeout_s-1)*1000)), "--req", req],
        text=True, capture_output=True, timeout=timeout_s + 1,
    )
    if p.returncode or "data: true" not in p.stdout:
        raise RuntimeError(f"Gazebo control {req}: {p.returncode}: {(p.stdout+p.stderr)[-180:]}")


class Probe:
    def __init__(self):
        self.node = rclpy.create_node("pulse1_first_bounded_motion_probe")
        self.msg = {}
        self.stamps = {}
        self.stats = None
        self.pose = None
        self.pos_stamp = None
        self.cmd = None
        self.cmd_stamp = None
        self.guard_authorized = False
        self.watchdog_monitor = False
        self.positive_cmd = False
        self.limits_breached = False
        self.subs = []
        for name, topic in (
            ("observer", "/pulse1/avoid_observer"),
            ("shadow", "/pulse1/s1_approach_shadow"),
            ("guard", "/pulse1/motor_guard_live"),
            ("geofence", "/pulse1/s1_geofence_watchdog"),
            ("trial", "/pulse1/s1_trial_status"),
            ("request", "/pulse1/avoid_request"),
        ):
            self.subs.append(self.node.create_subscription(
                String, topic, lambda m, key=name: self.on_json(key, m), 10))
        self.subs.append(self.node.create_subscription(
            WorldStatistics, "/world/empty/stats", self.on_stats, qos_profile_sensor_data))
        self.subs.append(self.node.create_subscription(
            Odometry, "/model/pulse1_proto/odometry", self.on_odom, qos_profile_sensor_data))
        self.subs.append(self.node.create_subscription(
            Twist, "/model/pulse1_proto/cmd_vel", self.on_cmd, 10))

    def on_json(self, key, m):
        try:
            d = json.loads(m.data)
            if not isinstance(d, dict):
                return
        except (ValueError, TypeError):
            return
        self.msg[key] = d
        self.stamps[key] = time.monotonic()
        if key == "guard" and d.get("guard_state") == "AUTHORIZED" and (d.get("permitted_linear_m_s") or 0) > 0:
            self.guard_authorized = True
        if key == "geofence":
            if d.get("action") == "MONITOR" and d.get("ready_for_motion") is True:
                self.watchdog_monitor = True
            if d.get("action") == "PAUSE":
                self.limits_breached = True

    def on_stats(self, m):
        self.stats = (m.iterations, m.paused, m.sim_time.sec + m.sim_time.nanosec * 1e-9)

    def on_odom(self, m):
        self.pose = (float(m.pose.pose.position.x), float(m.pose.pose.position.y))
        self.pos_stamp = time.monotonic()

    def on_cmd(self, m):
        self.cmd = (float(m.linear.x), float(m.angular.z))
        self.cmd_stamp = time.monotonic()
        if self.cmd[0] > 0:
            self.positive_cmd = True

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.03, max(0, end-time.monotonic())))

    def age(self, key):
        return time.monotonic()-self.stamps.get(key, -1e10)

    def publishers(self, topic):
        return self.node.get_publishers_info_by_topic(topic)

    def require_paused(self):
        if self.stats is None or self.stats[1] is not True:
            raise RuntimeError("Gazebo non PAUSED")

    def safety(self, first_x):
        if self.limits_breached:
            raise RuntimeError("Geofence PAUSE: allarme bloccante")
        if self.pose is not None:
            x, y = self.pose
            if not all(math.isfinite(z) for z in (x, y)):
                raise RuntimeError("Odometria non finita")
            if x >= min(MAX_X_M, first_x + MAX_X_M) or abs(y) >= 0.06:
                raise RuntimeError(f"Limite prudenziale raggiunto: x={x:.4f} y={y:.4f}")

    def step(self, n, first_x=None):
        self.require_paused()
        start_i, _, start_t = self.stats
        target = start_i+n
        p = subprocess.Popen(
            ["timeout", "9s", "gz", "service", "-s", WORLD_CONTROL,
             "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
             "--timeout", "7500", "--req", f"pause: true, multi_step: {n}"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            deadline = time.monotonic()+9
            while time.monotonic() < deadline:
                self.spin(0.03)
                if first_x is not None:
                    self.safety(first_x)
                if self.stats and self.stats[0] > target:
                    raise RuntimeError("Gazebo ha superato gli step autorizzati")
                if self.stats and self.stats[0] == target and self.stats[1] is True:
                    out, _ = p.communicate(timeout=2)
                    if p.returncode or "data: true" not in out:
                        raise RuntimeError(f"Gazebo non ha confermato gli step: {out[-120:]}")
                    return (self.stats[2]-start_t)/n
            raise RuntimeError("Timeout completamento step")
        finally:
            if p.poll() is None:
                p.kill()
                p.communicate()


def run(probe):
    print("=== PREFLIGHT: SOLO SIMULAZIONE ===", flush=True)
    if not (ROOT / "runtime/pulse1_world.json").is_file():
        raise RuntimeError("Metadata world mancanti")
    world = json.loads((ROOT / "runtime/pulse1_world.json").read_text())
    if (world.get("scenario"), world.get("navigation_mode")) != ("s1", "AVOID"):
        raise RuntimeError("Scenario diverso da S1/AVOID")

    deadline = time.monotonic()+8
    while time.monotonic()<deadline:
        probe.spin(0.10)
        if (probe.stats and probe.stats[1] is True
            and len(probe.publishers("/model/pulse1_proto/cmd_vel")) == 1
            and not probe.publishers("/pulse1/avoid_request")
            and probe.msg.get("geofence", {}).get("armed") is True
            and not probe.publishers("/pulse1/s1_trial_status")):
            break
    else:
        raise RuntimeError("DDS/Geofence/Trial preflight fallito")

    if probe.cmd is None or abs(probe.cmd[0])+abs(probe.cmd[1]) > 1e-6:
        raise RuntimeError("Cmd_vel iniziale non zero")
    initial_pose = probe.pose
    if initial_pose is not None and initial_pose[0] >= 0.015:
        raise RuntimeError("Posizione iniziale troppo vicina al limite prudenziale")

    print("=== RISCALDAMENTO SENSORI DISARMED ===", flush=True)
    for i in range(4):
        dt = probe.step(500)
        probe.spin(0.2)
        o, sh, g = (probe.msg.get(n, {}) for n in ("observer", "shadow", "guard"))
        ready = (o.get("state") == "SENSOR_READY"
                 and sh.get("sensors_ok") is True
                 and g.get("avoid_sensor_ready") is True
                 and probe.age("observer") < 0.5
                 and probe.age("shadow") < 0.4
                 and probe.age("guard") < 0.4
                 and probe.pose is not None
                 and probe.pos_stamp is not None
                 and time.monotonic() - probe.pos_stamp < 0.4)
        print(f"warmup {i+1}: observer={o.get('state')} shadow={sh.get('sensors_ok')} guard={g.get('avoid_sensor_ready')}", flush=True)
        if ready:
            break
    else:
        raise RuntimeError("Sensori non simultaneamente pronti, nessun armamento")

    first_x = probe.pose[0]
    if not -0.03 <= first_x <= 0.015:
        raise RuntimeError(f"X iniziale non sicura: {first_x:.4f}")
    if dt <= 0 or dt > 0.05:
        raise RuntimeError(f"Passo temporale inatteso: {dt}")

    print(f"=== ARMAMENTO TRIAL S1; x0={first_x:.4f} m, dt={dt:.6f} s/step ===", flush=True)
    log = open("/tmp/pulse1_first_bounded_motion.log", "w")
    env = os.environ.copy()
    env["PULSE1_S1_TRIAL_ARM"] = "YES"
    trial = subprocess.Popen(
        [sys.executable, "-u", str(ROOT / "scripts/pulse1_s1_trial_navigator.py"), "--armed"],
        cwd=str(ROOT), env=env, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic()+WALL_DEADLINE_S
        reached = False
        for i in range(160):
            if time.monotonic() >= deadline:
                print("TIMEOUT globale: arresto prudenziale", flush=True)
                break
            if trial.poll() is not None:
                print(f"Trial terminato: rc={trial.returncode}", flush=True)
                break
            probe.safety(first_x)
            t = probe.msg.get("trial", {})
            if t.get("trial_reason") in ("STARTUP_TIMEOUT", "NAVIGATOR_SHUTDOWN"):
                print(f"Trial terminale: {t.get('trial_reason')}", flush=True)
                break
            # Calcola un budget massimo di 5 mm teorici per richiesta,
            # limitato anche a 50 iterazioni; controlla X dopo ogni blocco.
            n = max(1, min(50, int(0.005 / (0.10*dt))))
            probe.step(n, first_x)
            probe.spin(0.04)
            probe.safety(first_x)
            if probe.pose is not None and probe.pose[0] >= first_x+GOAL_DELTA_M:
                reached = True
                break
        print("=== RISULTATO MOVIMENTO ===", flush=True)
        print(f"x0={first_x:.5f} x_final={probe.pose[0] if probe.pose else None}", flush=True)
        print(f"Trial phase={probe.msg.get('trial',{}).get('trial_phase')}", flush=True)
        print(f"Guard autorizzato osservato={probe.guard_authorized}", flush=True)
        print(f"Geofence MONITOR osservato={probe.watchdog_monitor}", flush=True)
        print(f"Comando positivo osservato={probe.positive_cmd}", flush=True)
        print(f"Avanzamento misurato >=2mm={reached}", flush=True)
        return reached and probe.guard_authorized and probe.watchdog_monitor and probe.positive_cmd
    finally:
        if trial.poll() is None:
            os.killpg(trial.pid, signal.SIGINT)
            try:
                trial.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(trial.pid, signal.SIGTERM)
                try:
                    trial.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    os.killpg(trial.pid, signal.SIGKILL)
                    trial.wait(timeout=1)
        log.close()


def main():
    rclpy.init()
    probe = Probe()
    valid = False
    try:
        valid = run(probe)
    except Exception as exc:
        print("STOP:", exc, flush=True)
    finally:
        try:
            service("pause: true", timeout_s=8)
        except Exception as exc:
            print("ALLARME: PAUSE service non confermato:", exc, flush=True)
        # Lascia tempo al Motor Guard di pubblicare un cmd_vel zero anche se
        # il Trial è cessato forzatamente (richiesta stale dopo 0,7 s).
        probe.spin(1.5)
        paused = probe.stats is not None and probe.stats[1] is True
        zero_cmd = (probe.cmd is not None
                    and abs(probe.cmd[0]) < 1e-8
                    and abs(probe.cmd[1]) < 1e-8
                    and probe.cmd_stamp is not None
                    and time.monotonic()-probe.cmd_stamp < 0.5)
        zero_guard = (probe.msg.get("guard", {}).get("permitted_linear_m_s") == 0
                      and probe.msg.get("guard", {}).get("permitted_angular_rad_s") == 0)
        print(f"=== FINALE: PAUSED={paused}, cmd_vel_zero={zero_cmd}, guard_zero={zero_guard} ===", flush=True)
        if not (paused and zero_cmd and zero_guard):
            valid = False
            print("ALLARME: arresto non completamente verificato", flush=True)
        probe.node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if valid else 1)


if __name__ == "__main__":
    main()
