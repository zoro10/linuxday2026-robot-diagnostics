# PULSE-1 — Linux & Robot Diagnostics

**Linux Day 2026 | October 24, 2026**

PULSE-1 is a Linux and robotics diagnostics project built with **ROS 2 Jazzy** and
**Gazebo Harmonic** on Ubuntu 24.04 LTS. It integrates a simulated mobile rover,
sensor observability, guarded motion trials, geofence supervision, and a web-based
diagnostic platform.

The project demonstrates *bounded, explicitly authorized motion*. The successful
trial below does **not** establish general-purpose autonomous navigation or
complete emergency-safety certification.

## Technology

- Ubuntu 24.04 LTS, Python 3
- ROS 2 Jazzy and Gazebo Harmonic
- Odometry, IMU, front and rear LiDAR
- ROS 2 / Gazebo bridges
- Docker, Nginx, MariaDB and a diagnostic HTTP API
- Browser-based rover and platform dashboards

## Architecture

### Simulation and safety

- **Gazebo** simulates the PULSE-1 rover and S1 environment.
- **ROS 2 bridges** publish simulation statistics, odometry and sensor data.
- **Avoid Observer** monitors sensed navigation conditions.
- **Approach Shadow** evaluates candidate navigation behavior without directly
  commanding the rover.
- **Trial Navigator** submits limited forward-motion requests when explicitly
  armed.
- **Motor Guard** gates motor commands.
- **Geofence V2** monitors the bounded motion and nominal stopping behavior.
- **Gazebo Supervisor** monitors safety-controller liveness.

### Diagnostics and dashboard

A ROS 2 collector writes runtime telemetry consumed by a diagnostic API. The
Docker web stack exposes Linux and robot health information, simulation state,
sensor freshness, and fault/recovery events.

**Important limitation:** while Gazebo is `PAUSED`, it may stop publishing sensor
samples. A fresh collector heartbeat does not mean the individual sensors are
fresh. The current dashboard's handling of paused simulations, last-known
positions and bridge detection is being refined.

## Validation results

**Verified locally on October 20, 2026**

- ROS 2 / Gazebo simulation and sensor reception verified.
- Bounded ControlWorld stepping verified.
- Motor Guard authorization observed during the bounded trial.
- Geofence nominal STOP observed and confirmed.
- Gazebo remained `PAUSED` with motor commands at zero after stopping.
- **289/289 Python tests passed** in the local test suite.

### Nominal STOP V5 — passed

| Measurement | Observed result |
| --- | --- |
| Nominal target | 200 mm |
| Final longitudinal position | **203.9 mm** |
| Error relative to target | +3.9 mm |
| V5 conservative limit | 215 mm |
| Trial Navigator | `DONE / TARGET_REACHED` |
| Geofence V2 | `COMPLETE / TARGET_REACHED` |
| Motor commands | Zero |
| Gazebo | `PAUSED` |
| V5 process exit code | **0** |

Validated commit: `bc077a8b272b2a83cfa5a06869db88a0e041340e`.

These are the results of **one successful integrated nominal STOP trial** in
the S1/AVOID bounded scenario. They must not be interpreted as proof of all
emergency, recovery or autonomous-navigation behaviors.

### Still pending

- Dedicated emergency Geofence behavior validation at the 220 mm boundary.
- Complete autonomous navigation scenarios.
- Final fault-injection and recovery demonstration.
- End-to-end demonstration of the live dashboard with correctly interpreted
  `PAUSED` state and sensor freshness.

## Dashboard and API

When the Docker stack and relevant ROS 2 services are running:

| Route | Purpose |
| --- | --- |
| `/dashboard/rover/` | Rover visualization |
| `/api/status` | Current diagnostic status |
| `/api/world` | S1 world metadata |
| `/api/events` | Fault and recovery history |

The collector is a separate ROS 2 process: starting Docker alone does **not**
ensure live rover telemetry. Do not infer the rover's current position from a
stale or absent odometry sample.

## Non-motion development checks

From the project root, in an environment with ROS 2 Jazzy and the required Python
dependencies installed:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q tests
python3 scripts/pulse1_nominal_stop_v5.py --plan
python3 scripts/pulse1_nominal_stop_v5.py --self-test
```

These commands do **not** request rover motion. Armed trial execution is not a
quick-start command: it requires explicit authorization and a verified Gazebo,
ROS 2 and safety-controller preflight.

## Documentation

- [PULSE-1 roadmap (October 13–22)](docs/PULSE1_Memo_Roadmap_13-22_ottobre_2026.md)
- [Scripts](docs/scripts.md)
- [Diagnostic platform v0.4](docs/v0.4-diagnostic-platform.md)
- [Legacy VM Diagnostics Demo](docs/legacy-vm-diagnostics.md)

## Project status and licensing

PULSE-1 is an experimental robotics diagnostics demonstration for Linux Day 2026.
The repository is public, but no software license has yet been selected or
published. Public availability should not be confused with an explicit open-source
license.
