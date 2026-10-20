# PULSE-1 — Linux & Robot Diagnostics

**Linux Day 2026 | October 24, 2026**

PULSE-1 is an open-source Linux and robotics diagnostics project
built with ROS 2 Jazzy and Gazebo Harmonic.

The project combines a simulated mobile rover, sensor monitoring,
a live web dashboard, navigation diagnostics, Motor Guard,
and Geofence safety supervision.

## Technology

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- Odometry, IMU, front and rear LiDAR
- ROS 2 / Gazebo bridges
- Live diagnostics dashboard

## Validation status

**Verified as of October 19, 2026:**

- ROS 2 and Gazebo simulation operational
- Sensor monitoring and diagnostics operational
- Persistent ControlWorld stepping validated
- First bounded rover movement: 3.5 mm
- Motor Guard authorization demonstrated
- Geofence monitoring demonstrated during motion
- Final Gazebo PAUSED and motor commands zero
- 210 Python tests passed

**Pending validation:**

- Nominal STOP at 20 cm
- Emergency Geofence behavior at 22 cm
- Complete navigation scenarios
- Final demonstration and recovery tests

## Previous development documentation

The original VM diagnostics prototype documentation
is preserved below for historical reference.

---

# VM Diagnostics Demo

Progetto laboratorio per LinuxDay 2026.

## Obiettivo
Passare da monitoraggio a diagnostica su una VM Linux, usando:
- applicazione PHP custom
- MariaDB
- reverse proxy Nginx
- scaling dinamico dei frontend web
- futura integrazione con Netdata e score engine

## Topologia attuale
proxy -> N web -> db

## Componenti
- compose.yaml -> stack runtime
- web/ -> applicazione PHP
- db/init/ -> inizializzazione database
- proxy/nginx.conf -> configurazione proxy
- scripts/render_nginx.sh -> genera upstream Nginx
- scripts/scale_web.sh -> scala i container web e aggiorna il proxy

## Endpoint applicativi
- / -> pagina HTML leggibile
- /health.php -> check tecnico JSON
- /dbcheck.php -> check database
- /info.php -> info applicative

## Avvio base
docker compose up -d

## Scaling
Esempi:
./scripts/scale_web.sh 1
./scripts/scale_web.sh 2
./scripts/scale_web.sh 3

## Stato attuale
- runtime validato su VM target
- scaling validato
- round-robin validato
- progetto in preparazione per pubblicazione GitHub

## Prossimi passi
- ulteriore pulizia repo
- pubblicazione GitHub
- HTTPS/SSL
- Netdata
- score engine
