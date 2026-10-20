#!/usr/bin/env python3
"""Esporta la geometria SDF per la sola dashboard."""

import json
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

WORLDS = {
    "demo1": "pulse1.sdf",
    "s1": "pulse1_s1.sdf",
}

if len(sys.argv) != 2 or sys.argv[1] not in WORLDS:
    raise SystemExit("Uso: pulse1_world_metadata.py demo1|s1")

scenario = sys.argv[1]
world_file = ROOT / "simulation/worlds" / WORLDS[scenario]

expected = (
    "gz sim -s -r --headless-rendering "
    f"{world_file} -v 3"
)

result = subprocess.run(
    ["pgrep", "-af", "^gz sim -s (-r )?--headless-rendering"],
    capture_output=True,
    text=True,
    check=False,
)

commands = [
    line.split(" ", 1)[1]
    for line in result.stdout.splitlines()
    if " " in line
]

expected_paused = expected.replace("gz sim -s -r ", "gz sim -s ", 1)

# Accetta anche il percorso relativo alla root del progetto.
relative_world = world_file.relative_to(ROOT).as_posix()

expected_relative = expected.replace(
    str(world_file), relative_world, 1
)

expected_paused_relative = expected_paused.replace(
    str(world_file), relative_world, 1
)

valid_commands = (
    [expected],
    [expected_paused],
    [expected_relative],
    [expected_paused_relative],
)

if commands not in valid_commands:
    raise SystemExit(
        f"ERRORE: Gazebo non sta eseguendo {world_file.name}"
    )

# Con percorso relativo, verifica anche la directory di Gazebo.
if commands in ([expected_relative], [expected_paused_relative]):
    pid = result.stdout.splitlines()[0].split(" ", 1)[0]
    process_cwd = (Path("/proc") / pid / "cwd").resolve()

    if process_cwd != ROOT:
        raise SystemExit(
            "ERRORE: directory Gazebo incompatibile con S1"
        )

if commands in ([expected_paused], [expected_paused_relative]):
    status = subprocess.run(
        ["timeout", "8s", "gz", "topic", "-e",
         "-t", "/world/empty/stats", "-n", "1"],
        capture_output=True, text=True,
    )
    if status.returncode != 0 or not re.search(
        r"(?m)^paused:\s*true\s*$", status.stdout
    ):
        raise SystemExit("ERRORE: Gazebo non confermato PAUSED")

text = world_file.read_text()

# Il mondo originale contiene un commento XML non valido.
text = re.sub(r"<!--.*?-->", "", text, flags=re.S)

root = ET.fromstring(text)
world = root.find("world")

if world is None:
    raise SystemExit("Elemento world assente")

obstacles = []

for model in world.findall("model"):
    name = model.get("name", "")

    if "obstacle" not in name:
        continue

    pose = [
        float(value)
        for value in model.findtext(
            "pose", "0 0 0 0 0 0"
        ).split()
    ]

    dimensions = model.findtext(
        "./link/collision/geometry/box/size"
    )

    if dimensions is None:
        raise SystemExit(
            f"Geometria non supportata: {name}"
        )

    size = [float(value) for value in dimensions.split()]

    obstacles.append({
        "name": name,
        "x": pose[0],
        "y": pose[1],
        "yaw_rad": pose[5],
        "size_x": size[0],
        "size_y": size[1],
    })

payload = {
    "scenario": scenario,
    "world": world.get("name"),
    "source": world_file.name,
    "obstacles": obstacles,
}

destination = ROOT / "runtime/pulse1_world.json"

with tempfile.NamedTemporaryFile(
    mode="w",
    encoding="utf-8",
    dir=destination.parent,
    prefix=".pulse1_world_",
    delete=False,
) as stream:
    temporary = Path(stream.name)
    json.dump(payload, stream, indent=2)

try:
    os.chmod(temporary, 0o644)
    os.replace(temporary, destination)
finally:
    temporary.unlink(missing_ok=True)

print(json.dumps(payload, indent=2))
