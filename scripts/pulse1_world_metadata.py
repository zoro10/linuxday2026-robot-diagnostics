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
    ["pgrep", "-af", "^gz sim -s -r --headless-rendering"],
    capture_output=True,
    text=True,
    check=False,
)

commands = [
    line.split(" ", 1)[1]
    for line in result.stdout.splitlines()
    if " " in line
]

if expected not in commands:
    raise SystemExit(
        f"ERRORE: Gazebo non sta eseguendo {world_file.name}"
    )

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
