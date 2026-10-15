"use strict";

const get = id => document.getElementById(id);
const show = (id, value) => { get(id).textContent = value; };
const valid = n => typeof n === "number" && Number.isFinite(n);

const SCALE = 115;
const ORIGIN_X = 150;
const ORIGIN_Y = 255;
const LIDAR_X = 34.34;
const trail = [];


async function refreshWorld() {
  const layer = get("world-obstacles");

  try {
    const response = await fetch(
      "/api/world", { cache: "no-store" }
    );

    if (!response.ok) {
      throw new Error("World API HTTP " + response.status);
    }

    const world = await response.json();

    if (!Array.isArray(world.obstacles) ||
        typeof world.scenario !== "string") {
      throw new Error("Metadati del mondo non validi");
    }

    const svgNS = layer.namespaceURI;
    const elements = [];

    for (const obstacle of world.obstacles) {
      const {x, y, size_x, size_y, yaw_rad} = obstacle;

      if (![x, y, size_x, size_y, yaw_rad].every(valid) ||
          size_x <= 0 || size_y <= 0) {
        continue;
      }

      const cx = ORIGIN_X + x * SCALE;
      const cy = ORIGIN_Y - y * SCALE;
      const w = size_x * SCALE;
      const h = size_y * SCALE;
      const angle = -yaw_rad * 180 / Math.PI;

      const group = document.createElementNS(svgNS, "g");
      group.setAttribute(
        "transform",
        `translate(${cx} ${cy}) rotate(${angle})`
      );

      const rect = document.createElementNS(svgNS, "rect");
      rect.setAttribute("x", -w / 2);
      rect.setAttribute("y", -h / 2);
      rect.setAttribute("width", w);
      rect.setAttribute("height", h);
      rect.setAttribute("fill", "#b45309");
      rect.setAttribute("stroke", "#fbbf24");
      rect.setAttribute("stroke-width", "2");

      group.appendChild(rect);
      elements.push(group);

      const label = document.createElementNS(svgNS, "text");
      label.setAttribute("x", cx);
      label.setAttribute("y", cy - h / 2 - 12);
      label.setAttribute("text-anchor", "middle");
      label.setAttribute("fill", "#fbbf24");
      label.setAttribute("font-size", "15");
      label.textContent = obstacle.name || "OSTACOLO";

      elements.push(label);
    }

    layer.replaceChildren(...elements);

    show(
      "world-label",
      `SCENARIO: ${world.scenario.toUpperCase()}`
    );

  } catch (error) {
    layer.replaceChildren();
    show("world-label", "SCENARIO NON DISPONIBILE");
    console.error("World metadata:", error);
  }
}

function setController(status) {
  const label = get("controller");
  label.textContent = status;
  label.className = status.toLowerCase();
}

function unavailable(reason) {
  get("rover").setAttribute("visibility", "hidden");
  get("beam").setAttribute("visibility", "hidden");
  show("feed", reason);
  for (const id of ["x", "y", "yaw", "speed", "distance", "motion"]) {
    show(id, "—");
  }
  show("motion", "UNKNOWN");
}

async function refresh() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error("HTTP " + response.status);

    const data = await response.json();
    const telemetry = data.diagnostics?.telemetry;
    const p = data.position || {};
    const odom = data.topics?.odometry || {};
    const lidar = data.topics?.lidar || {};

    const controllerKey =
    data.controller_expected || "pulse1_controller";
  const controller = data.nodes?.[controllerKey];
    const controllerStatus =
      telemetry?.status === "FRESH" &&
      typeof controller === "boolean"
        ? (controller ? "ONLINE" : "OFFLINE")
        : "UNKNOWN";

    setController(controllerStatus);

    const positionAvailable =
    [p.x, p.y, p.yaw_rad].every(valid);

  if (!positionAvailable) {
    unavailable("POSIZIONE NON DISPONIBILE");
    return;
  }

  const odomLive =
    telemetry?.status === "FRESH" &&
    odom.status === "OK";

  const px = ORIGIN_X + p.x * SCALE;
    const py = ORIGIN_Y - p.y * SCALE;
    const angle = -p.yaw_rad * 180 / Math.PI;

    get("rover").setAttribute("visibility", "visible");
    get("rover").setAttribute(
      "transform", `translate(${px} ${py}) rotate(${angle})`
    );

    if (!odomLive) {
    get("beam").setAttribute("visibility", "hidden");
    show("feed", "ULTIMA POSIZIONE · NON LIVE");
    show("x", p.x.toFixed(3) + " m");
    show("y", p.y.toFixed(3) + " m");
    show("yaw", (-angle).toFixed(1) + "°");
    show("speed", "—");
    show("distance", "—");
    show("motion", "DATI NON AGGIORNATI");
    return;
  }

  const previous = trail[trail.length - 1];
    if (!previous || Math.hypot(px - previous[0], py - previous[1]) > 4) {
      trail.push([px, py]);
      if (trail.length > 500) trail.shift();
      get("trail").setAttribute(
        "points", trail.map(point => point.join(",")).join(" ")
      );
    }

    const distance = data.lidar_front_distance_m;
    const laserOk = lidar.status === "OK" &&
                    valid(distance) && distance >= 0;

    get("beam").setAttribute("visibility", laserOk ? "visible" : "hidden");
    if (laserOk) {
      get("beam").setAttribute("x2", LIDAR_X + distance * SCALE);
    }

    const speed = valid(p.speed_m_s) ? p.speed_m_s : null;
  let motion = "UNKNOWN";

  if (controllerStatus === "OFFLINE") {
    motion = "CONTROLLER OFFLINE";
  } else if (controllerStatus !== "ONLINE") {
    motion = "CONTROLLER NON VERIFICATO";
  } else if (speed === null) {
    motion = "VELOCITA N/D";
  } else if (Math.abs(speed) > 0.02) {
    motion = "IN MOVIMENTO";
  } else if (!laserOk) {
    motion = "FERMO · LIDAR N/D";
  } else if (distance <= 1.2) {
    motion = "FERMO · OSTACOLO RILEVATO";
  } else {
    motion = "FERMO";
  }

  show("feed", "LIVE · " +
      (valid(telemetry.age_seconds)
        ? telemetry.age_seconds.toFixed(1) + " s" : "—"));
    show("x", p.x.toFixed(3) + " m");
    show("y", p.y.toFixed(3) + " m");
    show("yaw", (-angle).toFixed(1) + "°");
    show("speed", speed === null ? "—" : speed.toFixed(2) + " m/s");
    show("distance", laserOk ? distance.toFixed(2) + " m" : "N/D");
    show("motion", motion);

  } catch (error) {
    unavailable("API OFFLINE");
    setController("UNKNOWN");
    console.error("PULSE-1 Live:", error);
  }
}

refresh();
refreshWorld();

setInterval(refresh, 2000);
setInterval(refreshWorld, 5000);
