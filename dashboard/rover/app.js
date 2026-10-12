"use strict";

const get = id => document.getElementById(id);
const show = (id, value) => { get(id).textContent = value; };
const valid = n => typeof n === "number" && Number.isFinite(n);

const SCALE = 115;
const ORIGIN_X = 150;
const ORIGIN_Y = 255;
const LIDAR_X = 34.34;
const trail = [];

function unavailable(reason) {
  get("rover").setAttribute("visibility", "hidden");
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

    if (telemetry?.status !== "FRESH" ||
        odom.status !== "OK" ||
        ![p.x, p.y, p.yaw_rad].every(valid)) {
      unavailable("TELEMETRIA NON DISPONIBILE");
      return;
    }

    const px = ORIGIN_X + p.x * SCALE;
    const py = ORIGIN_Y - p.y * SCALE;
    const angle = -p.yaw_rad * 180 / Math.PI;

    get("rover").setAttribute("visibility", "visible");
    get("rover").setAttribute(
      "transform", `translate(${px} ${py}) rotate(${angle})`
    );

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
    let motion = "FERMO";
    if (speed === null) motion = "UNKNOWN";
    else if (Math.abs(speed) > 0.02) motion = "IN MOVIMENTO";
    else if (laserOk && distance <= 1.2) motion = "OSTACOLO";

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
    console.error("PULSE-1 Live:", error);
  }
}

refresh();
setInterval(refresh, 2000);
