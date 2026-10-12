"use strict";

const $ = id => document.getElementById(id);
const put = (id, value) => { $(id).textContent = value; };
const num = (value, digits = 1) =>
  typeof value === "number" && Number.isFinite(value)
    ? value.toFixed(digits) : "—";
const percent = value => num(value) + (value == null ? "" : "%");

const metricIds = [
  "cpu", "ram", "disk", "load", "lidar", "imu",
  "odometry", "distance", "pos-x", "pos-y",
  "speed", "http", "db"
];

function state(id, value) {
  const s = String(value || "UNKNOWN");
  $(id).textContent = s;
  $(id).className = "state " + s.toLowerCase();
}

function problems(items) {
  const list = $("problems");
  list.replaceChildren();
  for (const message of items) {
    const li = document.createElement("li");
    li.textContent = message;
    list.appendChild(li);
  }
}

function resetMetrics() {
  for (const id of metricIds) put(id, "—");
}

function render(data) {
  const d = data.diagnostics || {};
  const robot = d.robot_health || {};
  const platform = d.platform_health || {};
  const telemetry = d.telemetry || {};
  const fresh = telemetry.status === "FRESH";

  put("telemetry", fresh ? "LIVE" : "STALE");
  put("age", "Età telemetria: " +
    num(telemetry.age_seconds, 1) + " s");
  put("robot-score", robot.score == null ? "—" : robot.score + "/100");
  put("platform-score", platform.score == null ? "—" : platform.score + "/100");
  state("robot-status", robot.status);
  state("platform-status", platform.status);
  put("api", "ONLINE");

  const issues = [
    ...(robot.problems || []),
    ...(platform.problems || [])
  ];
  problems(issues.length ? [...new Set(issues)] : ["Nessuna anomalia rilevata"]);

  if (!fresh) {
    resetMetrics();
    return;
  }

  const l = data.linux || {};
  const t = data.topics || {};
  const p = data.position || {};
  const w = data.web || {};

  put("cpu", percent(l.cpu_percent));
  put("ram", percent(l.memory_percent));
  put("disk", percent(l.disk_percent));
  put("load", num(l.load_1m, 2));

  for (const [id, key] of [
    ["lidar", "lidar"], ["imu", "imu"], ["odometry", "odometry"]
  ]) {
    const item = t[key] || {};
    put(id, (item.status || "UNKNOWN") +
      " · " + num(item.rate_hz, 1) + " Hz");
  }

  put("distance", num(data.lidar_front_distance_m, 2) + " m");
  put("pos-x", num(p.x, 3) + " m");
  put("pos-y", num(p.y, 3) + " m");
  put("speed", num(p.speed_m_s, 2) + " m/s");
  put("http", w.status === "OK"
    ? "HTTP " + w.http_code + " · " + num(w.response_ms) + " ms"
    : (w.status || "UNKNOWN"));
  put("db", w.db_response_ms == null
    ? "—" : num(w.db_response_ms, 2) + " ms");
}

async function refresh() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw Error("HTTP " + response.status);
    render(await response.json());
  } catch (error) {
    put("telemetry", "OFFLINE");
    put("age", "Diagnostic API non raggiungibile");
    put("robot-score", "—");
    put("platform-score", "—");
    state("robot-status", "UNKNOWN");
    state("platform-status", "UNKNOWN");
    put("api", "OFFLINE");
    resetMetrics();
    problems(["Connessione API fallita: " + error.message]);
  }
}

refresh();
setInterval(refresh, 3000);
