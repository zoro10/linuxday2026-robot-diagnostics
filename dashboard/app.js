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

// Keep health measurements separate from the independent safety interlock.
// The simulator pauses after each V6 segment; a paused sensor is not failed.
let lastRobotSensorAssessment = null;

function validHealthScore(value) {
  return typeof value === "number" && Number.isFinite(value) &&
    value >= 0 && value <= 100;
}

function measuredSensorScore(data) {
  // Mirror the backend's 30 controller + 70 sensors weights, but only
  // snapshot measurements when all THREE sensor topics remain fresh.
  const t = data.topics || {};
  const expected = data.controller_expected || "pulse1_controller";
  if (data.nodes?.[expected] !== true) return null;
  let score = 30;
  for (const [name, weight, nominal] of [
    ["odometry", 20, 1], ["imu", 20, 10], ["lidar", 30, 5]
  ]) {
    const v = t[name];
    if (v?.status !== "OK" || !Number.isFinite(v.rate_hz) ||
        v.rate_hz < 0) return null;
    if (v.rate_hz >= nominal * 0.75) score += weight;
    else if (v.rate_hz >= nominal * 0.45) score += (weight / 2);
  }
  return score;
}

function sensorLabel(t, name, paused) {
  const v = t?.[name];
  if (v?.status === "OK") return "OK";
  if (paused && v?.status === "STALE") return "IN PAUSA";
  return v?.status || "NON DISP.";
}

function robotHealthPresentation(data, simulationState) {
  const robot = data.diagnostics?.robot_health || {};
  const telemetry = data.diagnostics?.telemetry || {};
  const safety = data.safety || {};
  const f = safety.fresh || {};
  const fresh = telemetry.status === "FRESH";
  const guard = f.guard === true ? safety.guard : null;
  const gf = f.geofence === true ? safety.geofence : null;
  const supervisor = f.supervisor === true ? safety.supervisor : null;
  const cmd = f.cmd === true ? safety.cmd : null;
  const paused = simulationState === "PAUSED";

  const zero = Array.isArray(cmd) && cmd.length === 2 &&
    cmd.every(x => Number.isFinite(x) && Math.abs(x) < 1e-8);
  const guardOK = guard?.guard_state === "BLOCKED" &&
    guard.emergency_latched === false && guard.v6_armed === true &&
    guard.cmd_vel_exclusive === true;
  const supervisorOK = supervisor?.state === "HEALTHY";
  const stopOK = guardOK && guard.nominal_stop_latched === true &&
    gf?.action === "COMPLETE" && gf.reason === "TARGET_REACHED" &&
    typeof guard.segment_id === "string" && guard.segment_id.length >= 8 &&
    guard.segment_id === gf.segment_id && gf.pause_ack === true;
  const readyOK = guardOK && guard.nominal_stop_latched === false &&
    gf?.action === "WAIT" && gf.reason === "NOT_STARTED";
  const moving = simulationState === "RUNNING";
  const guardRunning = guard?.guard_state === "AUTHORIZED" &&
    guard?.emergency_latched === false && guard?.v6_armed === true &&
    guard?.cmd_vel_exclusive === true;
  const gfRunning = gf?.action === "MONITOR" && gf?.ready_for_motion === true;
  const cmdRunning = Array.isArray(cmd) && cmd.length === 2 &&
    cmd.every(x => Number.isFinite(x));
  const gfOK = stopOK || readyOK;
  const safetyCount = [moving ? guardRunning : guardOK,
    moving ? gfRunning : gfOK, supervisorOK,
    moving ? cmdRunning : zero].filter(Boolean).length;
  const alarm = guard?.guard_state === "ABORT" ||
    guard?.emergency_latched === true || gf?.action === "PAUSE" ||
    supervisor?.state === "EMERGENCY_PAUSE";
  const sensorText = [
    "LiDAR " + sensorLabel(data.topics, "lidar", paused),
    "IMU " + sensorLabel(data.topics, "imu", paused),
    "Odom " + sensorLabel(data.topics, "odometry", paused)
  ].join("   ·   ");

  if (fresh && (readyOK || stopOK) && paused && supervisorOK && zero &&
      !guard.nominal_stop_latched) {
    // Reset begins a NEW motion session. No score from a prior run survives it.
    lastRobotSensorAssessment = null;
  }
  if (fresh && !alarm && simulationState === "RUNNING" &&
      validHealthScore(robot.score)) {
    lastRobotSensorAssessment = {score: robot.score, at: Date.now()};
  } else if (fresh && !alarm && paused &&
             (stopOK || readyOK) && supervisorOK && zero) {
    const observed = measuredSensorScore(data);
    if (observed !== null) {
      lastRobotSensorAssessment = {score: observed, at: Date.now()};
    }
  }
  const safetyLine = !fresh ? "Sicurezza non verificabile" :
    alarm ? "EMERGENZA V6 · MOVIMENTO BLOCCATO" :
    (paused && guardOK && supervisorOK && zero && gfOK)
      ? (stopOK ? "STOP CERTIFICATO" : "PRONTO ALL'AVVIO") +
        " · sicurezza 4/4" :
    moving && safetyCount === 4 ? "MOVIMENTO AUTORIZZATO · sicurezza 4/4" :
    "Sicurezza " + safetyCount + "/4 · da verificare";

  if (!fresh) return {
    score: "N/D", context: "TELEMETRIA ASSENTE", note: "Dati non aggiornati",
    safetyLine, sensorText, progress: 0, alarm: false
  };
  if (alarm) return {
    score: "ABORT", context: "EMERGENZA REALE", note: "Verificare Guard / Geofence / Supervisor",
    safetyLine, sensorText, progress: 0, alarm: true
  };
  if (paused) {
    if (!guardOK || !supervisorOK || !zero || !gfOK) return {
      score: "N/D", context: "STOP NON VERIFICATO",
      note: "Nessun punteggio assegnato senza sicurezza completa",
      safetyLine, sensorText, progress: 0, alarm: false
    };
    if (lastRobotSensorAssessment) {
      return {
        score: lastRobotSensorAssessment.score + "/100",
        context: "ULTIMA SALUTE MISURATA", note: "Pausa intenzionale · punteggio congelato, non live",
        safetyLine, sensorText, progress: lastRobotSensorAssessment.score, alarm: false
      };
    }
    // Show a truthful number rather than an invented 100/100 robot score.
    return {
      score: "4/4", context: "SICUREZZA VERIFICATA",
      note: "Sensori non ancora valutati dopo il reset",
      safetyLine, sensorText, progress: 100, alarm: false
    };
  }
  if (simulationState === "RUNNING" && validHealthScore(robot.score)) return {
    score: robot.score + "/100", context: "SALUTE MISURATA · LIVE",
    note: "Controller 30 pt · sensori 70 pt",
    safetyLine, sensorText, progress: robot.score, alarm: false
  };
  return {score: "N/D", context: "DATI INSUFFICIENTI",
    note: "In attesa di misure verificabili", safetyLine, sensorText,
    progress: 0, alarm: false};
}

// Enhance the existing card without touching its HTML or other dashboard cards.
function renderRobotHealthDetails(health) {
  const status = $("robot-status");
  if (!status || !status.parentNode || !document.createElement) return;
  let wrapper = $("robot-health-detail-v6");
  if (!wrapper) {
    wrapper = document.createElement("div");
    wrapper.id = "robot-health-detail-v6";
    wrapper.style.cssText = "margin-top:10px;padding-top:9px;border-top:1px solid #344761;font-size:11px;line-height:1.5;color:#b5c6dc;";
    const context = document.createElement("div");
    context.id = "robot-health-context-v6";
    context.style.cssText = "font-size:10px;letter-spacing:.06em;color:#8ccfff;font-weight:700;";
    const note = document.createElement("div");
    note.id = "robot-health-note-v6";
    const track = document.createElement("div");
    track.style.cssText = "height:5px;border-radius:8px;background:#2c3c52;margin:7px 0;overflow:hidden;";
    const bar = document.createElement("div");
    bar.id = "robot-health-bar-v6";
    bar.style.cssText = "height:100%;width:0%;background:#66b6ef;transition:width .3s;";
    track.appendChild(bar);
    const safety = document.createElement("div");
    safety.id = "robot-health-safety-v6";
    safety.style.cssText = "font-weight:700;";
    const sensors = document.createElement("div");
    sensors.id = "robot-health-sensors-v6";
    sensors.style.cssText = "font-size:10px;color:#9bacc3;margin-top:2px;";
    wrapper.append(context, note, track, safety, sensors);
    status.parentNode.appendChild(wrapper);
  }
  $("robot-health-context-v6").textContent = health.context;
  $("robot-health-note-v6").textContent = health.note;
  $("robot-health-safety-v6").textContent = health.safetyLine;
  $("robot-health-sensors-v6").textContent = health.sensorText;
  const bar = $("robot-health-bar-v6");
  bar.style.width = Math.max(0, Math.min(100, health.progress)) + "%";
  bar.style.background = health.alarm ? "#f87171" : "#66b6ef";
}

function render(data) {
  const d = data.diagnostics || {};
  const robot = d.robot_health || {};
  const platform = d.platform_health || {};
  const telemetry = d.telemetry || {};
  const fresh = telemetry.status === "FRESH";
  const sim = data.simulation?.state;
  const safety = data.safety || {};
  const sf = safety.fresh || {};
  const guard = sf.guard === true ? safety.guard : null;
  const geofence = sf.geofence === true ? safety.geofence : null;
  const simulationState = fresh &&
    (sim === "PAUSED" || sim === "RUNNING")
      ? sim : "UNKNOWN";
  state("simulation-status", simulationState);
  const safetyNote = guard?.guard_state === "ABORT" ||
    guard?.emergency_latched === true
    ? " · GUARD ABORT"
    : geofence?.action === "PAUSE"
      ? " · Geofence " + (geofence.reason || "PAUSE")
      : "";
  const health = robotHealthPresentation(data, simulationState);
  put("simulation-note", health.note + safetyNote);

  put("telemetry", fresh ? "LIVE" : "STALE");
  put("age", "Età telemetria: " +
    num(telemetry.age_seconds, 1) + " s");
  put("robot-score", health.score);
  $("robot-score").title = health.context + ": " + health.note;
  renderRobotHealthDetails(health);
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
  const speedFresh = t.odometry?.status === "OK";
  put("speed", simulationState === "PAUSED" || !speedFresh
    ? "— (ultima misura " + num(p.speed_m_s, 2) + " m/s)"
    : num(p.speed_m_s, 2) + " m/s");
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
    state("simulation-status", "UNKNOWN");
    put("simulation-note", "Stato Gazebo non verificabile");
    put("age", "Diagnostic API non raggiungibile");
    put("robot-score", "N/D");
    renderRobotHealthDetails({context:"OFFLINE", note:"API non raggiungibile",
      safetyLine:"Sicurezza non verificabile", sensorText:"Sensori non verificabili",progress:0,alarm:false});
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
