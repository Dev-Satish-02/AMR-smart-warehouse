import { MapView } from "./map.js";
import { Editor, dialog } from "./editor.js";
import { loadCatalog, objectType } from "./catalog.js";
import { displayColor, STATUS, ACTIVITY, fmtDuration } from "./format.js";
import { MissionsPage, FleetPage, AlertsPage, ReportsPage, BenchmarkPage, STRATEGY_LABEL, initTooltips, hideTooltip, renderAlertBadge } from "./pages.js";

const $ = (sel) => document.querySelector(sel);

const SPEEDS = [0.5, 1, 2, 4, 8];
const RENDER_DELAY_MS = 140; // render slightly in the past so we can interpolate

// status -> [palette class, icon, label]
const MISSION_PHASE = { assigned: "To pickup", loading: "Loading", to_dropoff: "Delivering", unloading: "Unloading" };

const EVENT_ICON = {
  conflict: "s-yield", deadlock: "s-backoff", backoff: "s-backoff", reroute: "s-reroute",
  arrival: "s-arrived", resume: "s-moving", route: "s-turning", system: "s-idle", error: "s-error",
  mission: "s-mission", battery: "s-charging",
};

const app = {
  ws: null,
  layout: null,
  layoutName: null,
  buffer: [],          // [{ at: ms, state }]
  latest: null,
  running: false,
  speed: 1,
  selected: null,
  events: [],
  stationsByCell: new Map(),
};

const map = new MapView($("#map"));
map.onSelect = (id) => select(id === app.selected ? null : id);

let mode = "live";
const editor = new Editor({
  toast,
  onSaved: async (file) => { await loadLayoutList(); syncSelect(); },
  onOpened: () => syncSelect(),
  onRun: (file) => { setMode("live"); send("load", { name: file }); },
});

// ------------------------------------------------------------------ socket

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  app.ws = ws;
  ws.onopen = () => setConn(true);
  ws.onclose = () => { setConn(false); setTimeout(connect, 1200); };
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "layout") onLayout(msg);
    else if (msg.type === "state") onState(msg);
    else if (msg.type === "error") toast(msg.message);
  };
}

function send(cmd, extra = {}) {
  if (app.ws && app.ws.readyState === WebSocket.OPEN) app.ws.send(JSON.stringify({ cmd, ...extra }));
}

function setConn(ok) {
  const node = $("#conn");
  node.classList.toggle("ok", ok);
  node.classList.toggle("bad", !ok);
  node.querySelector(".conn-label").textContent = ok ? "Live" : "Reconnecting…";
}

// ------------------------------------------------------------------ layout

function onLayout(msg) {
  app.layout = msg.layout;
  app.layoutName = msg.name;
  app.benchmark = msg.benchmark || null;
  app.buffer = [];
  app.events = [];
  app.selected = null;
  map.selected = null;
  app.stationsByCell = new Map(msg.layout.stations.map((s) => [`${s.x},${s.y}`, s]));
  map.setLayout(msg.layout);

  $("#layout-title").textContent = app.benchmark ? `Benchmark replay: ${app.benchmark.title}` : msg.layout.name;
  const lanes = msg.layout.rows.join("").replace(/[^+<>^v]/g, "").length;
  $("#layout-meta").textContent =
    `${msg.layout.width} × ${msg.layout.height} m grid · ${lanes} lane cells · ` +
    `${msg.layout.stations.length} stations · ${msg.layout.robots.length} robots` +
    (app.benchmark ? ` · seed ${app.benchmark.seed}, same orders as the benchmark` : "");
  syncSelect();

  const errors = msg.issues.filter((i) => i.severity === "error");
  const issues = $("#issues");
  issues.hidden = errors.length === 0;
  issues.innerHTML = errors.length
    ? `<strong>This layout can't be simulated yet.</strong><ul>${errors.map((i) => `<li>${escapeHtml(i.message)}</li>`).join("")}</ul>`
    : "";
  renderLegend();
}

async function loadLayoutList() {
  const res = await fetch("/api/layouts");
  const data = await res.json();
  editor.knownFiles = data.layouts.map((l) => l.name);
  const select = $("#layout-select");
  select.innerHTML = data.layouts
    .map((l) => `<option value="${l.name}">${escapeHtml(l.title)}</option>`)
    .join("") + `<option value="" hidden>Unsaved layout</option>`;
  if (!app.layoutName && data.active) app.layoutName = data.active;
  syncSelect();
}

// The layout dropdown follows the current mode: in Live it picks what runs,
// in the Editor it picks what you edit.
function syncSelect() {
  const select = $("#layout-select");
  select.querySelector("option[data-benchmark]")?.remove();
  if (mode !== "editor" && app.benchmark) {
    const option = document.createElement("option");
    option.value = app.layoutName;
    option.dataset.benchmark = "1";
    option.textContent = `Benchmark: ${app.benchmark.title}`;
    select.append(option);
  }
  select.value = mode === "editor" ? (editor.file || "") : (app.layoutName || "");
}

// ------------------------------------------------------------------ modes

const MODES = ["live", "missions", "fleet", "alerts", "reports", "benchmark", "editor"];

function modeFromHash() {
  const hash = location.hash.replace("#", "");
  if (hash === "overview") return "live";
  return MODES.includes(hash) ? hash : "live";
}

async function setMode(next) {
  if (!MODES.includes(next)) next = "live";
  if (next === mode) return;
  pages[mode]?.hide();
  hideTooltip();
  mode = next;
  document.body.dataset.mode = mode;
  history.replaceState(null, "", `#${mode === "live" ? "overview" : mode}`);
  for (const node of document.querySelectorAll("[data-view]")) node.hidden = node.dataset.view !== mode;
  for (const tab of document.querySelectorAll(".tab")) {
    const active = tab.dataset.mode === mode;
    tab.classList.toggle("active", active);
    tab.setAttribute("aria-selected", active);
  }
  if (mode === "editor") {
    // A benchmark replay is not a saved layout: open the first saved one instead.
    const file = app.benchmark ? editor.knownFiles?.[0] : app.layoutName;
    if (!editor.loaded && file) await editor.open(file);
    else editor.map.fit();
  } else if (mode === "live") {
    map.fit();
  }
  pages[mode]?.show();
  syncSelect();
}

// ------------------------------------------------------------------ state

function onState(msg) {
  app.running = msg.running;
  app.speed = msg.speed;
  app.latest = msg.state;
  const now = performance.now();
  if (msg.reset_events) { app.buffer = []; app.events = []; }
  app.buffer.push({ at: now, state: msg.state });
  if (app.buffer.length > 30) app.buffer.shift();

  map.setState(msg.state);
  if (msg.events.length) addEvents(msg.events, !msg.reset_events);
  renderTransport();
  renderStatus(msg.state);
  renderStrategy(msg.strategy);
  renderKpis(msg.state.metrics, msg.state.fleet);
  renderFleet(msg.state);
  renderMissions(msg.state.fleet);
  renderEstop(msg.state);
  renderAlertBadge(msg.state.alerts);
  if (mode === "fleet") pages.fleet.render(msg.state);
  if (mode === "alerts") pages.alerts.render(msg.state);
}

// ------------------------------------------------------------------ E-stop

function renderEstop(state) {
  const engaged = !!state.estop;
  const button = $("#btn-estop");
  button.classList.toggle("engaged", engaged);
  button.querySelector("span").textContent = engaged ? "E-STOP ON" : "E-STOP";
  button.title = engaged ? "E-stop engaged: release it from the red banner" : "Fleet-wide emergency stop (all robots halt immediately)";
  $("#estop-banner").hidden = !engaged;
  if (engaged) {
    $("#estop-text").textContent = `All robots stopped for ${fmtDuration(state.time - state.estop_since)}. Orders keep queueing; nothing is dispatched until release.`;
  }
}

function releaseEstop() {
  if (window.confirm("Release the E-stop? Robots will resume their tasks.")) send("estop", { value: false });
}

// ------------------------------------------------------------------ theme

const THEME_KEY = "nexus.theme";

function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  try { localStorage.setItem(THEME_KEY, theme); } catch { /* storage unavailable */ }
  // Robot colours and SVG content are theme-dependent: redraw.
  if (app.layout) {
    map.setLayout(app.layout, { fit: false });
    map.robotNodes.forEach((node) => node.group.remove());
    map.robotNodes.clear();
    if (app.latest) map.setState(app.latest);
    renderLegend();
  }
  if (editor.loaded) editor.render();
  if (app.latest) {
    renderFleet(app.latest);
    renderMissions(app.latest.fleet);
    pages[mode]?.render(app.latest, true);
  }
}

function initTheme() {
  let theme = "dark";
  try { theme = localStorage.getItem(THEME_KEY) || "dark"; } catch { /* storage unavailable */ }
  document.documentElement.dataset.theme = theme;
  $("#btn-theme").addEventListener("click", () => applyTheme(document.documentElement.dataset.theme === "light" ? "dark" : "light"));
  // Reports print on white whatever the screen theme.
  let before = null;
  window.addEventListener("beforeprint", () => { before = document.documentElement.dataset.theme; if (before !== "light") applyTheme("light"); });
  window.addEventListener("afterprint", () => { if (before && before !== "light") applyTheme(before); before = null; });
}

function interpolatedRobots(now) {
  const buf = app.buffer;
  if (!buf.length) return [];
  const t = now - RENDER_DELAY_MS;
  let a = buf[0], b = buf[buf.length - 1];
  if (t <= a.at) return a.state.robots;
  if (t >= b.at) return b.state.robots;
  for (let i = buf.length - 1; i > 0; i--) {
    if (buf[i - 1].at <= t) { a = buf[i - 1]; b = buf[i]; break; }
  }
  const k = (t - a.at) / Math.max(b.at - a.at, 1);
  const prev = new Map(a.state.robots.map((r) => [r.id, r]));
  return b.state.robots.map((r) => {
    const p = prev.get(r.id);
    if (!p) return r;
    let dh = r.heading - p.heading;
    dh = Math.atan2(Math.sin(dh), Math.cos(dh));
    return { ...r, x: p.x + (r.x - p.x) * k, y: p.y + (r.y - p.y) * k, heading: p.heading + dh * k };
  });
}

function frame(now) {
  if (app.layout) map.drawRobots(interpolatedRobots(now));
  requestAnimationFrame(frame);
}

// ------------------------------------------------------------------ panels

function stationName(cell) {
  const s = app.stationsByCell.get(`${cell[0]},${cell[1]}`);
  return s ? (s.label || s.id) : `(${cell[0]}, ${cell[1]})`;
}

function renderStatus(state) {
  const node = $("#sim-status");
  const label = { READY: "Ready", RUNNING: app.running ? "Running" : "Paused", COMPLETED: "Completed", INVALID: "Invalid layout" }[state.status] || state.status;
  node.className = `sim-status ${state.status === "RUNNING" && app.running ? "running" : state.status.toLowerCase()}`;
  node.innerHTML = `<span class="pulse"></span>${label}`;
  $("#clock").textContent = `${state.time.toFixed(1)} s`;
}

function renderStrategy(strategy) {
  if (!strategy) return;
  app.strategy = strategy;
  for (const b of $("#strategy").children) b.classList.toggle("active", b.dataset.value === strategy);
}

function renderTransport() {
  $("#btn-play use").setAttribute("href", app.running ? "#i-pause" : "#i-play");
  for (const btn of $("#speed").children) btn.classList.toggle("active", Number(btn.dataset.speed) === app.speed);
}

function kpi(label, value, sub = "", unit = "") {
  return `<div class="kpi"><div class="kpi-label">${label}</div><div class="kpi-value">${value}${unit ? `<small>${unit}</small>` : ""}</div><div class="kpi-sub">${sub}</div></div>`;
}

function renderKpis(m, fleet) {
  const safe = m.safety_violations === 0;
  const gap = m.min_separation !== null ? `min gap ${m.min_separation.toFixed(2)} m` : "no contact";
  const safety = `<span class="kpi-status" style="color:var(${safe ? "--good" : "--critical"})"><svg><use href="#${safe ? "s-arrived" : "s-error"}"/></svg>${safe ? "Safe" : "Contact"}</span> · ${gap}`;
  const f = fleet?.metrics;
  $("#kpis").innerHTML = [
    f ? kpi("Missions delivered", f.completed, `${f.per_hour.toFixed(0)} per hour · ${f.queued} queued`)
      : kpi("Trips completed", m.trips_completed, `${m.throughput_per_min.toFixed(1)} per minute`),
    kpi("Fleet moving", `${m.moving}<small>/ ${m.robots}</small>`, `${m.waiting} waiting · ${m.stops} full stops so far`),
    kpi("Conflicts negotiated", m.negotiations, `${m.reroutes} reroutes`),
    kpi("Deadlocks resolved", m.deadlocks_resolved, `${m.backoffs} back-offs`),
    kpi("Safety violations", m.safety_violations, safety),
    f ? kpi("Fleet battery", f.avg_battery === null ? "—" : f.avg_battery.toFixed(0), `${f.charging} charging · ${f.charges} charges`, "%")
      : kpi("Distance driven", m.distance_m.toFixed(0), `${Math.round(m.utilisation * 100)}% fleet utilisation`, "m"),
  ].join("");
}

function batteryBadge(value) {
  if (value === null || value === undefined) return "";
  const cls = value < 15 ? "critical" : value < 30 ? "low" : "";
  return `<span class="battery ${cls}" title="Battery"><svg><use href="#s-battery"/></svg>${value.toFixed(0)}%</span>`;
}

function renderFleet(state) {
  const fleet = $("#fleet");
  const moving = state.robots.filter((r) => ["MOVING", "TURNING", "REROUTING", "BACKING_OFF"].includes(r.status)).length;
  $("#fleet-summary").textContent = `${state.robots.length} robots · ${moving} in motion`;

  const html = state.robots.map((r) => {
    const [cls, icon, label] = STATUS[r.status] || STATUS.IDLE;
    const total = Math.max(r.distance + pathLength(r.path), 0.001);
    const pct = ["ARRIVED", "DOCKED"].includes(r.status) ? 100 : Math.round((r.distance / total) * 100);
    const waitingOn = r.blocked_by && ["WAITING", "YIELDING", "BACKING_OFF"].includes(r.status) ? ` · for ${r.blocked_by}` : "";
    const dispatched = r.mode === "dispatch";
    const heading = dispatched
      ? `${escapeHtml(r.mission ? r.mission : ACTIVITY[r.activity] || r.activity)}${r.mission || !["IDLE"].includes(r.activity) ? `<span class="arrow">→</span>${escapeHtml(stationName(r.goal))}` : ""}`
      : `${escapeHtml(stationName(r.start))}<span class="arrow">→</span>${escapeHtml(stationName(r.goal))}`;
    const eta = r.eta !== null && !["ARRIVED", "DOCKED", "PARKED", "CHARGING"].includes(r.status) ? r.eta.toFixed(0) + " s" : "—";
    return `
      <div class="robot-row${app.selected === r.id ? " selected" : ""}" data-id="${r.id}">
        <div class="robot-badge" style="background:${displayColor(r.color)}">${escapeHtml(r.id)}</div>
        <div class="robot-main">
          <div class="robot-route">${heading}</div>
          <div class="robot-meta">${batteryBadge(r.battery)}<span>${r.speed.toFixed(2)} m/s</span><span>ETA ${eta}</span><span>${r.trips} ${dispatched ? "mission" : "trip"}${r.trips === 1 ? "" : "s"}</span>${dispatched && r.mission ? `<span>${ACTIVITY[r.activity] || ""}</span>` : ""}${r.via.length ? `<span>${r.via.length} via</span>` : ""}</div>
          <div class="progress"><span style="width:${pct}%;background:${displayColor(r.color)}"></span></div>
        </div>
        <span class="status-chip ${cls}" title="${escapeHtml(r.stop_reason || "")}"><svg><use href="#${icon}"/></svg>${label}${waitingOn}</span>
      </div>`;
  }).join("");
  fleet.innerHTML = html;
}

function renderMissions(fleet) {
  const card = $("#missions-card");
  card.hidden = !fleet;
  if (!fleet) return;
  $("#gen-toggle").checked = fleet.generator;
  const m = fleet.metrics;
  const tile = (value, label) => `<div><b>${value}</b><span>${label}</span></div>`;
  $("#mission-kpis").innerHTML = [
    tile(m.queued, "queued"),
    tile(m.active, "in progress"),
    tile(m.avg_wait === null ? "—" : `${m.avg_wait.toFixed(0)} s`, "wait for robot"),
    tile(m.avg_lead_time === null ? "—" : `${m.avg_lead_time.toFixed(0)} s`, "lead time"),
  ].join("");
  const robotColor = new Map((app.latest?.robots || []).map((r) => [r.id, r.color]));
  const row = (mission, extra) => `
    <li class="mission">
      <span class="mid">${mission.id}</span>
      <div>
        <div class="mname">${escapeHtml(mission.pickup_label)} → ${escapeHtml(mission.dropoff_label)}</div>
        <div class="msub">${extra}</div>
      </div>
      ${mission.status === "queued" || mission.status === "assigned"
        ? `<button class="cancel" data-cancel="${mission.id}" title="Cancel order">×</button>` : "<span></span>"}
    </li>`;
  const prio = (p) => `<span class="prio ${p}">${p}</span>`;
  const parts = [];
  if (fleet.active.length) {
    parts.push(`<li class="mission-group">In progress</li>`);
    for (const mission of fleet.active) {
      parts.push(row(mission, `<span class="robot-badge" style="background:${displayColor(robotColor.get(mission.robot)) || "#555"}">${mission.robot}</span>${MISSION_PHASE[mission.status] || mission.status} ${prio(mission.priority)}`));
    }
  }
  if (fleet.queued.length) {
    parts.push(`<li class="mission-group">Queue</li>`);
    const now = app.latest?.time || 0;
    for (const mission of fleet.queued) parts.push(row(mission, `${prio(mission.priority)} waiting ${Math.max(0, now - mission.created).toFixed(0)} s`));
  }
  if (!parts.length) parts.push(`<li class="mission-group">No open orders</li>`);
  $("#missions").innerHTML = parts.join("");
}

async function newOrder() {
  if (!app.layout) return;
  const stations = app.layout.stations.filter((s) => !["parking", "charging"].includes(s.type));
  const options = stations.map((s) => `<option value="${escapeHtml(s.id)}">${escapeHtml(s.label || s.id)}</option>`).join("");
  const form = await dialog({
    title: "New transport order",
    ok: "Create order",
    body: `
      <label>Pickup <select name="pickup">${options}</select></label>
      <label>Drop-off <select name="dropoff">${options}</select></label>
      <label>Priority
        <select name="priority"><option value="high">High</option><option value="normal" selected>Normal</option><option value="low">Low</option></select>
      </label>
      <p class="note">The dispatcher assigns the nearest free robot with enough battery.</p>`,
  });
  if (!form) return;
  send("mission.create", { pickup: form.get("pickup"), dropoff: form.get("dropoff"), priority: form.get("priority") });
}

function pathLength(path) {
  let total = 0;
  for (let i = 1; i < path.length; i++) total += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
  return total;
}

function addEvents(events, animate) {
  const list = $("#events");
  if (!animate) list.innerHTML = "";
  const frag = document.createDocumentFragment();
  for (const e of events) {
    const li = document.createElement("li");
    li.className = `event ev-${e.kind}${animate ? " new" : ""}`;
    li.innerHTML = `<span class="event-time">${e.t.toFixed(1)}s</span><svg class="event-icon"><use href="#${EVENT_ICON[e.kind] || "s-idle"}"/></svg><span class="event-text">${escapeHtml(e.text)}</span>`;
    if (e.robot) li.dataset.robot = e.robot;
    frag.prepend(li);
  }
  list.prepend(frag);
  while (list.children.length > 150) list.lastChild.remove();
}

// Legend for what this layout actually contains, so it stays short.
function renderLegend() {
  const layout = app.layout;
  const chars = new Set(layout.rows.join(""));
  const types = new Set((layout.objects || []).map((o) => o.type));
  const families = new Set([...types].map((t) => objectType(t).family));
  const stations = new Set(layout.stations.map((s) => s.type));
  const item = (swatch, label) => `<span class="legend-item">${swatch}${label}</span>`;
  const box = (fill, stroke = "none", extra = "") => `<svg viewBox="0 0 16 16"><rect x="1.5" y="1.5" width="13" height="13" rx="3" fill="${fill}" stroke="${stroke}" stroke-width="1.2" ${extra}/></svg>`;
  const icon = (id) => `<svg viewBox="0 0 16 16"><use href="#${id}" width="16" height="16" color="var(--text)"/></svg>`;
  const items = [
    item(`<svg viewBox="0 0 16 16"><rect x="1" y="1" width="14" height="14" rx="3" fill="var(--lane)"/><path d="M1 8h14" stroke="var(--lane-edge)" stroke-width="1.6"/><circle cx="8" cy="8" r="1.8" fill="var(--lane-node)"/></svg>`, "Robot lane"),
  ];
  if ([">", "<", "^", "v"].some((c) => chars.has(c))) {
    items.push(item(`<svg viewBox="0 0 16 16"><rect x="1" y="1" width="14" height="14" rx="3" fill="var(--lane)"/><path d="M6 4.5 9.5 8 6 11.5" fill="none" stroke="var(--lane-node)" stroke-width="1.8" stroke-linecap="round"/></svg>`, "One-way"));
  }
  if (chars.has("W")) items.push(item(`<svg viewBox="0 0 16 16"><rect x="1" y="1" width="14" height="14" fill="var(--walkway)"/><path d="M2 2v12M14 2v12" stroke="var(--walkway-edge)" stroke-width="1.6"/></svg>`, "Walkway"));
  if (types.has("crosswalk")) items.push(item(`<svg viewBox="0 0 16 16"><path d="M3 2v12M6.3 2v12M9.6 2v12M12.9 2v12" stroke="var(--crosswalk)" stroke-width="2"/></svg>`, "Crosswalk"));
  if (types.has("slow_zone")) items.push(item(box("rgba(201,162,39,0.08)", "var(--zone-slow)", 'stroke-dasharray="3 2"'), "Slow zone"));
  if (chars.has("S") || families.has("storage")) items.push(item(box("var(--shelf)", "var(--storage-stroke)"), "Rack"));
  if (families.has("production") || families.has("facility")) items.push(item(box("var(--production-fill)", "var(--production-stroke)"), "Equipment"));
  if (families.has("area")) items.push(item(box("var(--area-fill)", "var(--area-stroke)", 'stroke-dasharray="3 2"'), "Area"));
  if (chars.has("H")) items.push(item(`<svg viewBox="0 0 16 16"><rect x="1.5" y="1.5" width="13" height="13" rx="3" fill="rgba(201,162,39,0.25)" stroke="rgba(201,162,39,0.7)"/></svg>`, "Human only"));
  const stationNames = { loading: "Loading", unloading: "Unloading", workstation: "Workstation", charging: "Charging", parking: "Parking" };
  for (const [type, label] of Object.entries(stationNames)) if (stations.has(type)) items.push(item(icon(`st-${type}`), label));
  items.push(item(`<svg viewBox="0 0 16 16"><circle cx="8" cy="8" r="5.5" fill="none" stroke="var(--critical)" stroke-width="1.8"/><circle cx="8" cy="8" r="1.6" fill="var(--critical)"/></svg>`, "Predicted conflict"));
  $("#legend").innerHTML = items.join("");
}

// ------------------------------------------------------------------ selection

function select(id) {
  app.selected = id;
  map.select(id);
  for (const row of document.querySelectorAll(".robot-row")) row.classList.toggle("selected", row.dataset.id === id);
}

// ------------------------------------------------------------------ controls

function initControls() {
  $("#btn-play").addEventListener("click", () => send(app.running ? "pause" : "play"));
  $("#btn-step").addEventListener("click", () => send("step"));
  $("#btn-reset").addEventListener("click", () => send("reset"));
  $("#btn-fit").addEventListener("click", () => map.fit());

  $("#speed").innerHTML = SPEEDS.map((s) => `<button data-speed="${s}" role="radio">${s}×</button>`).join("");
  $("#speed").addEventListener("click", (event) => {
    const btn = event.target.closest("button");
    if (btn) send("speed", { value: Number(btn.dataset.speed) });
  });

  $("#layout-select").addEventListener("change", (event) => {
    const name = event.target.value;
    if (mode === "live") { send("load", { name }); return; }
    if (editor.confirmDiscard()) editor.open(name);
    else syncSelect();
  });

  for (const tab of document.querySelectorAll(".tab")) tab.addEventListener("click", () => setMode(tab.dataset.mode));
  $("#strategy").addEventListener("click", (event) => {
    const button = event.target.closest("button");
    if (!button || button.dataset.value === app.strategy) return;
    send("strategy", { value: button.dataset.value });
    toast(`Coordination: ${STRATEGY_LABEL[button.dataset.value]}${button.dataset.value === "nexus" ? "" : " (classical baseline)"}. Layout restarted.`);
  });
  $("#btn-estop").addEventListener("click", () => {
    if (app.latest?.estop) releaseEstop();
    else send("estop", { value: true });
  });
  $("#btn-estop-release").addEventListener("click", releaseEstop);
  // Typing /#editor or /#live into the address bar of an open page only
  // changes the hash (no reload), so follow it here.
  window.addEventListener("hashchange", () => setMode(modeFromHash()));
  window.addEventListener("beforeunload", (event) => { if (editor.dirty) { event.preventDefault(); event.returnValue = ""; } });

  $("#btn-new-order").addEventListener("click", () => newOrder());
  $("#gen-toggle").addEventListener("change", (event) => send("fleet.generator", { value: event.target.checked }));
  $("#missions").addEventListener("click", (event) => {
    const button = event.target.closest("[data-cancel]");
    if (button) send("mission.cancel", { id: button.dataset.cancel });
  });

  $("#fleet").addEventListener("click", (event) => {
    const row = event.target.closest(".robot-row");
    if (row) select(row.dataset.id === app.selected ? null : row.dataset.id);
  });

  const pop = $("#layers-pop");
  $("#btn-layers").addEventListener("click", (event) => { event.stopPropagation(); pop.hidden = !pop.hidden; });
  document.addEventListener("click", (event) => { if (!event.target.closest(".layers-wrap")) pop.hidden = true; });
  pop.addEventListener("change", (event) => map.setLayer(event.target.dataset.layer, event.target.checked));

  document.addEventListener("keyup", (event) => { if (mode === "editor") editor.onKeyUp(event); });
  document.addEventListener("keydown", (event) => {
    if ($("#dlg").open) return;
    if (mode === "editor") { editor.onKey(event); return; }
    if (mode !== "live") return; // simulation shortcuts only on Overview
    if (event.target.closest("input, select, textarea, button")) return;
    if (event.code === "Space") { event.preventDefault(); send(app.running ? "pause" : "play"); }
    else if (event.code === "ArrowRight") send("step");
    else if (event.key === "r" || event.key === "R") send("reset");
    else if (event.key === "f" || event.key === "F") map.fit();
    else if (event.key === "Escape") select(null);
  });
}

// ------------------------------------------------------------------ splitter

// Resizable side panel: drag the divider, double-click to reset, or focus it
// and use the arrow keys. The width is remembered per browser.
const SIDE_DEFAULT = 380;
const SIDE_MIN = 280;
const SIDE_KEY = "nexus.sideWidth";

function sideLimits() {
  const main = $("#main");
  const max = Math.max(SIDE_MIN, Math.min(900, main.clientWidth - 40 - 16 - 420)); // keep >= 420px of map
  return { min: SIDE_MIN, max };
}

function setSideWidth(px, { save = true } = {}) {
  const { min, max } = sideLimits();
  const width = Math.round(Math.min(max, Math.max(min, px)));
  $("#main").style.setProperty("--side-width", `${width}px`);
  $("#splitter").setAttribute("aria-valuenow", String(width));
  if (save) { try { localStorage.setItem(SIDE_KEY, String(width)); } catch { /* storage unavailable */ } }
  return width;
}

function initSplitter() {
  const splitter = $("#splitter");
  const main = $("#main");
  let saved = SIDE_DEFAULT;
  try { saved = Number(localStorage.getItem(SIDE_KEY)) || SIDE_DEFAULT; } catch { /* storage unavailable */ }
  setSideWidth(saved, { save: false });

  splitter.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    event.preventDefault();
    splitter.setPointerCapture(event.pointerId);
    splitter.classList.add("dragging");
    document.body.classList.add("resizing");
  });
  splitter.addEventListener("pointermove", (event) => {
    if (!splitter.hasPointerCapture(event.pointerId)) return;
    const rect = main.getBoundingClientRect();
    const padding = parseFloat(getComputedStyle(main).paddingRight) || 0;
    // Side panel spans from just right of the handle to the padded edge.
    setSideWidth(rect.right - padding - event.clientX - splitter.offsetWidth / 2, { save: false });
  });
  const end = (event) => {
    if (!splitter.hasPointerCapture(event.pointerId)) return;
    splitter.releasePointerCapture(event.pointerId);
    splitter.classList.remove("dragging");
    document.body.classList.remove("resizing");
    setSideWidth(parseFloat(main.style.getPropertyValue("--side-width")) || SIDE_DEFAULT);
  };
  splitter.addEventListener("pointerup", end);
  splitter.addEventListener("pointercancel", end);
  splitter.addEventListener("dblclick", () => setSideWidth(SIDE_DEFAULT));
  splitter.addEventListener("keydown", (event) => {
    const current = parseFloat(main.style.getPropertyValue("--side-width")) || SIDE_DEFAULT;
    const step = event.shiftKey ? 80 : 24;
    if (event.key === "ArrowLeft") setSideWidth(current + step);
    else if (event.key === "ArrowRight") setSideWidth(current - step);
    else if (event.key === "Home") setSideWidth(SIDE_DEFAULT);
    else return;
    event.preventDefault();
    event.stopPropagation(); // don't also step the simulation / switch tools
  });
  // Keep the saved width valid when the window gets smaller.
  window.addEventListener("resize", () => setSideWidth(parseFloat(main.style.getPropertyValue("--side-width")) || SIDE_DEFAULT, { save: false }));
  splitter.setAttribute("aria-valuemin", String(SIDE_MIN));
}

// ------------------------------------------------------------------ util

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

let toastTimer = null;
function toast(message) {
  const node = $("#toast");
  node.textContent = message;
  node.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { node.hidden = true; }, 4000);
}

// Operations pages share the live connection.
const pageContext = {
  send,
  state: () => app.latest,
  newOrder,
  stationName: (cell) => stationName(cell),
  locate: (id) => { setMode("live"); select(id); },
  toast,
  watchBenchmark: (scenario, strategy) => {
    send("benchmark.watch", { scenario, strategy });
    setMode("live");
    toast(`Replaying ${scenario.replace(/_/g, " ")} with ${STRATEGY_LABEL[strategy]}: press Play`);
  },
};
const pages = {
  missions: new MissionsPage(pageContext),
  fleet: new FleetPage(pageContext),
  alerts: new AlertsPage(pageContext),
  reports: new ReportsPage(pageContext),
  benchmark: new BenchmarkPage(pageContext),
};

// Debug handle for the browser console and end-to-end tests.
window.nexus = { app, map, editor, pages, setMode: (m) => setMode(m), mode: () => mode };

document.body.dataset.mode = mode;
initTheme();
initTooltips();
initControls();
initSplitter();
loadCatalog().then(() => editor.populateCatalog()).then(loadLayoutList).then(() => {
  connect();
  const initial = modeFromHash();
  if (initial !== "live") setMode(initial);
});
requestAnimationFrame(frame);
