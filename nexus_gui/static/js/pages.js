// Operations pages: Missions, Fleet, Alerts, Reports.
//
// Each page renders only while visible. Live data comes from the WebSocket
// state (fleet, robots, alerts); the Missions and Reports pages also fetch
// their full data sets from the REST API.

import {
  ACTIVITY, ALERT_CODES, SEVERITY, TIME_CATEGORIES, breakdown, categoryColor, displayColor, escapeHtml,
  fmtClock, fmtDuration, fmtNumber, fmtPct, niceScale, statusChip,
} from "./format.js";

const $ = (sel) => document.querySelector(sel);

// ================================================================== tooltip

// Any element with data-tip shows it in the shared tooltip on hover.
export function initTooltips() {
  const tip = $("#tooltip");
  document.addEventListener("mousemove", (event) => {
    const target = event.target.closest?.("[data-tip]");
    if (!target) { tip.hidden = true; return; }
    tip.innerHTML = target.dataset.tip;
    tip.hidden = false;
    const pad = 14;
    const { innerWidth, innerHeight } = window;
    const rect = tip.getBoundingClientRect();
    let x = event.clientX + pad, y = event.clientY + pad;
    if (x + rect.width > innerWidth - 8) x = event.clientX - rect.width - pad;
    if (y + rect.height > innerHeight - 8) y = event.clientY - rect.height - pad;
    tip.style.left = `${x}px`;
    tip.style.top = `${y}px`;
  });
  document.addEventListener("mouseleave", hideTooltip);
  document.addEventListener("mousedown", hideTooltip);
  document.addEventListener("scroll", hideTooltip, true);
}

export function hideTooltip() {
  const tip = $("#tooltip");
  if (tip) tip.hidden = true;
}

function segValue(group) {
  return group.querySelector("button.active")?.dataset.value ?? "";
}

function bindSeg(group, onChange) {
  group.addEventListener("click", (event) => {
    const button = event.target.closest("button");
    if (!button) return;
    for (const b of group.querySelectorAll("button")) b.classList.toggle("active", b === button);
    onChange(button.dataset.value);
  });
}

function timeline(_robot, parts) {
  return `<div class="stackbar">${parts.filter((p) => p.share > 0).map((p) =>
    `<span style="width:${(p.share * 100).toFixed(2)}%;background:${categoryColor(p.key)}" data-tip="<b>${escapeHtml(p.label)}</b>${fmtPct(p.share, 1)} · ${fmtDuration(p.seconds)}"></span>`).join("")}</div>`;
}

export function legendHtml() {
  return TIME_CATEGORIES.map((c) => `<span><i style="background:${categoryColor(c.key)}"></i>${escapeHtml(c.label)}</span>`).join("");
}

// ================================================================== missions

const PHASE = { queued: "Queued", assigned: "To pickup", loading: "Loading", to_dropoff: "Delivering", unloading: "Unloading", completed: "Completed", cancelled: "Cancelled" };
const OPEN = new Set(["queued", "assigned", "loading", "to_dropoff", "unloading"]);
const ACTIVE = new Set(["assigned", "loading", "to_dropoff", "unloading"]);
const PRIORITY_RANK = { high: 0, normal: 1, low: 2 };
const MAX_ROWS = 400;

export class MissionsPage {
  constructor(ctx) {
    this.ctx = ctx;
    this.data = { missions: [], flows: [], generator: false, time: 0 };
    this.timer = null;
    bindSeg($("#mp-status"), () => this.render());
    for (const id of ["#mp-priority", "#mp-flow"]) $(id).addEventListener("change", () => this.render());
    $("#mp-search").addEventListener("input", () => this.render());
    $("#mp-new").addEventListener("click", () => ctx.newOrder().then(() => this.refresh(300)));
    $("#mp-gen").addEventListener("change", (event) => { ctx.send("fleet.generator", { value: event.target.checked }); this.refresh(300); });
    $("#mp-table").addEventListener("click", (event) => {
      const cancel = event.target.closest("[data-cancel]");
      if (cancel) { ctx.send("mission.cancel", { id: cancel.dataset.cancel }); this.refresh(300); }
      const robot = event.target.closest("[data-locate]");
      if (robot) ctx.locate(robot.dataset.locate);
    });
  }

  show() { this.refresh(0); this.timer = setInterval(() => this.refresh(0), 2000); }
  hide() { clearInterval(this.timer); this.timer = null; }

  refresh(delay) {
    setTimeout(async () => {
      try {
        const res = await fetch("/api/missions/all");
        this.data = await res.json();
        this.render();
      } catch { /* server restarting; next poll retries */ }
    }, delay);
  }

  render() {
    const { missions, flows } = this.data;
    const colors = new Map((this.ctx.state()?.robots || []).map((r) => [r.id, r.color]));
    $("#mp-gen").checked = this.data.generator;
    const flowSelect = $("#mp-flow");
    const current = flowSelect.value;
    flowSelect.innerHTML = `<option value="">All flows</option>${flows.map((f) => `<option value="${escapeHtml(f.id)}">${escapeHtml(f.name)}</option>`).join("")}<option value="manual">Manual orders</option>`;
    flowSelect.value = current;

    const status = segValue($("#mp-status"));
    const priority = $("#mp-priority").value;
    const flow = flowSelect.value;
    const query = $("#mp-search").value.trim().toLowerCase();
    const flowName = new Map(flows.map((f) => [f.id, f.name]));
    let rows = missions.filter((m) => {
      if (status === "open" && !OPEN.has(m.status)) return false;
      if (status === "queued" && m.status !== "queued") return false;
      if (status === "active" && !ACTIVE.has(m.status)) return false;
      if (status === "completed" && m.status !== "completed") return false;
      if (status === "cancelled" && m.status !== "cancelled") return false;
      if (priority && m.priority !== priority) return false;
      if (flow === "manual" && m.source !== "manual") return false;
      if (flow && flow !== "manual" && m.flow !== flow) return false;
      if (query) {
        const hay = `${m.id} ${m.pickup_label} ${m.dropoff_label} ${m.robot || ""} ${flowName.get(m.flow) || ""}`.toLowerCase();
        if (!hay.includes(query)) return false;
      }
      return true;
    });
    rows.sort(status === "open" || status === "queued" || status === "active"
      ? (a, b) => PRIORITY_RANK[a.priority] - PRIORITY_RANK[b.priority] || a.created - b.created
      : (a, b) => (b.completed_at ?? b.created) - (a.completed_at ?? a.created));

    const counts = { open: 0, done: 0 };
    for (const m of missions) { if (OPEN.has(m.status)) counts.open++; if (m.status === "completed") counts.done++; }
    $("#mp-meta").textContent = `${missions.length} orders this shift · ${counts.open} open · ${counts.done} delivered · sim time ${fmtClock(this.data.time)}`;

    const now = this.data.time;
    const shown = rows.slice(0, MAX_ROWS);
    $("#mp-table").innerHTML = shown.length ? `
      <table class="data">
        <thead><tr><th>ID</th><th>Order</th><th>Flow</th><th>Priority</th><th>Status</th><th>Robot</th>
          <th class="num">Created</th><th class="num">Waited</th><th class="num">Lead time</th><th></th></tr></thead>
        <tbody>${shown.map((m) => {
          const waited = m.assigned_at !== null ? m.assigned_at - m.created : (m.status === "queued" ? now - m.created : null);
          const lead = m.completed_at !== null && m.status === "completed" ? m.completed_at - m.created : null;
          const robot = m.robot ? `<button class="linklike" data-locate="${escapeHtml(m.robot)}" title="Show on map"><span class="robot-badge" style="background:${displayColor(colors.get(m.robot)) || "#666"}">${escapeHtml(m.robot)}</span></button>` : `<span class="muted">—</span>`;
          return `<tr>
            <td class="mono">${m.id}</td>
            <td class="wrap">${escapeHtml(m.pickup_label)} → ${escapeHtml(m.dropoff_label)}</td>
            <td class="muted">${m.source === "manual" ? "Manual" : escapeHtml(flowName.get(m.flow) || m.flow || "")}</td>
            <td><span class="prio ${m.priority}">${m.priority}</span></td>
            <td>${PHASE[m.status] || m.status}</td>
            <td>${robot}</td>
            <td class="num mono">${fmtClock(m.created)}</td>
            <td class="num">${fmtDuration(waited)}</td>
            <td class="num">${fmtDuration(lead)}</td>
            <td>${m.status === "queued" || m.status === "assigned" ? `<button class="cancel" data-cancel="${m.id}" title="Cancel order">×</button>` : ""}</td>
          </tr>`;
        }).join("")}</tbody>
      </table>${rows.length > MAX_ROWS ? `<div class="empty-row">Showing ${MAX_ROWS} of ${rows.length}: refine the filters or export the CSV from Reports.</div>` : ""}`
      : `<div class="empty-row">No orders match these filters.</div>`;

    $("#mp-flows").innerHTML = flows.length ? `
      <table class="data">
        <thead><tr><th>Flow</th><th class="num">Rate</th><th class="num">Created</th><th class="num">Delivered</th><th class="num">Open</th><th class="num">Avg lead</th></tr></thead>
        <tbody>${flows.map((f) => `<tr${f.enabled ? "" : ' class="muted"'}>
          <td class="wrap">${escapeHtml(f.name)} ${f.priority === "high" ? '<span class="prio high">high</span>' : ""}${f.enabled ? "" : ' <span class="muted">(off)</span>'}</td>
          <td class="num">${fmtNumber(f.rate)}/h</td><td class="num">${f.created}</td><td class="num">${f.completed}</td>
          <td class="num">${f.open}</td><td class="num">${fmtDuration(f.avg_lead_time)}</td></tr>`).join("")}</tbody>
      </table>` : `<div class="empty-row">This layout has no mission flows. Add them in the Layout Editor → Missions tab.</div>`;
  }
}

// ================================================================== fleet

export class FleetPage {
  constructor(ctx) {
    this.ctx = ctx;
    this.last = 0;
    $("#fl-cards").addEventListener("click", (event) => {
      const action = event.target.closest("[data-action]");
      if (!action) return;
      const id = action.dataset.robot;
      if (action.dataset.action === "hold") ctx.send("robot.hold", { id, value: true });
      else if (action.dataset.action === "release") ctx.send("robot.hold", { id, value: false });
      else if (action.dataset.action === "charge") ctx.send("robot.charge", { id });
      else if (action.dataset.action === "locate") ctx.locate(id);
    });
  }

  show() { $("#fl-legend").innerHTML = legendHtml(); this.last = 0; this.render(this.ctx.state()); }
  hide() {}

  render(state, force = false) {
    if (!state) return;
    const now = performance.now();
    if (!force && now - this.last < 500) return; // 2 Hz is plenty for cards
    this.last = now;
    const robots = state.robots;
    const held = robots.filter((r) => r.hold).length;
    const moving = robots.filter((r) => ["MOVING", "TURNING", "REROUTING", "BACKING_OFF"].includes(r.status)).length;
    const dispatched = robots.filter((r) => r.mode === "dispatch");
    const battery = dispatched.length ? dispatched.reduce((s, r) => s + r.battery, 0) / dispatched.length : null;
    $("#fl-meta").textContent = `${robots.length} robots · ${moving} moving · ${dispatched.length} dispatched · ${held} held`
      + (battery !== null ? ` · average battery ${battery.toFixed(0)}%` : "");

    const station = this.ctx.stationName;
    $("#fl-cards").innerHTML = robots.map((r) => {
      const dispatch = r.mode === "dispatch";
      const total = Object.values(r.time_in || {}).reduce((a, b) => a + b, 0);
      const util = total > 0 ? ((r.time_in.moving || 0) + (r.time_in.handling || 0)) / total : 0;
      let now = "";
      if (dispatch) {
        now = r.mission ? `${escapeHtml(r.mission)} · ${ACTIVITY[r.activity] || r.activity} → ${escapeHtml(station(r.goal))}`
          : r.activity === "IDLE" ? `Parked at ${escapeHtml(station(r.home))}` : `${ACTIVITY[r.activity] || r.activity} → ${escapeHtml(station(r.goal))}`;
      } else {
        now = `Route ${escapeHtml(station(r.start))} → ${escapeHtml(station(r.goal))}`;
      }
      const level = r.battery;
      const gauge = dispatch
        ? `<div class="gauge"><svg width="16" height="16"><use href="#s-battery"/></svg><div class="track"><div class="fill" style="width:${level}%;background:var(${level < 15 ? "--critical" : level < 30 ? "--warning" : "--good"})"></div></div><b>${level.toFixed(0)}%</b></div>`
        : `<div class="gauge muted"><svg width="16" height="16"><use href="#s-battery"/></svg>Fixed-route shuttle (battery not simulated)</div>`;
      return `
        <div class="rcard${r.hold ? " held" : ""}">
          <div class="rcard-head">
            <div class="robot-badge" style="background:${displayColor(r.color)}">${escapeHtml(r.id)}</div>
            <div class="rcard-title"><b>${escapeHtml(r.id)}</b><div>${dispatch ? `Dispatched · home ${escapeHtml(station(r.home))}` : "Fixed route"}${r.max_speed ? "" : ""}</div></div>
            ${statusChip(r.status)}
          </div>
          <div class="rcard-now" title="${now.replace(/<[^>]+>/g, "")}">${now}</div>
          ${gauge}
          <div class="rstats">
            <div><b>${r.trips}</b><span>${dispatch ? "missions" : "trips"}</span></div>
            <div><b>${fmtNumber(r.distance)} m</b><span>odometer</span></div>
            <div><b>${fmtPct(util)}</b><span>utilisation</span></div>
            <div><b>${r.charges || 0}</b><span>charges</span></div>
          </div>
          ${timeline(r, breakdown(r.time_in))}
          <div class="rcard-actions">
            ${r.hold
              ? `<button class="btn small" data-action="release" data-robot="${escapeHtml(r.id)}"><svg><use href="#s-moving"/></svg>Release</button>`
              : `<button class="btn small" data-action="hold" data-robot="${escapeHtml(r.id)}" title="Stop this robot where it is"><svg><use href="#i-hold"/></svg>Hold</button>`}
            ${dispatch ? `<button class="btn small" data-action="charge" data-robot="${escapeHtml(r.id)}" ${["CHARGING", "TO_CHARGER"].includes(r.activity) ? "disabled" : ""}><svg><use href="#s-charging"/></svg>Charge now</button>` : ""}
            <button class="btn small" data-action="locate" data-robot="${escapeHtml(r.id)}"><svg><use href="#i-locate"/></svg>Show on map</button>
          </div>
        </div>`;
    }).join("");
  }
}

// ================================================================== alerts

export class AlertsPage {
  constructor(ctx) {
    this.ctx = ctx;
    this.last = 0;
    bindSeg($("#al-filter"), () => this.render(ctx.state(), true));
    $("#al-ack-all").addEventListener("click", () => ctx.send("alert.ack", {}));
    $("#al-active").addEventListener("click", (event) => {
      const ack = event.target.closest("[data-ack]");
      if (ack) ctx.send("alert.ack", { id: ack.dataset.ack });
      const robot = event.target.closest("[data-locate]");
      if (robot) ctx.locate(robot.dataset.locate);
    });
  }

  show() { this.render(this.ctx.state(), true); }
  hide() {}

  render(state, force = false) {
    if (!state?.alerts) return;
    const now = performance.now();
    if (!force && now - this.last < 500) return;
    this.last = now;
    const { active, recent, counts } = state.alerts;
    const t = state.time;
    const filter = segValue($("#al-filter"));
    const keep = (a) => !filter || a.severity === filter;
    $("#al-meta").textContent = `${counts.critical} critical · ${counts.warning} warning · ${counts.info} info · ${counts.unacknowledged} need acknowledgement`;

    const row = (a) => {
      const sev = SEVERITY[a.severity];
      return `
        <div class="alert-row${a.acknowledged ? " acked" : ""}">
          <span class="status-chip ${sev.cls}"><svg><use href="#${sev.icon}"/></svg>${sev.label}</span>
          <div class="what"><b>${escapeHtml(ALERT_CODES[a.code] || a.code)}${a.robot ? ` · <button class="linklike" data-locate="${escapeHtml(a.robot)}">${escapeHtml(a.robot)}</button>` : ""}</b>
            <span>${escapeHtml(a.message)}</span></div>
          <div class="when">since ${fmtClock(a.first_seen)} · ${fmtDuration(t - a.first_seen)}${a.count > 1 ? ` · ×${a.count}` : ""}<br />
            ${a.acknowledged ? "acknowledged" : a.severity === "info" ? "" : `<button class="btn small" data-ack="${a.id}">Acknowledge</button>`}</div>
        </div>`;
    };
    const list = active.filter(keep);
    $("#al-active").innerHTML = list.length ? list.map(row).join("")
      : `<div class="empty-row">No active alerts${filter ? ` of this severity` : ""}. The fleet is operating normally.</div>`;

    const history = recent.filter(keep);
    $("#al-history").innerHTML = history.length ? `
      <table class="data">
        <thead><tr><th>ID</th><th>Severity</th><th>Type</th><th>Robot</th><th>Message</th><th class="num">First seen</th><th class="num">Resolved</th><th class="num">Duration</th><th>Ack</th></tr></thead>
        <tbody>${history.map((a) => `<tr>
          <td class="mono">${a.id}</td><td>${SEVERITY[a.severity].label}</td><td>${escapeHtml(ALERT_CODES[a.code] || a.code)}</td>
          <td>${escapeHtml(a.robot || "—")}</td><td class="wrap">${escapeHtml(a.message)}</td>
          <td class="num mono">${fmtClock(a.first_seen)}</td><td class="num mono">${fmtClock(a.resolved_at)}</td>
          <td class="num">${fmtDuration(a.resolved_at - a.first_seen)}</td><td>${a.acknowledged ? "yes" : "—"}</td></tr>`).join("")}</tbody>
      </table>` : `<div class="empty-row">No resolved alerts yet.</div>`;
  }
}

// Badge on the Alerts tab: unacknowledged critical/warning alerts.
export function renderAlertBadge(alerts) {
  const badge = $("#alert-badge");
  if (!alerts) { badge.hidden = true; return; }
  const n = alerts.counts.unacknowledged;
  badge.hidden = n === 0;
  badge.textContent = n;
  badge.classList.toggle("warning", alerts.counts.critical === 0);
}

// ================================================================== reports

export class ReportsPage {
  constructor(ctx) {
    this.ctx = ctx;
    this.timer = null;
    this.report = null;
    $("#rp-refresh").addEventListener("click", () => this.refresh());
    $("#rp-print").addEventListener("click", () => window.print());
  }

  show() { this.refresh(); this.timer = setInterval(() => this.refresh(), 5000); }
  hide() { clearInterval(this.timer); this.timer = null; }

  async refresh() {
    try {
      const res = await fetch("/api/report");
      if (!res.ok) return;
      this.report = await res.json();
      this.render();
    } catch { /* retry on next tick */ }
  }

  render() {
    const r = this.report;
    if (!r) return;
    const s = r.summary;
    const unit = s.mode === "missions" ? "missions" : "trips";
    $("#rp-meta").textContent = `${r.layout} · shift so far ${fmtDuration(r.shift_seconds)} (simulated) · ${s.robots} robots · generated ${new Date(r.generated_at).toLocaleTimeString()}`
      + (r.estop ? " · E-STOP ENGAGED" : "");

    const tile = (label, value, sub, unitText = "") =>
      `<div class="kpi"><div class="kpi-label">${label}</div><div class="kpi-value">${value}${unitText ? `<small>${unitText}</small>` : ""}</div><div class="kpi-sub">${sub}</div></div>`;
    const kpis = [
      tile(`${unit[0].toUpperCase()}${unit.slice(1)} delivered`, fmtNumber(s.delivered), `${fmtNumber(s.delivered_per_hour, 1)} per hour`),
      tile("Lead time (avg)", fmtDuration(s.lead_time_avg), `90th percentile ${fmtDuration(s.lead_time_p90)}`),
      tile("Wait for a robot (avg)", fmtDuration(s.wait_avg), `90th percentile ${fmtDuration(s.wait_p90)}`),
      tile("Fleet utilisation", fmtPct(s.utilisation), "moving or loading / unloading"),
      tile("Time lost to traffic", fmtPct(s.traffic_share, 1), "robots waiting for each other"),
      tile("Distance driven", fmtNumber(s.distance_m / 1000, 2), `${s.charges} charging sessions`, "km"),
      tile("Safety violations", fmtNumber(r.safety.violations), `min gap ${r.safety.min_separation ?? "—"} m`),
      tile("Orders open", fmtNumber(s.open), `${s.created} created · ${s.cancelled} cancelled`),
    ].join("");

    const fleetParts = breakdown(Object.fromEntries(Object.entries(r.time_breakdown).map(([k, v]) => [k, v * 100])));
    $("#rp-body").innerHTML = `
      <section class="report-kpis">${kpis}</section>
      <div class="report-grid">
        <section class="card">
          <div class="card-head"><h2>${unit[0].toUpperCase()}${unit.slice(1)} delivered per ${r.timeline.bucket_seconds === 60 ? "minute" : `${r.timeline.bucket_seconds / 60} minutes`}</h2><span class="card-hint">hover a bar for details</span></div>
          <div class="chart">${barChart(r.timeline, unit)}</div>
        </section>
        <section class="card">
          <div class="card-head"><h2>Where fleet time goes</h2><span class="card-hint">all robots, whole shift</span></div>
          <div class="chart">
            <div class="stackbar" style="height:22px">${fleetParts.filter((p) => p.share > 0).map((p) =>
              `<span style="width:${(p.share * 100).toFixed(2)}%;background:${categoryColor(p.key)}" data-tip="<b>${escapeHtml(p.label)}</b>${fmtPct(p.share, 1)} of fleet time"></span>`).join("")}</div>
            <div class="chart-legend" style="margin-top:12px">${legendHtml()}</div>
          </div>
          <div class="table-wrap"><table class="data">
            <thead><tr><th>Category</th><th class="num">Share</th></tr></thead>
            <tbody>${fleetParts.map((p) => `<tr><td><span class="chart-legend"><span><i style="background:${categoryColor(p.key)}"></i>${escapeHtml(p.label)}</span></span></td><td class="num">${fmtPct(p.share, 1)}</td></tr>`).join("")}</tbody>
          </table></div>
        </section>
      </div>
      <section class="card">
        <div class="card-head"><h2>Robots</h2><span class="card-hint">per-robot utilisation and time breakdown</span></div>
        <div class="table-wrap"><table class="data">
          <thead><tr><th>Robot</th><th>Mode</th><th class="num">${unit[0].toUpperCase()}${unit.slice(1)}</th><th class="num">Distance</th><th>Utilisation</th><th class="num">Waiting</th><th class="num">Charges</th><th class="num">Battery</th><th style="width:30%">Time breakdown</th></tr></thead>
          <tbody>${r.robots.map((x) => `<tr>
            <td><b>${escapeHtml(x.id)}</b></td><td class="muted">${x.mode === "dispatch" ? "Dispatched" : "Fixed route"}</td>
            <td class="num">${x.completed}</td><td class="num">${fmtNumber(x.distance_m)} m</td>
            <td><span class="bar-cell"><span style="width:${(x.utilisation * 100).toFixed(1)}%"></span></span>${fmtPct(x.utilisation)}</td>
            <td class="num">${fmtPct(x.waiting_share, 1)}</td><td class="num">${x.charges}</td>
            <td class="num">${x.battery === null ? "—" : `${x.battery.toFixed(0)}%`}</td>
            <td>${timeline(x, breakdown(x.time_in))}</td></tr>`).join("")}</tbody>
        </table></div>
      </section>
      ${r.flows.length ? `
      <section class="card">
        <div class="card-head"><h2>Mission flows</h2><span class="card-hint">configured rate vs what was delivered</span></div>
        <div class="table-wrap"><table class="data">
          <thead><tr><th>Flow</th><th>Priority</th><th class="num">Rate</th><th class="num">Created</th><th class="num">Delivered</th><th class="num">Open</th><th class="num">Avg lead time</th></tr></thead>
          <tbody>${r.flows.map((f) => `<tr><td class="wrap">${escapeHtml(f.name)}</td><td><span class="prio ${f.priority}">${f.priority}</span></td>
            <td class="num">${fmtNumber(f.rate)}/h</td><td class="num">${f.created}</td><td class="num">${f.completed}</td><td class="num">${f.open}</td>
            <td class="num">${fmtDuration(f.avg_lead_time)}</td></tr>`).join("")}</tbody>
        </table></div>
      </section>` : ""}
      <section class="card">
        <div class="card-head"><h2>Safety &amp; coordination</h2><span class="card-hint">how NEXUS kept traffic moving</span></div>
        <div class="facts">
          <div><b>${r.safety.violations}</b><span>safety violations (robots closer than 0.7 m)</span></div>
          <div><b>${r.safety.min_separation ?? "—"} m</b><span>closest approach between robots</span></div>
          <div><b>${r.coordination.negotiations}</b><span>conflicts negotiated peer-to-peer</span></div>
          <div><b>${r.coordination.reroutes}</b><span>dynamic reroutes</span></div>
          <div><b>${r.coordination.deadlocks_resolved}</b><span>deadlocks resolved by back-off</span></div>
          <div><b>${Object.values(r.alerts.totals).reduce((a, b) => a + b, 0)}</b><span>alerts raised (${r.alerts.active} active now)</span></div>
        </div>
        ${Object.keys(r.alerts.totals).length ? `<div class="report-note">Alerts by type: ${Object.entries(r.alerts.totals).map(([k, v]) => `${escapeHtml(ALERT_CODES[k] || k)} ${v}`).join(" · ")}</div>` : ""}
      </section>`;
  }
}

// Single-series bar chart (no legend: the card title names the series).
function barChart(timeline, unit) {
  const counts = timeline.counts;
  const width = 640, height = 220, left = 36, bottom = 26, top = 10, right = 8;
  const innerW = width - left - right, innerH = height - top - bottom;
  const { max, ticks } = niceScale(Math.max(...counts, 1), 4);
  const n = counts.length;
  const slot = innerW / n;
  const barW = Math.max(2, Math.min(28, slot - 2)); // 2px gap between bars
  const y = (v) => top + innerH - (v / max) * innerH;
  const minutes = timeline.bucket_seconds / 60;
  const labelEvery = Math.ceil(n / 8);
  const color = categoryColor("moving");
  const bars = counts.map((v, i) => {
    const x = left + i * slot + (slot - barW) / 2;
    const h = Math.max(0, y(0) - y(v));
    const from = fmtClock(i * timeline.bucket_seconds), to = fmtClock((i + 1) * timeline.bucket_seconds);
    const tip = `<b>${from} – ${to}</b>${v} ${unit}`;
    // Rounded data end, square at the baseline; hit area spans the full slot.
    const r = Math.min(4, barW / 2, h);
    const path = h > 0
      ? `M${x},${y(0)} V${y(v) + r} Q${x},${y(v)} ${x + r},${y(v)} H${x + barW - r} Q${x + barW},${y(v)} ${x + barW},${y(v) + r} V${y(0)} Z`
      : "";
    return `<g data-tip="${escapeHtml(tip)}"><rect class="hit" x="${left + i * slot}" y="${top}" width="${slot}" height="${innerH}"></rect>${path ? `<path d="${path}" fill="${color}"></path>` : ""}</g>`;
  }).join("");
  const grid = ticks.map((t) => `<line x1="${left}" x2="${width - right}" y1="${y(t)}" y2="${y(t)}"></line>`).join("");
  const yLabels = ticks.map((t) => `<text x="${left - 6}" y="${y(t) + 4}" text-anchor="end">${t}</text>`).join("");
  const xLabels = counts.map((_, i) => (i % labelEvery === 0
    ? `<text x="${left + i * slot + slot / 2}" y="${height - 8}" text-anchor="middle">${Math.round(i * minutes)}′</text>` : "")).join("");
  return `<svg viewBox="0 0 ${width} ${height}" role="img" aria-label="${unit} delivered per ${minutes} minutes">
    <g class="grid">${grid}</g><line class="baseline" x1="${left}" x2="${width - right}" y1="${y(0)}" y2="${y(0)}"></line>
    <g class="axis">${yLabels}${xLabels}</g>${bars}</svg>`;
}
