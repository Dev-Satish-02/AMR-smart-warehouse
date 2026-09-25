// Layout editor: paint lanes, roads, shelves, walls and human-only areas;
// place stations; add robots with start/goal and via-point routes; preview
// every robot's A* route live; save straight to layouts/*.json.

import { MapView, el } from "./map.js";
import * as ops from "./grid-ops.js";
import * as docs from "./doc-ops.js";
import { CATALOG, objectType, speedLimit } from "./catalog.js";

const $ = (sel) => document.querySelector(sel);

const TOOLS = {
  pan: { label: "Pan" },
  select: { label: "Select" },
  road: { label: "One-way road", color: "#8fb8ff" },
  lane: { label: "Junction (two-way lane)", char: ops.LANE, color: "#8fb8ff" },
  shelf: { label: "Shelf", char: ops.SHELF, color: "#6b7482" },
  wall: { label: "Wall", char: ops.WALL, color: "#9aa0aa" },
  human: { label: "Human-only area", char: ops.HUMAN, color: "#c9a227" },
  walkway: { label: "Walkway", char: "W", color: "#c9a227" },
  erase: { label: "Eraser", char: ops.FLOOR, color: "#d03b3b" },
  station: { label: "Station" },
  route: { label: "Route" },
  object: { label: "Object", color: "#8fb8ff" },
};
const PAINT_TOOLS = new Set(["road", "lane", "shelf", "wall", "human", "walkway", "erase"]);
const TOOL_KEYS = { h: "pan", v: "select", 1: "road", 2: "lane", 3: "shelf", 4: "wall", 5: "human", 6: "erase", 7: "station", 8: "route", 9: "walkway", 0: "object" };

const CELL_NAMES = {
  ".": "Floor", "#": "Wall", S: "Shelf", H: "Human-only", W: "Walkway (people only)", "+": "Junction (two-way)",
  ">": "One-way lane →", "<": "One-way lane ←", "^": "One-way lane ↑", v: "One-way lane ↓",
};

// Same fixed-order categorical palette as the live view (nexus/layout.py).
const ROBOT_COLORS = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"];

const DEFAULT_HINT = "Drag to paint a straight run · Shift-drag fills a rectangle · middle-drag or Space to pan · scroll to zoom";
const TOOL_HINTS = {
  select: "Click a station or robot to edit it · drag robot starts, goals, via-points and stations to move them (robots on top)",
  station: "Click a cell to place a station of the chosen type · drag an existing station to move it",
  route: "Click lane cells to add via-points in order · click a via-point to remove it",
  object: "Drag a rectangle to place the chosen object · select it to rename, resize or change its type",
};

const MAX_UNDO = 100;

export function slugify(text) {
  return String(text).toLowerCase().replace(/[^a-z0-9]+/g, "_").replace(/^_+|_+$/g, "").slice(0, 64) || "layout";
}

const same = (a, b) => !!a && !!b && a[0] === b[0] && a[1] === b[1];

export class Editor {
  constructor({ onRun, onSaved, onOpened = () => {}, toast }) {
    this.onRun = onRun;
    this.onOpened = onOpened;
    this.onSaved = onSaved;
    this.toast = toast;
    this.map = new MapView($("#editor-map"));
    this.map.editHandler = this;
    this.map.panWithLeft = false;
    this.doc = null;
    this.file = null;
    this.dirty = false;
    this.undoStack = [];
    this.redoStack = [];
    this.tool = "road";
    this.lanes = 2;
    this.stationType = "loading";
    this.objectType = "rack";
    this.pane = "robots";
    this.lastDir = [1, 0];
    this.stroke = null;
    this.drag = null;
    this.pick = null;        // { kind: "add" | "field", stage, start, robotId, field }
    this.selection = null;   // { kind: "robot" | "station", id }
    this.hover = null;
    this.highlight = [];
    this.issues = [];
    this.routes = {};
    this.knownFiles = [];
    this._validateTimer = null;
    this._validateSeq = 0;
    this._bind();
    this.setTool("road");
  }

  // ================================================================ document

  get loaded() { return this.doc !== null; }

  async open(name) {
    const res = await fetch(`/api/layouts/${encodeURIComponent(name)}`);
    if (!res.ok) { this.toast(`Could not open ${name}`); return; }
    const body = await res.json();
    this.load(body.layout, name);
  }

  load(layout, file) {
    this.doc = {
      version: layout.version || 1,
      name: layout.name,
      cell_size: layout.cell_size || 1,
      grid: ops.gridFromRows(layout.rows),
      stations: structuredClone(layout.stations || []),
      robots: structuredClone(layout.robots || []).map((r) => ({ ...r, via: r.via || [] })),
      objects: structuredClone(layout.objects || []),
      flows: structuredClone(layout.flows || []),
      fleet: structuredClone(layout.fleet || {}),
      simulation: structuredClone(layout.simulation || {}),
    };
    this.file = file;
    this.undoStack = [];
    this.redoStack = [];
    this.highlight = [];
    this.routes = {};
    this.selection = null;
    this.pick = null;
    this.setDirty(file === null);
    this.render({ fit: true });
    this.validateSoon(0);
    this.onOpened(file);
  }

  toLayout() {
    const d = this.doc;
    return {
      version: d.version,
      name: d.name,
      cell_size: d.cell_size,
      width: d.grid[0].length,
      height: d.grid.length,
      rows: ops.rowsFromGrid(d.grid),
      stations: d.stations,
      robots: d.robots,
      objects: d.objects,
      flows: d.flows,
      fleet: d.fleet,
      simulation: d.simulation,
    };
  }

  setDirty(dirty) {
    this.dirty = dirty;
    $("#ed-dirty").hidden = !dirty;
  }

  confirmDiscard() {
    if (!this.dirty) return true;
    return window.confirm(`Discard unsaved changes to "${this.doc.name}"?`);
  }

  robotColor(robot) {
    const index = this.doc.robots.indexOf(robot);
    return robot.color || ROBOT_COLORS[Math.max(index, 0) % ROBOT_COLORS.length];
  }

  selectedRobot() {
    return this.selection?.kind === "robot" ? this.doc.robots.find((r) => r.id === this.selection.id) || null : null;
  }

  selectedStation() {
    return this.selection?.kind === "station" ? this.doc.stations.find((s) => s.id === this.selection.id) || null : null;
  }

  selectedObject() {
    return this.selection?.kind === "object" ? this.doc.objects.find((o) => o.id === this.selection.id) || null : null;
  }

  // ================================================================ undo

  snapshot() {
    return JSON.stringify({
      grid: this.doc.grid, stations: this.doc.stations, robots: this.doc.robots, objects: this.doc.objects,
      flows: this.doc.flows, fleet: this.doc.fleet, name: this.doc.name,
    });
  }

  restore(snap) {
    Object.assign(this.doc, JSON.parse(snap));
  }

  commit(before, { fit = false } = {}) {
    if (before === this.snapshot()) { this.render(); return; }
    this.undoStack.push(before);
    if (this.undoStack.length > MAX_UNDO) this.undoStack.shift();
    this.redoStack = [];
    this.setDirty(true);
    this.render({ fit });
    this.validateSoon();
  }

  // Apply fn to the document as one undoable change.
  change(fn, options) {
    const before = this.snapshot();
    const result = fn();
    this.commit(before, options);
    return result;
  }

  _history(from, to) {
    if (!from.length) return;
    to.push(this.snapshot());
    const size = () => `${this.doc.grid[0].length}x${this.doc.grid.length}`;
    const sizeBefore = size();
    this.restore(from.pop());
    this.setDirty(true);
    this.render({ fit: sizeBefore !== size() });
    this.validateSoon();
  }

  undo() { this._history(this.undoStack, this.redoStack); }
  redo() { this._history(this.redoStack, this.undoStack); }

  // ================================================================ render

  render({ fit = false } = {}) {
    // Drop a selection whose object no longer exists (undo, delete).
    if (this.selection && !this.selectedRobot() && !this.selectedStation() && !this.selectedObject()) this.selection = null;

    const layout = this.toLayout();
    this.map.setLayout(layout, { fit });
    this.gRoutes = el("g", {}, this.map.gOverlay);
    this.gHighlight = el("g", {}, this.map.gOverlay);
    this.gSelection = el("g", {}, this.map.gOverlay);
    this.gPreview = el("g", {}, this.map.gOverlay);
    this.gHover = el("g", {}, this.map.gOverlay);
    // Routes sit under the robot markers.
    this.map.gMarkers.parentNode.insertBefore(this.gRoutes, this.map.gMarkers);

    this.map.drawRobotMarkers(this.doc.robots.map((r) => ({
      id: r.id, color: this.robotColor(r), start: docs.resolveRef(this.doc, r.start), goal: docs.resolveRef(this.doc, r.goal),
    })));

    $("#ed-name").value = this.doc.name;
    $("#ed-file").textContent = this.file ? `layouts/${this.file}.json` : "Not saved yet";
    $("#ed-width").value = layout.width;
    $("#ed-height").value = layout.height;
    $("#ed-undo").disabled = !this.undoStack.length;
    $("#ed-redo").disabled = !this.redoStack.length;

    const c = ops.countCells(this.doc.grid);
    const stat = (value, label) => `<div class="ed-stat"><b>${value}</b><span>${label}</span></div>`;
    $("#ed-stats").innerHTML = [
      stat(`${layout.width}×${layout.height}`, "metres"),
      stat(c.lane, "lane cells"),
      stat(c.junction, "junctions"),
      stat(this.doc.objects.length, "named objects"),
      stat(this.doc.stations.length, "stations"),
      stat(this.doc.robots.length, "robots"),
    ].join("");
    $("#ed-convert").hidden = c.shelf === 0;
    $("#ed-convert-text").textContent = `${c.shelf} painted shelf cell${c.shelf === 1 ? "" : "s"} without names`;
    this._drawRoutes();
    this._drawHighlight();
    this._drawSelection();
    this._renderFleet();
    this._renderMissionsPane();
  }

  _cellRect(x, y, inset = 0) {
    return this.map.cellRect(x, y, inset);
  }

  _point(cell) {
    const [x, y] = this.map.cellCenter(cell[0], cell[1]);
    return [x, this.map.sy(y)];
  }

  _drawHighlight() {
    if (!this.gHighlight) return;
    this.gHighlight.innerHTML = "";
    for (const [x, y] of this.highlight) {
      const r = el("rect", { ...this._cellRect(x, y, 0.04), rx: 0.1, fill: "rgba(208,59,59,0.18)", stroke: "var(--critical)", "stroke-width": 0.07 }, this.gHighlight);
      el("animate", { attributeName: "stroke-opacity", values: "1;0.25;1", dur: "1.2s", repeatCount: "indefinite" }, r);
    }
  }

  // Planned A* routes (from the last validation): faint for every robot,
  // bold with direction arrows and numbered via-points for the selected one.
  _drawRoutes() {
    if (!this.gRoutes) return;
    this.gRoutes.innerHTML = "";
    const selected = this.selectedRobot();
    const ordered = [...this.doc.robots].sort((a, b) => (a === selected) - (b === selected));
    for (const robot of ordered) {
      const route = this.routes[robot.id];
      const color = this.robotColor(robot);
      const isSel = robot === selected;
      const dim = selected && !isSel;
      const line = (cells, dashed) => {
        if (!cells || cells.length < 2) return;
        el("polyline", {
          points: cells.map((c) => this._point(c).join(",")).join(" "),
          fill: "none", stroke: color, "stroke-width": isSel ? 0.13 : 0.08,
          "stroke-opacity": isSel ? 0.95 : dim ? 0.18 : 0.5,
          "stroke-linejoin": "round", "stroke-linecap": "round",
          "stroke-dasharray": dashed ? "0.3 0.2" : null,
        }, this.gRoutes);
      };
      if (route) {
        line(route.outbound, false);
        if (isSel) line(route.back, true);
        if (isSel && route.outbound) {
          for (let i = 1; i < route.outbound.length - 1; i += 3) {
            const a = route.outbound[i], b = route.outbound[i + 1];
            const [x, y] = this._point(a);
            const angle = Math.atan2(-(b[1] - a[1]), b[0] - a[0]) * 180 / Math.PI;
            el("path", {
              d: "M-0.12 -0.15 L0.08 0 L-0.12 0.15", transform: `translate(${x} ${y}) rotate(${angle})`,
              fill: "none", stroke: "#fff", "stroke-width": 0.06, "stroke-linecap": "round", "stroke-linejoin": "round",
            }, this.gRoutes);
          }
        }
      }
      (robot.via || []).forEach((v, index) => {
        const [x, y] = this._point(v);
        const size = isSel ? 0.22 : 0.14;
        el("rect", {
          x: x - size, y: y - size, width: 2 * size, height: 2 * size, transform: `rotate(45 ${x} ${y})`,
          fill: color, stroke: "#0b0c0e", "stroke-width": 0.04, "fill-opacity": dim ? 0.3 : 1,
        }, this.gRoutes);
        if (isSel) {
          const t = el("text", { x, y: y + 0.09, "font-size": 0.24, "font-weight": 800, fill: "#fff", "text-anchor": "middle" }, this.gRoutes);
          t.textContent = String(index + 1);
        }
      });
    }
  }

  _drawSelection() {
    if (!this.gSelection) return;
    this.gSelection.innerHTML = "";
    const station = this.selectedStation();
    if (station) {
      el("rect", { ...this._cellRect(station.x, station.y, -0.08), rx: 0.2, fill: "none", stroke: "var(--text)", "stroke-width": 0.07 }, this.gSelection);
    }
    const obj = this.selectedObject();
    if (obj) {
      const cs = this.map.cs;
      el("rect", {
        x: obj.x * cs - 0.08, y: this.map.worldH - (obj.y + obj.h) * cs - 0.08,
        width: obj.w * cs + 0.16, height: obj.h * cs + 0.16, rx: 0.16,
        fill: "none", stroke: "var(--text)", "stroke-width": 0.07, "stroke-dasharray": "0.3 0.15",
      }, this.gSelection);
    }
    const robot = this.selectedRobot();
    if (robot) {
      const start = docs.resolveRef(this.doc, robot.start);
      const goal = docs.resolveRef(this.doc, robot.goal);
      if (start) el("circle", { cx: this._point(start)[0], cy: this._point(start)[1], r: 0.52, fill: "none", stroke: "var(--text)", "stroke-width": 0.07 }, this.gSelection);
      if (goal) el("circle", { cx: this._point(goal)[0], cy: this._point(goal)[1], r: 0.44, fill: "none", stroke: this.robotColor(robot), "stroke-width": 0.1 }, this.gSelection);
    }
  }

  // ================================================================ sidebar

  _renderFleet() {
    const list = $("#ed-robot-list");
    const selected = this.selectedRobot();
    if (!this.doc.robots.length) {
      list.innerHTML = `<div class="ed-empty">No robots yet. Use <b>+ Add robot</b>: a dispatched robot needs a home, a fixed-route robot a start and a goal.</div>`;
    } else {
      list.innerHTML = this.doc.robots.map((r) => {
        const route = this.routes[r.id];
        const unreachable = route && route.outbound === null;
        const text = r.mode === "dispatch"
          ? `Dispatched<span class="arrow">·</span>home ${escapeHtml(docs.describeRef(this.doc, r.home ?? r.start))}`
          : `${escapeHtml(docs.describeRef(this.doc, r.start))}<span class="arrow">→</span>${escapeHtml(docs.describeRef(this.doc, r.goal))}`;
        return `
          <div class="ed-robot${r === selected ? " selected" : ""}" data-robot="${escapeHtml(r.id)}">
            <div class="robot-badge" style="background:${this.robotColor(r)}">${escapeHtml(r.id)}</div>
            <div class="route">${text}</div>
            <div class="tags">
              ${r.via?.length ? `<span title="via-points">${r.via.length} via</span>` : ""}
              ${r.loop ? `<svg title="loops"><use href="#i-loop"/></svg>` : ""}
              ${unreachable ? `<span class="bad" title="no lane route">no route</span>` : ""}
            </div>
          </div>`;
      }).join("");
    }
    this._renderInspector();
  }

  _stationOptions(current) {
    const opts = this.doc.stations.map((s) =>
      `<option value="s:${escapeHtml(s.id)}" ${current === s.id ? "selected" : ""}>${escapeHtml(s.label || s.id)} (${escapeHtml(s.id)})</option>`);
    if (Array.isArray(current)) opts.unshift(`<option value="c:${current[0]},${current[1]}" selected>Cell (${current[0]}, ${current[1]})</option>`);
    if (typeof current === "string" && !this.doc.stations.some((s) => s.id === current)) {
      opts.unshift(`<option value="" selected>${escapeHtml(current)} (missing)</option>`);
    }
    if (current === null || current === undefined) opts.unshift(`<option value="" selected>— not set —</option>`);
    return opts.join("");
  }

  _renderInspector() {
    const box = $("#ed-inspect");
    const robot = this.selectedRobot();
    const station = this.selectedStation();

    if (robot && robot.mode === "dispatch") {
      box.innerHTML = `
        <h3><span class="robot-badge" style="background:${this.robotColor(robot)}">${escapeHtml(robot.id)}</span>Robot <span class="muted">${escapeHtml(robot.id)} · dispatched</span></h3>
        <div class="fields">
          <label>ID</label><input type="text" data-field="robot-id" value="${escapeHtml(robot.id)}" />
          <label>Mode</label><select data-field="mode"><option value="dispatch" selected>Dispatched (takes orders)</option><option value="fixed">Fixed route</option></select>
          <label>Home</label><div class="pair"><select data-field="home">${this._stationOptions(robot.home ?? robot.start)}</select><button class="btn small" data-pick="home" title="Pick on map"><svg><use href="#i-target"/></svg></button></div>
          <label>Battery</label><div class="pair"><input type="number" data-field="battery" min="0" max="100" step="5" value="${robot.battery ?? 100}" /><span class="muted">% at start</span></div>
          <label>Max speed</label><div class="pair"><input type="number" data-field="max_speed" min="0.1" max="1" step="0.1" value="${robot.max_speed ?? 1}" /><span class="muted">m/s</span></div>
        </div>
        <div class="route-stats">Parks at home and takes transport orders from the <b>Missions</b> tab flows. Charges automatically below the battery threshold.</div>
        <div class="inspect-actions">
          <button class="btn small danger" data-action="delete-robot"><svg><use href="#i-trash"/></svg>Delete</button>
        </div>`;
      return;
    }

    if (robot) {
      const route = this.routes[robot.id];
      let stats = "";
      if (route && route.outbound) {
        const turns = countTurns(route.outbound);
        stats = `<div class="route-stats">Route: ${route.outbound.length - 1} m · ${turns} turn${turns === 1 ? "" : "s"}${route.back ? ` · return ${route.back.length - 1} m` : ""}</div>`;
      } else if (route) {
        stats = `<div class="route-stats bad">No lane route from start to goal${robot.via.length ? " through these via-points" : ""}.</div>`;
      }
      const vias = robot.via.length
        ? robot.via.map((v, i) => `<span class="via-chip"><b>${i + 1}</b>${v[0]},${v[1]}<button data-remove-via="${i}" title="Remove">×</button></span>`).join("")
        : `<span class="muted">None: A* picks the route</span>`;
      box.innerHTML = `
        <h3><span class="robot-badge" style="background:${this.robotColor(robot)}">${escapeHtml(robot.id)}</span>Robot <span class="muted">${escapeHtml(robot.id)}</span></h3>
        <div class="fields">
          <label>ID</label><input type="text" data-field="robot-id" value="${escapeHtml(robot.id)}" />
          <label>Mode</label><select data-field="mode"><option value="dispatch">Dispatched (takes orders)</option><option value="fixed" selected>Fixed route</option></select>
          <label>Start</label><div class="pair"><select data-field="start">${this._stationOptions(robot.start)}</select><button class="btn small" data-pick="start" title="Pick on map"><svg><use href="#i-target"/></svg></button></div>
          <label>Goal</label><div class="pair"><select data-field="goal">${this._stationOptions(robot.goal)}</select><button class="btn small" data-pick="goal" title="Pick on map"><svg><use href="#i-target"/></svg></button></div>
          <label>Max speed</label><div class="pair"><input type="number" data-field="max_speed" min="0.1" max="1" step="0.1" value="${robot.max_speed ?? 1}" /><span class="muted">m/s</span></div>
          <label>Behaviour</label><label class="check"><input type="checkbox" data-field="loop" ${robot.loop ? "checked" : ""} /> Shuttle back and forth</label>
          <label>Via</label><div class="via-list">${vias}</div>
        </div>
        ${stats}
        <div class="inspect-actions">
          <button class="btn small ${this.tool === "route" ? "active" : ""}" data-action="edit-route"><svg><use href="#t-route"/></svg>${this.tool === "route" ? "Editing route…" : "Edit route"}</button>
          ${robot.via.length ? `<button class="btn small" data-action="clear-via">Clear via-points</button>` : ""}
          <button class="btn small danger" data-action="delete-robot"><svg><use href="#i-trash"/></svg>Delete</button>
        </div>`;
      return;
    }

    if (station) {
      const users = this.doc.robots.filter((r) => r.start === station.id || r.goal === station.id).map((r) => r.id);
      box.innerHTML = `
        <h3><svg width="18" height="18"><use href="#st-${station.type}"/></svg>Station <span class="muted">${escapeHtml(station.id)} at (${station.x}, ${station.y})</span></h3>
        <div class="fields">
          <label>ID</label><input type="text" data-field="station-id" value="${escapeHtml(station.id)}" />
          <label>Label</label><input type="text" data-field="label" value="${escapeHtml(station.label || "")}" />
          <label>Type</label><select data-field="type">${docs.STATION_TYPES.map((t) => `<option value="${t}" ${t === station.type ? "selected" : ""}>${t[0].toUpperCase() + t.slice(1)}</option>`).join("")}</select>
          <label>Used by</label><span class="muted">${users.length ? users.join(", ") : "no robots"}</span>
        </div>
        <div class="inspect-actions">
          <button class="btn small danger" data-action="delete-station"><svg><use href="#i-trash"/></svg>Delete</button>
        </div>`;
      return;
    }

    const obj = this.selectedObject();
    if (obj) {
      const info = objectType(obj.type);
      const families = CATALOG.families || {};
      const groups = Object.entries(families).map(([family, title]) => {
        const options = Object.entries(CATALOG.object_types)
          .filter(([, t]) => t.family === family)
          .map(([type, t]) => `<option value="${type}" ${type === obj.type ? "selected" : ""}>${escapeHtml(t.label)}</option>`).join("");
        return `<optgroup label="${escapeHtml(title)}">${options}</optgroup>`;
      }).join("");
      const effect = info.family === "zone"
        ? `Robots slow to ≤ ${speedLimit(obj)} m/s here; the planner avoids it when a similar route exists.`
        : info.blocking ? "Robots route around it." : "Label only: robots may drive lanes through it.";
      box.innerHTML = `
        <h3><svg width="18" height="18"><use href="#ob-${obj.type}"/></svg>${escapeHtml(info.label)} <span class="muted">${escapeHtml(obj.id)}</span></h3>
        <div class="fields">
          <label>Name</label><input type="text" data-field="object-name" value="${escapeHtml(obj.name)}" />
          <label>Type</label><select data-field="object-type">${groups}</select>
          <label>Placement</label>
          <div class="grid4">
            <label><span>x</span><input type="number" data-field="obj-x" min="0" value="${obj.x}" /></label>
            <label><span>y</span><input type="number" data-field="obj-y" min="0" value="${obj.y}" /></label>
            <label><span>width</span><input type="number" data-field="obj-w" min="1" value="${obj.w}" /></label>
            <label><span>height</span><input type="number" data-field="obj-h" min="1" value="${obj.h}" /></label>
          </div>
          ${info.family === "zone" ? `<label>Speed limit</label><div class="pair"><input type="number" data-field="speed_limit" min="0.1" max="1" step="0.1" value="${speedLimit(obj)}" /><span class="muted">m/s</span></div>` : ""}
        </div>
        <div class="route-stats">${effect}</div>
        <div class="inspect-actions">
          <button class="btn small danger" data-action="delete-object"><svg><use href="#i-trash"/></svg>Delete</button>
        </div>`;
      return;
    }

    box.innerHTML = `<div class="ed-empty">
      Select a robot above, or a station or robot on the map (<b>Select</b> tool, V), to edit it.<br />
      <b>Station</b> (7) places stations · <b>Object</b> (0) places racks, equipment, areas and safety zones · <b>Route</b> (8) adds via-points.<br />
      Everything with a name can be renamed here.<br />
      Robots only turn, change lanes or U-turn at <b>junctions</b>.
    </div>`;
  }

  _onInspectorChange(event) {
    const field = event.target.dataset.field;
    if (!field) return;
    const value = event.target.type === "checkbox" ? event.target.checked : event.target.value;
    const robot = this.selectedRobot();
    const station = this.selectedStation();

    if (robot) {
      if (field === "robot-id") {
        const before = this.snapshot();
        if (!docs.renameRobot(this.doc, robot.id, value)) {
          this.toast("Robot IDs must be unique and not empty");
          this.render();
          return;
        }
        this.selection = { kind: "robot", id: robot.id };
        this.commit(before);
      } else if (field === "start" || field === "goal" || field === "home") {
        if (!value) return;
        this.change(() => {
          docs.setRobotRef(robot, field, value.startsWith("s:") ? value.slice(2) : value.slice(2).split(",").map(Number));
        });
      } else if (field === "mode") {
        this.change(() => docs.setRobotMode(robot, value));
        if (value === "fixed") this.toast(`Pick a goal for ${robot.id}`);
      } else if (field === "battery") {
        const battery = Math.min(100, Math.max(0, Number(value) || 100));
        this.change(() => { robot.battery = battery; });
      } else if (field === "max_speed") {
        const speed = Math.min(1, Math.max(0.1, Number(value) || 1));
        this.change(() => { robot.max_speed = speed; });
      } else if (field === "loop") {
        this.change(() => { if (value) robot.loop = true; else delete robot.loop; });
      }
      return;
    }

    const obj = this.selectedObject();
    if (obj) {
      if (field === "object-name") {
        const before = this.snapshot();
        if (!docs.renameObject(this.doc, obj.id, value)) { this.toast("Names can't be empty"); this.render(); return; }
        this.commit(before);
      } else if (field === "object-type") {
        this.change(() => {
          const oldLabel = objectType(obj.type).label;
          obj.type = value;
          delete obj.speed_limit;
          // Keep custom names; refresh auto-generated ones.
          if (obj.name.startsWith(oldLabel)) obj.name = docs.nextObjectName(this.doc, value);
        });
      } else if (["obj-x", "obj-y"].includes(field)) {
        this.change(() => docs.moveObject(this.doc, obj.id, field === "obj-x" ? Number(value) : obj.x, field === "obj-y" ? Number(value) : obj.y));
      } else if (["obj-w", "obj-h"].includes(field)) {
        this.change(() => docs.resizeObject(this.doc, obj.id, field === "obj-w" ? Number(value) : obj.w, field === "obj-h" ? Number(value) : obj.h));
      } else if (field === "speed_limit") {
        const limit = Math.min(1, Math.max(0.1, Number(value) || 0.3));
        this.change(() => { obj.speed_limit = limit; });
      }
      return;
    }

    if (station) {
      if (field === "station-id") {
        const before = this.snapshot();
        if (!docs.renameStation(this.doc, station.id, value)) {
          this.toast("Station IDs must be unique and not empty");
          this.render();
          return;
        }
        this.selection = { kind: "station", id: station.id };
        this.commit(before);
      } else if (field === "label") {
        this.change(() => { station.label = value.trim() || station.id; });
      } else if (field === "type") {
        this.change(() => { station.type = value; });
      }
    }
  }

  _onInspectorClick(event) {
    const robot = this.selectedRobot();
    const station = this.selectedStation();
    const pick = event.target.closest("[data-pick]");
    if (pick && robot) {
      this.startPick({ kind: "field", robotId: robot.id, field: pick.dataset.pick });
      return;
    }
    const removeVia = event.target.closest("[data-remove-via]");
    if (removeVia && robot) {
      this.change(() => robot.via.splice(Number(removeVia.dataset.removeVia), 1));
      return;
    }
    const action = event.target.closest("[data-action]")?.dataset.action;
    if (action === "edit-route") this.setTool(this.tool === "route" ? "select" : "route");
    else if (action === "clear-via" && robot) this.change(() => { robot.via = []; });
    else if (["delete-robot", "delete-station", "delete-object"].includes(action)) this.deleteSelection();
  }

  deleteSelection() {
    const robot = this.selectedRobot();
    const station = this.selectedStation();
    const obj = this.selectedObject();
    if (obj) {
      this.selection = null;
      this.change(() => docs.removeObject(this.doc, obj.id));
    } else if (robot) {
      this.selection = null;
      this.change(() => docs.removeRobot(this.doc, robot.id));
    } else if (station) {
      const users = this.doc.robots.filter((r) => r.start === station.id || r.goal === station.id).map((r) => r.id);
      if (users.length && !window.confirm(`Robots ${users.join(", ")} use ${station.id}. Delete it anyway?`)) return;
      this.selection = null;
      this.change(() => docs.removeStation(this.doc, station.id));
    }
  }

  select(selection) {
    this.selection = selection;
    this._drawRoutes();
    this._drawSelection();
    this._renderFleet();
  }

  // ================================================================ tools

  setTool(tool) {
    if (tool === "route" && !this.selectedRobot()) {
      if (this.doc?.robots.length) this.toast("Select a robot first (click it in the Robots list)");
      else this.toast("Add a robot first");
      tool = "select";
    }
    this.cancelPick();
    this.tool = tool;
    for (const btn of document.querySelectorAll("#ed-tools .tool")) btn.classList.toggle("active", btn.dataset.tool === tool);
    $("#ed-lanes").hidden = tool !== "road";
    $("#ed-station-type").hidden = tool !== "station";
    $("#ed-object-type").hidden = tool !== "object";
    this._setHint(TOOL_HINTS[tool] || DEFAULT_HINT);
    this._updatePanMode();
    if (this.loaded) this._renderInspector();
  }

  setLanes(n) {
    this.lanes = Math.max(1, Math.min(4, n));
    $("#ed-lanes-value").textContent = this.lanes;
    if (this.stroke) this._drawPreview();
  }

  _setHint(text, picking = false) {
    $("#ed-hint").textContent = text;
    $(".ed-status").classList.toggle("picking", picking);
    this.map.svg.classList.toggle("picking", picking);
  }

  _updatePanMode() {
    const pan = this.tool === "pan" || this.spaceHeld;
    this.map.panWithLeft = !!pan;
    this.map.svg.classList.toggle("pan", !!pan);
  }

  // ---------------------------------------------------------------- pick mode

  startPick(pick) {
    if (!this.loaded) return;
    this.pick = { stage: pick.kind === "add" ? (pick.mode === "dispatch" ? "home" : "start") : pick.field, ...pick };
    const what = this.pick.stage.toUpperCase();
    const who = pick.kind === "add" ? "the new robot" : pick.robotId;
    this._setHint(`Click the ${what} for ${who}: a station or any lane cell · Esc to cancel`, true);
  }

  cancelPick() {
    if (!this.pick) return;
    this.pick = null;
    this._setHint(TOOL_HINTS[this.tool] || DEFAULT_HINT);
  }

  _handlePick(cell) {
    const ref = docs.refAt(this.doc, cell[0], cell[1]);
    if (this.pick.kind === "add") {
      if (this.pick.stage === "home") {
        this.pick = null;
        const robot = this.change(() => docs.addDispatchRobot(this.doc, ref));
        this.setTool("select");
        this.select({ kind: "robot", id: robot.id });
        return;
      }
      if (this.pick.stage === "start") {
        this.pick.start = ref;
        this.pick.stage = "goal";
        this._setHint("Now click the GOAL for the new robot: a station or any lane cell · Esc to cancel", true);
        return;
      }
      const start = this.pick.start;
      this.pick = null;
      const robot = this.change(() => docs.addRobot(this.doc, start, ref));
      this.setTool("select");
      this.select({ kind: "robot", id: robot.id });
      return;
    }
    const robot = this.doc.robots.find((r) => r.id === this.pick.robotId);
    const field = this.pick.field;
    this.cancelPick();
    if (robot) this.change(() => docs.setRobotRef(robot, field, ref));
  }

  // ---------------------------------------------------------------- hit testing

  _hit(cell) {
    const robot = this.selectedRobot();
    if (robot) {
      const via = robot.via.findIndex((v) => same(v, cell));
      if (via >= 0) return { what: "via", id: robot.id, index: via };
      if (same(docs.resolveRef(this.doc, robot.goal), cell)) return { what: "goal", id: robot.id };
    }
    const starter = [...this.doc.robots].reverse().find((r) => same(docs.resolveRef(this.doc, r.start), cell));
    if (starter) return { what: "start", id: starter.id };
    const station = docs.stationAt(this.doc, cell[0], cell[1]);
    if (station) return { what: "station", id: station.id };
    const goaler = [...this.doc.robots].reverse().find((r) => same(docs.resolveRef(this.doc, r.goal), cell));
    if (goaler) return { what: "goal", id: goaler.id };
    const obj = docs.objectAt(this.doc, cell[0], cell[1]);
    if (obj) return { what: "object", id: obj.id, grab: [cell[0] - obj.x, cell[1] - obj.y] };
    return null;
  }

  // ---------------------------------------------------------------- MapView edit handler

  down(cell, event) {
    if (!cell || !this.loaded) return;
    if (this.pick) { this._handlePick(cell); return; }

    if (PAINT_TOOLS.has(this.tool)) {
      this.stroke = { start: cell, current: cell, rect: event.shiftKey && this.tool !== "road" };
      this._drawPreview();
      return;
    }

    if (this.tool === "route") {
      const robot = this.selectedRobot();
      if (!robot) { this.setTool("select"); return; }
      const ch = this.doc.grid[cell[1]][cell[0]];
      const isVia = robot.via.some((v) => same(v, cell));
      if (!isVia && !ops.isLane(ch)) { this.toast("Via-points must be on lane cells"); return; }
      this.change(() => docs.toggleVia(robot, cell[0], cell[1]));
      return;
    }

    if (this.tool === "object") {
      this.stroke = { start: cell, current: cell, rect: true };
      this._drawPreview();
      return;
    }

    if (this.tool === "station") {
      const existing = docs.stationAt(this.doc, cell[0], cell[1]);
      if (existing) {
        this.select({ kind: "station", id: existing.id });
        this.drag = { what: "station", id: existing.id, before: this.snapshot(), last: cell, moved: false };
        return;
      }
      const ch = this.doc.grid[cell[1]][cell[0]];
      if (ch === ops.WALL || ch === ops.SHELF) { this.toast("Stations can't be placed on walls or shelves"); return; }
      const station = this.change(() => docs.addStation(this.doc, cell[0], cell[1], this.stationType));
      this.select({ kind: "station", id: station.id });
      return;
    }

    if (this.tool === "select") {
      const hit = this._hit(cell);
      if (!hit) { this.select(null); return; }
      this.select({ kind: hit.what === "station" || hit.what === "object" ? hit.what : "robot", id: hit.id });
      this.drag = { ...hit, before: this.snapshot(), last: cell, moved: false };
    }
  }

  move(cell) {
    this.hover = cell;
    this._drawHover();
    this._updateCursor(cell);
    if (this.tool === "select" && !this.drag) this.map.svg.classList.toggle("hit", !!(cell && this._hit(cell)));
    if (this.stroke && cell && !same(cell, this.stroke.current)) {
      this.stroke.current = cell;
      this._drawPreview();
    }
    if (this.drag && cell && !same(cell, this.drag.last)) {
      this._dragTo(cell);
    }
  }

  _dragTo(cell) {
    const d = this.drag;
    let ok = true;
    if (d.what === "object") {
      ok = docs.moveObject(this.doc, d.id, cell[0] - d.grab[0], cell[1] - d.grab[1]);
    } else if (d.what === "station") {
      const ch = this.doc.grid[cell[1]][cell[0]];
      ok = ch !== ops.WALL && ch !== ops.SHELF && docs.moveStation(this.doc, d.id, cell[0], cell[1]);
    } else {
      const robot = this.doc.robots.find((r) => r.id === d.id);
      if (!robot) return;
      const ch = this.doc.grid[cell[1]][cell[0]];
      const isStation = !!docs.stationAt(this.doc, cell[0], cell[1]);
      // Only drop where the robot can actually be: lanes (and stations for start/goal).
      if (d.what === "via") {
        if (!ops.isLane(ch) || isStation) return;
        robot.via[d.index] = [cell[0], cell[1]];
      } else {
        if (!ops.isLane(ch) && !isStation) return;
        docs.setRobotRef(robot, d.what, docs.refAt(this.doc, cell[0], cell[1]));
      }
    }
    if (!ok) return;
    d.last = cell;
    d.moved = true;
    this.render();
  }

  up() {
    if (this.drag) {
      const drag = this.drag;
      this.drag = null;
      if (drag.moved) this.commit(drag.before);
      return;
    }
    if (!this.stroke) return;
    if (this.tool === "object") {
      const { start, current } = this.stroke;
      this.stroke = null;
      this.gPreview.innerHTML = "";
      const rect = {
        x: Math.min(start[0], current[0]), y: Math.min(start[1], current[1]),
        w: Math.abs(current[0] - start[0]) + 1, h: Math.abs(current[1] - start[1]) + 1,
      };
      const obj = this.change(() => docs.addObject(this.doc, this.objectType, rect));
      this.select({ kind: "object", id: obj.id });
      return;
    }
    const cells = this._strokeCells();
    const before = this.snapshot();
    let changed = 0;
    if (this.tool === "road") {
      const { dir } = ops.lineCells(this.stroke.start, this.stroke.current);
      if (dir) this.lastDir = dir;
      const stationCells = new Set(this.doc.stations.map((s) => `${s.x},${s.y}`));
      changed = ops.paintRoad(this.doc.grid, cells, stationCells);
    } else if (this.tool === "walkway") {
      // Walkways never cut robot roads: crossings become crosswalks.
      const stationCells = new Set(this.doc.stations.map((s) => `${s.x},${s.y}`));
      const result = ops.paintWalkway(this.doc.grid, cells.map((c) => [c.x, c.y]), stationCells);
      const crosswalks = docs.addCrosswalks(this.doc, result.crossings);
      changed = result.changed + crosswalks.length;
      if (crosswalks.length) this.toast(`Added ${crosswalks.length} crosswalk${crosswalks.length === 1 ? "" : "s"} where the walkway crosses robot lanes`);
    } else {
      changed = ops.paintCells(this.doc.grid, cells.map((c) => [c.x, c.y]), TOOLS[this.tool].char);
    }
    this.stroke = null;
    this.gPreview.innerHTML = "";
    if (changed) this.commit(before);
  }

  leave() {
    this.hover = null;
    if (this.gHover) this.gHover.innerHTML = "";
    this._updateCursor(null);
  }

  _strokeCells() {
    const { start, current, rect } = this.stroke;
    if (this.tool === "road") return ops.roadCells(start, current, this.lanes, this.lastDir);
    const cells = rect ? ops.rectCells(start, current) : ops.lineCells(start, current).cells;
    return cells.map(([x, y]) => ({ x, y }));
  }

  _drawPreview() {
    this.gPreview.innerHTML = "";
    if (!this.stroke) return;
    const color = TOOLS[this.tool].color || "#8fb8ff";
    const width = this.doc.grid[0].length, height = this.doc.grid.length;
    for (const c of this._strokeCells()) {
      if (c.x < 0 || c.y < 0 || c.x >= width || c.y >= height) continue;
      el("rect", { ...this._cellRect(c.x, c.y, 0.05), rx: 0.08, fill: color, "fill-opacity": 0.28, stroke: color, "stroke-width": 0.04 }, this.gPreview);
      if (c.dir) {
        const [cx, cy] = this.map.cellCenter(c.x, c.y);
        const angle = Math.atan2(-c.dir[1], c.dir[0]) * 180 / Math.PI;
        el("path", {
          d: "M-0.16 -0.2 L0.1 0 L-0.16 0.2", transform: `translate(${cx} ${this.map.sy(cy)}) rotate(${angle})`,
          fill: "none", stroke: "#fff", "stroke-width": 0.08, "stroke-linecap": "round", "stroke-linejoin": "round",
        }, this.gPreview);
      }
    }
  }

  _drawHover() {
    if (!this.gHover) return;
    this.gHover.innerHTML = "";
    if (!this.hover || this.stroke || this.drag || this.tool === "pan") return;
    const [x, y] = this.hover;
    const color = this.pick ? "var(--warning)" : "var(--text)";
    el("rect", { ...this._cellRect(x, y, 0.03), rx: 0.08, fill: "none", stroke: color, "stroke-width": 0.05, "stroke-opacity": 0.8 }, this.gHover);
  }

  _updateCursor(cell) {
    const node = $("#ed-cursor");
    if (!cell) { node.textContent = "—"; return; }
    const [x, y] = cell;
    const station = docs.stationAt(this.doc, x, y);
    const obj = docs.objectAt(this.doc, x, y);
    let what = station ? `Station ${station.label || station.id} (${station.type})` : CELL_NAMES[this.doc.grid[y][x]] || "";
    if (obj && !station) what = `${obj.name} (${objectType(obj.type).label}) · ${what}`;
    let extra = "";
    if (this.stroke) {
      const n = this._strokeCells().length;
      extra = ` · ${n} cell${n === 1 ? "" : "s"}`;
    }
    node.textContent = `x ${x}, y ${y} · ${what}${extra}`;
  }

  // ================================================================ document ops

  async resize() {
    const width = Number($("#ed-width").value);
    const height = Number($("#ed-height").value);
    if (!(width >= 3 && width <= 200 && height >= 3 && height <= 200)) {
      this.toast("Width and height must be between 3 and 200 cells");
      return;
    }
    const before = this.snapshot();
    const out = ops.resizeLayout(this.doc.grid, this.doc.stations, width, height);
    if (out.dropped.length && !window.confirm(`Resizing removes station(s) ${out.dropped.join(", ")}. Continue?`)) {
      this.render();
      return;
    }
    this.doc.grid = out.grid;
    this.doc.stations = out.stations;
    this.commit(before, { fit: true });
  }

  rename(name) {
    const clean = name.trim() || "Untitled warehouse";
    if (clean === this.doc.name) return;
    this.change(() => { this.doc.name = clean; });
  }

  async newLayout() {
    if (!this.confirmDiscard()) return;
    const form = await dialog({
      title: "New warehouse",
      ok: "Create",
      body: `
        <label>Name <input type="text" name="name" value="New warehouse" required /></label>
        <div class="row">
          <label>Width (m) <input type="number" name="width" value="30" min="3" max="200" required /></label>
          <label>Height (m) <input type="number" name="height" value="20" min="3" max="200" required /></label>
        </div>
        <label class="check"><input type="checkbox" name="walls" checked /> Surround with walls</label>
        <p class="note">1 cell = 1 m. You can resize later.</p>`,
    });
    if (!form) return;
    const width = Number(form.get("width")), height = Number(form.get("height"));
    this.load({
      name: form.get("name").trim() || "New warehouse",
      cell_size: 1,
      rows: ops.rowsFromGrid(ops.blankGrid(width, height, form.get("walls") === "on")),
      stations: [],
      robots: [],
      simulation: {},
    }, null);
  }

  async save() {
    if (!this.file) return this.saveAs();
    return this._put(this.file);
  }

  async saveAs() {
    const suggested = this.file && this.file !== slugify(this.doc.name) ? slugify(this.doc.name) : (this.file || slugify(this.doc.name));
    const form = await dialog({
      title: "Save layout as",
      ok: "Save",
      body: `
        <label>File name <input type="text" name="file" value="${suggested}" pattern="[A-Za-z0-9_\\-]{1,64}" required /></label>
        <p class="note">Saved to <code>layouts/&lt;file name&gt;.json</code>. Letters, digits, - and _ only.</p>`,
    });
    if (!form) return false;
    const file = form.get("file").trim();
    if (file !== this.file && this.knownFiles.includes(file) && !window.confirm(`layouts/${file}.json already exists. Overwrite it?`)) return false;
    return this._put(file);
  }

  async _put(file) {
    const res = await fetch(`/api/layouts/${encodeURIComponent(file)}`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(this.toLayout()),
    });
    if (!res.ok) {
      const body = await res.json().catch(() => ({}));
      this.toast(`Save failed: ${body.detail || res.status}`);
      return false;
    }
    this.file = file;
    this.setDirty(false);
    this.render();
    await this.onSaved(file);
    return true;
  }

  async run() {
    if (this.issues.some((i) => i.severity === "error")) {
      this.toast("Fix the errors in the Issues panel before running");
      return;
    }
    if (this.dirty || !this.file) {
      const ok = await this.save();
      if (!ok) return;
    }
    this.onRun(this.file);
  }

  // ================================================================ validation + routes

  validateSoon(delay = 250) {
    clearTimeout(this._validateTimer);
    this._validateTimer = setTimeout(() => this.validate(), delay);
  }

  async validate() {
    if (!this.loaded) return;
    const seq = ++this._validateSeq;
    let body;
    try {
      const res = await fetch("/api/validate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(this.toLayout()),
      });
      body = await res.json();
    } catch {
      return;
    }
    if (seq !== this._validateSeq) return; // a newer validation is in flight
    this.issues = (body.issues || []).map((issue) => ({ ...issue, cells: issue.cells || [] }));
    this.routes = body.routes || {};
    this.highlight = [];
    this._drawHighlight();
    this._drawRoutes();
    this._renderFleet();

    const issues = this.issues;
    const errors = issues.filter((i) => i.severity === "error").length;
    const warnings = issues.length - errors;
    $("#ed-issues-summary").textContent = issues.length ? `${errors} error${errors === 1 ? "" : "s"} · ${warnings} warning${warnings === 1 ? "" : "s"}` : "";
    const list = $("#ed-issues");
    if (!issues.length) {
      list.innerHTML = `<li class="ed-issue ok"><svg><use href="#s-arrived"/></svg><span>No issues: this layout can be simulated.</span></li>`;
      return;
    }
    list.innerHTML = issues.map((issue, index) => `
      <li class="ed-issue ${issue.severity}" data-index="${index}" ${issue.cells.length ? "data-has-cells" : ""}>
        <svg><use href="#${issue.severity === "error" ? "s-error" : "s-yield"}"/></svg>
        <span>${escapeHtml(issue.message)}</span>
      </li>`).join("");
  }

  showIssue(index) {
    const issue = this.issues[index];
    if (!issue) return;
    if (issue.target === "fleet" || (this.doc.flows || []).some((f) => f.id === issue.target)) this.setPane("missions");
    if (this.doc.robots.some((r) => r.id === issue.target)) this.select({ kind: "robot", id: issue.target });
    else if (this.doc.stations.some((s) => s.id === issue.target)) this.select({ kind: "station", id: issue.target });
    if (!issue.cells.length) return;
    this.highlight = issue.cells;
    this._drawHighlight();
  }

  // ================================================================ input

  onKey(event) {
    const typing = event.target.closest("input, textarea, select");
    const mod = event.ctrlKey || event.metaKey;
    if (mod && event.key.toLowerCase() === "s") { event.preventDefault(); this.save(); return true; }
    if (typing) return false;
    if (mod && event.key.toLowerCase() === "z") { event.preventDefault(); event.shiftKey ? this.redo() : this.undo(); return true; }
    if (mod && event.key.toLowerCase() === "y") { event.preventDefault(); this.redo(); return true; }
    if (mod) return false;
    if (event.code === "Space") {
      event.preventDefault();
      if (!this.spaceHeld) { this.spaceHeld = true; this._updatePanMode(); }
      return true;
    }
    if (event.key === "Escape") {
      if (this.pick) { this.cancelPick(); return true; }
      if (this.stroke) { this.stroke = null; this.gPreview.innerHTML = ""; return true; }
      if (this.tool === "route") { this.setTool("select"); return true; }
      if (this.selection) { this.select(null); return true; }
      return false;
    }
    if ((event.key === "Delete" || event.key === "Backspace") && this.selection) {
      this.deleteSelection();
      return true;
    }
    const tool = TOOL_KEYS[event.key.toLowerCase()];
    if (tool) { this.setTool(tool); return true; }
    if (event.key === "[") { this.setLanes(this.lanes - 1); return true; }
    if (event.key === "]") { this.setLanes(this.lanes + 1); return true; }
    if (event.key === "f" || event.key === "F") { this.map.fit(); return true; }
    return false;
  }

  onKeyUp(event) {
    if (event.code === "Space" && this.spaceHeld) { this.spaceHeld = false; this._updatePanMode(); }
  }

  // ================================================================ missions pane

  setPane(pane) {
    this.pane = pane;
    for (const tab of document.querySelectorAll(".subtab")) tab.classList.toggle("active", tab.dataset.pane === pane);
    for (const node of document.querySelectorAll(".ed-pane")) node.hidden = node.dataset.pane !== pane;
    $("#ed-add-robot").hidden = pane !== "robots";
    $("#ed-add-flow").hidden = pane !== "missions";
  }

  _renderMissionsPane() {
    const box = $("#ed-missions");
    if (!box || !this.loaded) return;
    const fleet = { generator: true, battery_low: 30, charge_target: 95, charge_rate: 2, drain_per_m: 0.1, service_time: 3, seed: 7, ...this.doc.fleet };
    const flows = this.doc.flows || [];
    const dispatched = this.doc.robots.filter((r) => r.mode === "dispatch").length;
    const label = (id) => {
      const st = this.doc.stations.find((s) => s.id === id);
      return st ? (st.label || st.id) : `${id} (missing)`;
    };
    const stationOptions = this.doc.stations
      .filter((s) => !["parking", "charging"].includes(s.type))
      .map((s) => `<option value="${escapeHtml(s.id)}">${escapeHtml(s.label || s.id)}</option>`).join("");
    const chips = (flow, key) => flow[key].map((id) =>
      `<span class="chip">${escapeHtml(label(id))}<button data-remove="${escapeHtml(flow.id)}|${key}|${escapeHtml(id)}" title="Remove">×</button></span>`).join("");
    const num = (field, value, attrs) => `<input type="number" data-fleet="${field}" value="${value}" ${attrs} />`;
    box.innerHTML = `
      <h4>Fleet settings <span>${dispatched} dispatched robot${dispatched === 1 ? "" : "s"}</span></h4>
      <div class="fleet-settings">
        <label class="check"><input type="checkbox" data-fleet="generator" ${fleet.generator ? "checked" : ""} /> Create orders automatically from the flows</label>
        <label>Charge below (%)${num("battery_low", fleet.battery_low, 'min="5" max="90" step="5"')}</label>
        <label>Charge to (%)${num("charge_target", fleet.charge_target, 'min="30" max="100" step="5"')}</label>
        <label>Charge rate (%/s)${num("charge_rate", fleet.charge_rate, 'min="0.1" max="20" step="0.5"')}</label>
        <label>Drain (%/m)${num("drain_per_m", fleet.drain_per_m, 'min="0" max="2" step="0.05"')}</label>
        <label>Load / unload (s)${num("service_time", fleet.service_time, 'min="0" max="60" step="1"')}</label>
        <label>Random seed${num("seed", fleet.seed, 'min="0" step="1"')}</label>
      </div>
      <h4>Mission flows <span>${flows.length}</span></h4>
      ${flows.length ? "" : `<div class="ed-empty">No flows yet. A flow creates transport orders from its source stations to its destination stations at a set rate. Use <b>+ Add flow</b>.</div>`}
      ${flows.map((f) => `
        <div class="flow${f.enabled ? "" : " off"}" data-flow="${escapeHtml(f.id)}">
          <div class="flow-head">
            <input type="checkbox" data-flow-field="enabled" ${f.enabled ? "checked" : ""} title="Enabled" />
            <input type="text" data-flow-field="name" value="${escapeHtml(f.name)}" />
            <button class="icon-del" data-delete-flow="${escapeHtml(f.id)}" title="Delete flow"><svg><use href="#i-trash"/></svg></button>
          </div>
          <div class="flow-row"><span>From</span><div class="chips">${chips(f, "from")}</div>
            <select data-add="from"><option value="">+ station</option>${stationOptions}</select></div>
          <div class="flow-row"><span>To</span><div class="chips">${chips(f, "to")}</div>
            <select data-add="to"><option value="">+ station</option>${stationOptions}</select></div>
          <div class="flow-row"><span>Rate</span><input type="number" data-flow-field="rate" min="0" max="600" step="5" value="${f.rate}" /> <span style="width:auto">orders / hour</span>
            <select data-flow-field="priority">${["high", "normal", "low"].map((p) => `<option value="${p}" ${p === f.priority ? "selected" : ""}>${p[0].toUpperCase() + p.slice(1)} priority</option>`).join("")}</select></div>
        </div>`).join("")}`;
  }

  _onMissionsChange(event) {
    const t = event.target;
    if (t.dataset.fleet) {
      const key = t.dataset.fleet;
      const value = t.type === "checkbox" ? t.checked : Number(t.value);
      this.change(() => { this.doc.fleet = { ...this.doc.fleet, [key]: value }; });
      return;
    }
    const flowId = t.closest("[data-flow]")?.dataset.flow;
    const flow = (this.doc.flows || []).find((f) => f.id === flowId);
    if (!flow) return;
    if (t.dataset.add) {
      if (!t.value || flow[t.dataset.add].includes(t.value)) { t.value = ""; return; }
      this.change(() => flow[t.dataset.add].push(t.value));
      return;
    }
    const field = t.dataset.flowField;
    if (field === "enabled") this.change(() => { flow.enabled = t.checked; });
    else if (field === "name") this.change(() => { flow.name = t.value.trim() || flow.id; });
    else if (field === "rate") this.change(() => { flow.rate = Math.max(0, Number(t.value) || 0); });
    else if (field === "priority") this.change(() => { flow.priority = t.value; });
  }

  _onMissionsClick(event) {
    const remove = event.target.closest("[data-remove]");
    if (remove) {
      const [flowId, key, id] = remove.dataset.remove.split("|");
      const flow = this.doc.flows.find((f) => f.id === flowId);
      if (flow) this.change(() => { flow[key] = flow[key].filter((x) => x !== id); });
      return;
    }
    const del = event.target.closest("[data-delete-flow]");
    if (del) this.change(() => docs.removeFlow(this.doc, del.dataset.deleteFlow));
  }

  // Fill the Object tool's type picker from the catalogue (after it loads).
  populateCatalog() {
    const select = $("#ed-object-type-select");
    select.innerHTML = Object.entries(CATALOG.families || {}).map(([family, title]) => {
      const options = Object.entries(CATALOG.object_types)
        .filter(([, t]) => t.family === family)
        .map(([type, t]) => `<option value="${type}" ${type === this.objectType ? "selected" : ""}>${escapeHtml(t.label)}</option>`).join("");
      return `<optgroup label="${escapeHtml(title)}">${options}</optgroup>`;
    }).join("");
  }

  _bind() {
    $("#ed-tools").addEventListener("click", (event) => {
      const tool = event.target.closest(".tool");
      if (tool) this.setTool(tool.dataset.tool);
      const step = event.target.closest("[data-step]");
      if (step) { this.setLanes(this.lanes + Number(step.dataset.step)); this.setTool("road"); }
    });
    $("#ed-station-type-select").addEventListener("change", (event) => { this.stationType = event.target.value; });
    $("#ed-object-type-select").addEventListener("change", (event) => { this.objectType = event.target.value; });
    $("#ed-convert-btn").addEventListener("click", () => {
      const racks = this.change(() => docs.shelvesToRacks(this.doc));
      this.toast(`Created ${racks.length} named rack${racks.length === 1 ? "" : "s"}: select one to rename it`);
    });
    $("#ed-undo").addEventListener("click", () => this.undo());
    $("#ed-redo").addEventListener("click", () => this.redo());
    $("#ed-fit").addEventListener("click", () => this.map.fit());
    $("#ed-resize").addEventListener("click", () => this.resize());
    $("#ed-new").addEventListener("click", () => this.newLayout());
    $("#ed-save").addEventListener("click", () => this.save());
    $("#ed-save-as").addEventListener("click", () => this.saveAs());
    $("#ed-run").addEventListener("click", () => this.run());
    $("#ed-name").addEventListener("change", (event) => this.rename(event.target.value));
    $("#ed-name").addEventListener("keydown", (event) => { if (event.key === "Enter") event.target.blur(); });
    $("#ed-issues").addEventListener("click", (event) => {
      const item = event.target.closest(".ed-issue[data-index]");
      if (item) this.showIssue(Number(item.dataset.index));
    });
    $("#ed-add-robot").addEventListener("click", async () => {
      if (!this.loaded) return;
      const form = await dialog({
        title: "Add robot",
        ok: "Next: pick on map",
        body: `
          <label>Mode
            <select name="mode">
              <option value="dispatch">Dispatched: parks at a home and takes transport orders</option>
              <option value="fixed">Fixed route: drives start → goal (loop, via-points)</option>
            </select>
          </label>
          <p class="note">Then click on the map: the home for a dispatched robot, or the start and goal for a fixed route.</p>`,
      });
      if (!form) return;
      this.setTool("select");
      this.startPick({ kind: "add", mode: form.get("mode") });
    });
    for (const tab of document.querySelectorAll(".subtab")) tab.addEventListener("click", () => this.setPane(tab.dataset.pane));
    $("#ed-add-flow").addEventListener("click", () => {
      const flow = this.change(() => docs.addFlow(this.doc));
      this.toast(`Added ${flow.name}: choose its source and destination stations`);
    });
    $("#ed-missions").addEventListener("change", (event) => this._onMissionsChange(event));
    $("#ed-missions").addEventListener("click", (event) => this._onMissionsClick(event));
    $("#ed-missions").addEventListener("keydown", (event) => {
      if (event.key === "Enter" && event.target.matches("input[type=text], input[type=number]")) event.target.blur();
    });
    $("#ed-robot-list").addEventListener("click", (event) => {
      const row = event.target.closest("[data-robot]");
      if (!row) return;
      this.select({ kind: "robot", id: row.dataset.robot });
    });
    $("#ed-inspect").addEventListener("change", (event) => this._onInspectorChange(event));
    $("#ed-inspect").addEventListener("click", (event) => this._onInspectorClick(event));
    $("#ed-inspect").addEventListener("keydown", (event) => {
      if (event.key === "Enter" && event.target.matches("input[type=text], input[type=number]")) event.target.blur();
    });
    window.addEventListener("blur", () => { this.spaceHeld = false; this._updatePanMode(); });
  }
}

// ------------------------------------------------------------------ helpers

function countTurns(cells) {
  let turns = 0;
  for (let i = 1; i < cells.length - 1; i++) {
    const a = [cells[i][0] - cells[i - 1][0], cells[i][1] - cells[i - 1][1]];
    const b = [cells[i + 1][0] - cells[i][0], cells[i + 1][1] - cells[i][1]];
    if (a[0] !== b[0] || a[1] !== b[1]) turns++;
  }
  return turns;
}

export function dialog({ title, body, ok = "OK" }) {
  const dlg = $("#dlg");
  $("#dlg-title").textContent = title;
  $("#dlg-body").innerHTML = body;
  $("#dlg-ok").textContent = ok;
  return new Promise((resolve) => {
    const form = $("#dlg-form");
    const onClose = () => {
      dlg.removeEventListener("close", onClose);
      resolve(dlg.returnValue === "ok" ? new FormData(form) : null);
    };
    dlg.returnValue = "";
    dlg.addEventListener("close", onClose);
    dlg.showModal();
    const first = dlg.querySelector("#dlg-body input, #dlg-body select");
    if (first) { first.focus(); first.select?.(); }
  });
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
