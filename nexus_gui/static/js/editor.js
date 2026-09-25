// Layout editor: paint lanes, roads, shelves, walls and human-only areas on
// the warehouse grid, validate live, and save straight to layouts/*.json.

import { MapView, el } from "./map.js";
import * as ops from "./grid-ops.js";

const $ = (sel) => document.querySelector(sel);

const TOOLS = {
  pan: { label: "Pan" },
  road: { label: "One-way road", color: "#8fb8ff" },
  lane: { label: "Junction (two-way lane)", char: ops.LANE, color: "#8fb8ff" },
  shelf: { label: "Shelf", char: ops.SHELF, color: "#6b7482" },
  wall: { label: "Wall", char: ops.WALL, color: "#9aa0aa" },
  human: { label: "Human-only area", char: ops.HUMAN, color: "#c9a227" },
  erase: { label: "Eraser", char: ops.FLOOR, color: "#d03b3b" },
};
const TOOL_KEYS = { h: "pan", 1: "road", 2: "lane", 3: "shelf", 4: "wall", 5: "human", 6: "erase" };

const CELL_NAMES = {
  ".": "Floor", "#": "Wall", S: "Shelf", H: "Human-only", "+": "Junction (two-way)",
  ">": "One-way lane →", "<": "One-way lane ←", "^": "One-way lane ↑", v: "One-way lane ↓",
};

const MAX_UNDO = 100;

export function slugify(text) {
  return String(text).toLowerCase().replace(/[^a-z0-9]+/g, "_").replace(/^_+|_+$/g, "").slice(0, 64) || "layout";
}

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
    this.lastDir = [1, 0];
    this.stroke = null;
    this.hover = null;
    this.highlight = [];
    this.issues = [];
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
      robots: structuredClone(layout.robots || []),
      simulation: structuredClone(layout.simulation || {}),
    };
    this.file = file;
    this.undoStack = [];
    this.redoStack = [];
    this.highlight = [];
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

  // ================================================================ undo

  snapshot() {
    return JSON.stringify({ grid: this.doc.grid, stations: this.doc.stations, robots: this.doc.robots, name: this.doc.name });
  }

  restore(snap) {
    const s = JSON.parse(snap);
    Object.assign(this.doc, s);
  }

  commit(before, { fit = false } = {}) {
    this.undoStack.push(before);
    if (this.undoStack.length > MAX_UNDO) this.undoStack.shift();
    this.redoStack = [];
    this.setDirty(true);
    this.render({ fit });
    this.validateSoon();
  }

  undo() {
    if (!this.undoStack.length) return;
    this.redoStack.push(this.snapshot());
    const sizeBefore = `${this.doc.grid[0].length}x${this.doc.grid.length}`;
    this.restore(this.undoStack.pop());
    this.setDirty(true);
    this.render({ fit: sizeBefore !== `${this.doc.grid[0].length}x${this.doc.grid.length}` });
    this.validateSoon();
  }

  redo() {
    if (!this.redoStack.length) return;
    this.undoStack.push(this.snapshot());
    const sizeBefore = `${this.doc.grid[0].length}x${this.doc.grid.length}`;
    this.restore(this.redoStack.pop());
    this.setDirty(true);
    this.render({ fit: sizeBefore !== `${this.doc.grid[0].length}x${this.doc.grid.length}` });
    this.validateSoon();
  }

  // ================================================================ render

  render({ fit = false } = {}) {
    const layout = this.toLayout();
    this.map.setLayout(layout, { fit });
    this.gHighlight = el("g", {}, this.map.gOverlay);
    this.gPreview = el("g", {}, this.map.gOverlay);
    this.gHover = el("g", {}, this.map.gOverlay);

    const stationCell = new Map(this.doc.stations.map((s) => [s.id, [s.x, s.y]]));
    const resolve = (ref) => (typeof ref === "string" ? stationCell.get(ref) : ref) || null;
    const colors = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"];
    this.map.drawRobotMarkers(this.doc.robots.map((r, i) => ({
      id: r.id, color: r.color || colors[i % colors.length], start: resolve(r.start), goal: resolve(r.goal),
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
      stat(c.shelf, "shelf cells"),
      stat(this.doc.stations.length, "stations"),
      stat(this.doc.robots.length, "robots"),
    ].join("");
    this._drawHighlight();
  }

  _cellRect(x, y, inset = 0) {
    return this.map.cellRect(x, y, inset);
  }

  _drawHighlight() {
    if (!this.gHighlight) return;
    this.gHighlight.innerHTML = "";
    for (const [x, y] of this.highlight) {
      const r = el("rect", { ...this._cellRect(x, y, 0.04), rx: 0.1, fill: "rgba(208,59,59,0.18)", stroke: "var(--critical)", "stroke-width": 0.07 }, this.gHighlight);
      el("animate", { attributeName: "stroke-opacity", values: "1;0.25;1", dur: "1.2s", repeatCount: "indefinite" }, r);
    }
  }

  // ================================================================ tools

  setTool(tool) {
    this.tool = tool;
    for (const btn of document.querySelectorAll("#ed-tools .tool")) btn.classList.toggle("active", btn.dataset.tool === tool);
    $("#ed-lanes").classList.toggle("off", tool !== "road");
    this._updatePanMode();
  }

  setLanes(n) {
    this.lanes = Math.max(1, Math.min(4, n));
    $("#ed-lanes-value").textContent = this.lanes;
    if (this.stroke) this._drawPreview();
  }

  _updatePanMode() {
    const pan = this.tool === "pan" || this.spaceHeld;
    this.map.panWithLeft = !!pan;
    this.map.svg.classList.toggle("pan", !!pan);
  }

  _strokeCells() {
    const { start, current, rect } = this.stroke;
    if (this.tool === "road") return ops.roadCells(start, current, this.lanes, this.lastDir);
    const cells = rect ? ops.rectCells(start, current) : ops.lineCells(start, current).cells;
    return cells.map(([x, y]) => ({ x, y }));
  }

  // MapView edit handler ---------------------------------------------

  down(cell, event) {
    if (!cell || !this.loaded) return;
    this.stroke = { start: cell, current: cell, rect: event.shiftKey && this.tool !== "road" };
    this._drawPreview();
  }

  move(cell) {
    this.hover = cell;
    this._drawHover();
    this._updateCursor(cell);
    if (this.stroke && cell && (cell[0] !== this.stroke.current[0] || cell[1] !== this.stroke.current[1])) {
      this.stroke.current = cell;
      this._drawPreview();
    }
  }

  up() {
    if (!this.stroke) return;
    const cells = this._strokeCells();
    const before = this.snapshot();
    let changed = 0;
    if (this.tool === "road") {
      const { dir } = ops.lineCells(this.stroke.start, this.stroke.current);
      if (dir) this.lastDir = dir;
      const stationCells = new Set(this.doc.stations.map((s) => `${s.x},${s.y}`));
      changed = ops.paintRoad(this.doc.grid, cells, stationCells);
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

  _drawPreview() {
    this.gPreview.innerHTML = "";
    if (!this.stroke) return;
    const color = TOOLS[this.tool].color;
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
    if (!this.hover || this.stroke || this.tool === "pan") return;
    const [x, y] = this.hover;
    el("rect", { ...this._cellRect(x, y, 0.03), rx: 0.08, fill: "none", stroke: "var(--text)", "stroke-width": 0.05, "stroke-opacity": 0.7 }, this.gHover);
  }

  _updateCursor(cell) {
    const node = $("#ed-cursor");
    if (!cell) { node.textContent = "—"; return; }
    const [x, y] = cell;
    const station = this.doc.stations.find((s) => s.x === x && s.y === y);
    const what = station ? `Station ${station.label || station.id} (${station.type})` : CELL_NAMES[this.doc.grid[y][x]] || "";
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
    const before = this.snapshot();
    this.doc.name = clean;
    this.commit(before);
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

  // ================================================================ validation

  validateSoon(delay = 250) {
    clearTimeout(this._validateTimer);
    this._validateTimer = setTimeout(() => this.validate(), delay);
  }

  async validate() {
    if (!this.loaded) return;
    const seq = ++this._validateSeq;
    let issues;
    try {
      const res = await fetch("/api/validate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(this.toLayout()),
      });
      issues = (await res.json()).issues;
    } catch {
      return;
    }
    if (seq !== this._validateSeq) return; // a newer validation is in flight
    this.issues = issues.map((issue) => ({ ...issue, cells: issue.cells || [] }));
    issues = this.issues;
    this.highlight = [];
    this._drawHighlight();

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
    if (!issue || !issue.cells.length) return;
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
    const tool = TOOL_KEYS[event.key.toLowerCase()];
    if (tool) { this.setTool(tool); return true; }
    if (event.key === "[") { this.setLanes(this.lanes - 1); return true; }
    if (event.key === "]") { this.setLanes(this.lanes + 1); return true; }
    if (event.key === "f" || event.key === "F") { this.map.fit(); return true; }
    if (event.key === "Escape" && this.stroke) { this.stroke = null; this.gPreview.innerHTML = ""; return true; }
    return false;
  }

  onKeyUp(event) {
    if (event.code === "Space" && this.spaceHeld) { this.spaceHeld = false; this._updatePanMode(); }
  }

  _bind() {
    $("#ed-tools").addEventListener("click", (event) => {
      const tool = event.target.closest(".tool");
      if (tool) this.setTool(tool.dataset.tool);
      const step = event.target.closest("[data-step]");
      if (step) { this.setLanes(this.lanes + Number(step.dataset.step)); this.setTool("road"); }
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
    window.addEventListener("blur", () => { this.spaceHeld = false; this._updatePanMode(); });
  }
}

// ------------------------------------------------------------------ dialog

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
    const first = dlg.querySelector("input");
    if (first) { first.focus(); first.select?.(); }
  });
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
