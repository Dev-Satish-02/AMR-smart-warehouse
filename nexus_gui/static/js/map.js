// SVG warehouse renderer shared by the live view (and, next, the editor).
// World units are metres; cell (x, y) spans [x, x+1] x [y, y+1] * cellSize,
// with y pointing up. The SVG y axis is flipped via sy().

import { labelPlacement } from "./grid-ops.js";
import { objectType, objectCells, speedLimit } from "./catalog.js";

const NS = "http://www.w3.org/2000/svg";

export const ONE_WAY = { ">": [1, 0], "<": [-1, 0], "^": [0, 1], "v": [0, -1] };
const LANE_CHARS = new Set(["+", ">", "<", "^", "v"]);

export function el(tag, attrs = {}, parent = null) {
  const node = document.createElementNS(NS, tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (value !== undefined && value !== null) node.setAttribute(key, value);
  }
  if (parent) parent.appendChild(node);
  return node;
}

const STATUS_RING = {
  WAITING: "wait",
  YIELDING: "wait",
  BACKING_OFF: "back",
  REROUTING: "back",
};

export class MapView {
  constructor(svg) {
    this.svg = svg;
    this.uid = `m${Math.random().toString(36).slice(2, 8)}`; // unique pattern ids per map
    this.layout = null;
    this.selected = null;
    this.robotNodes = new Map();
    this.hidden = new Set();
    this.onSelect = () => {};
    this.view = null;
    // Editor hooks: when set, left-drag goes to the handler instead of panning
    // (middle-drag, or panWithLeft = true, still pans).
    this.editHandler = null;
    this.panWithLeft = true;
    this._initPanZoom();
  }

  // ------------------------------------------------------------ geometry

  sx(x) { return x; }
  sy(y) { return this.worldH - y; }

  cellRect(x, y, inset = 0) {
    const cs = this.cs;
    return { x: x * cs + inset, y: this.worldH - (y + 1) * cs + inset, width: cs - 2 * inset, height: cs - 2 * inset };
  }

  cellCenter(x, y) {
    return [(x + 0.5) * this.cs, (y + 0.5) * this.cs];
  }

  // ------------------------------------------------------------ layout

  setLayout(layout, { fit = true } = {}) {
    this.layout = layout;
    this.cs = layout.cell_size;
    this.worldW = layout.width * this.cs;
    this.worldH = layout.height * this.cs;
    // grid[y][x], y = 0 bottom
    this.grid = [...layout.rows].reverse();
    this.stations = new Map(layout.stations.map((s) => [`${s.x},${s.y}`, s]));
    this.objects = layout.objects || [];
    // Cells under blocking equipment/racks are not drivable.
    this.blocked = new Set();
    for (const obj of this.objects) {
      if (objectType(obj.type).blocking) for (const [x, y] of objectCells(obj)) this.blocked.add(`${x},${y}`);
    }

    this.svg.innerHTML = "";
    this.robotNodes.clear();
    this._defs();

    this.gFloor = el("g", {}, this.svg);
    this.gCells = el("g", {}, this.svg);
    this.gAreas = el("g", {}, this.svg);
    this.gGraph = el("g", { "data-layer": "graph" }, this.svg);
    this.gZones = el("g", {}, this.svg);
    this.gSafety = el("g", {}, this.svg);
    this.gEquipment = el("g", {}, this.svg);
    this.gObjectLabels = el("g", { "data-layer": "labels" }, this.svg);
    this.gStations = el("g", {}, this.svg);
    this.gStationLabels = el("g", { "data-layer": "labels" }, this.svg);
    this.gReserved = el("g", { "data-layer": "reservations" }, this.svg);
    this.gPaths = el("g", { "data-layer": "paths" }, this.svg);
    this.gConflicts = el("g", { "data-layer": "conflicts" }, this.svg);
    this.gMarkers = el("g", {}, this.svg);
    this.gRobots = el("g", {}, this.svg);
    this.gLabels = el("g", {}, this.svg);
    this.gOverlay = el("g", {}, this.svg);

    this._drawFloor();
    this._drawCells();
    this._drawGraph();
    this._drawZones();
    this._drawObjects();
    this._drawStations();
    this._applyLayers();
    if (fit || !this.view) this.fit();
  }

  _defs() {
    const defs = el("defs", {}, this.svg);
    const hatch = el("pattern", { id: `${this.uid}-hatch`, width: 0.28, height: 0.28, patternUnits: "userSpaceOnUse", patternTransform: "rotate(45)" }, defs);
    el("rect", { width: 0.28, height: 0.28, fill: "rgba(201,162,39,0.07)" }, hatch);
    el("line", { x1: 0, y1: 0, x2: 0, y2: 0.28, stroke: "rgba(201,162,39,0.55)", "stroke-width": 0.06 }, hatch);
    const shelf = el("pattern", { id: `${this.uid}-shelf`, width: 0.25, height: 0.25, patternUnits: "userSpaceOnUse" }, defs);
    el("rect", { width: 0.25, height: 0.25, fill: "var(--shelf)" }, shelf);
    el("line", { x1: 0, y1: 0.125, x2: 0.25, y2: 0.125, stroke: "var(--shelf-line)", "stroke-width": 0.03 }, shelf);
  }

  _char(x, y) {
    if (x < 0 || y < 0 || x >= this.layout.width || y >= this.layout.height) return null;
    return this.grid[y][x];
  }

  isDrivable(x, y) {
    const c = this._char(x, y);
    if (c === null || this.blocked.has(`${x},${y}`)) return false;
    return LANE_CHARS.has(c) || this.stations.has(`${x},${y}`);
  }

  moveAllowed(ax, ay, bx, by) {
    if (!this.isDrivable(ax, ay) || !this.isDrivable(bx, by)) return false;
    const step = [bx - ax, by - ay];
    for (const [x, y] of [[ax, ay], [bx, by]]) {
      if (this.stations.has(`${x},${y}`)) continue;
      const dir = ONE_WAY[this._char(x, y)];
      if (dir && (dir[0] !== step[0] || dir[1] !== step[1])) return false;
    }
    return true;
  }

  _drawFloor() {
    el("rect", { x: 0, y: 0, width: this.worldW, height: this.worldH, fill: "var(--floor)", rx: 0.2 }, this.gFloor);
    let d = "";
    for (let x = 1; x < this.layout.width; x++) d += `M${x * this.cs} 0V${this.worldH}`;
    for (let y = 1; y < this.layout.height; y++) d += `M0 ${y * this.cs}H${this.worldW}`;
    el("path", { d, stroke: "var(--floor-grid)", "stroke-width": 0.02, fill: "none" }, this.gFloor);
  }

  _drawCells() {
    const { width, height } = this.layout;
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const c = this.grid[y][x];
        if (this.blocked.has(`${x},${y}`)) continue; // drawn as equipment
        if (c === "W") {
          this._drawWalkway(x, y);
        } else if (LANE_CHARS.has(c) || this.stations.has(`${x},${y}`)) {
          el("rect", { ...this.cellRect(x, y, 0.02), rx: 0.06, fill: "var(--lane)" }, this.gCells);
        } else if (c === "S") {
          el("rect", { ...this.cellRect(x, y, 0.07), rx: 0.08, fill: `url(#${this.uid}-shelf)`, stroke: "var(--shelf-line)", "stroke-width": 0.03 }, this.gCells);
        } else if (c === "#") {
          el("rect", { ...this.cellRect(x, y, 0), fill: "var(--wall)" }, this.gCells);
        } else if (c === "H") {
          el("rect", { ...this.cellRect(x, y, 0), fill: `url(#${this.uid}-hatch)` }, this.gCells);
        }
      }
    }
  }

  // Pedestrian walkway: lighter floor with safety-yellow edge lines where
  // it borders anything that is not walkway.
  _drawWalkway(x, y) {
    const r = this.cellRect(x, y, 0);
    el("rect", { ...r, fill: "var(--walkway)" }, this.gCells);
    const inset = 0.07;
    const edges = [
      [[0, 1], [r.x, r.y + inset, r.x + r.width, r.y + inset]],
      [[0, -1], [r.x, r.y + r.height - inset, r.x + r.width, r.y + r.height - inset]],
      [[-1, 0], [r.x + inset, r.y, r.x + inset, r.y + r.height]],
      [[1, 0], [r.x + r.width - inset, r.y, r.x + r.width - inset, r.y + r.height]],
    ];
    for (const [[dx, dy], [x1, y1, x2, y2]] of edges) {
      if (this._char(x + dx, y + dy) === "W") continue;
      el("line", { x1, y1, x2, y2, stroke: "var(--walkway-edge)", "stroke-width": 0.06 }, this.gCells);
    }
  }

  // Racks, equipment, labelled areas and safety zones.
  _drawObjects() {
    for (const obj of this.objects) {
      const info = objectType(obj.type);
      const rect = {
        x: obj.x * this.cs,
        y: this.worldH - (obj.y + obj.h) * this.cs,
        width: obj.w * this.cs,
        height: obj.h * this.cs,
      };
      const family = info.family;
      if (family === "area") this._drawArea(obj, rect);
      else if (family === "zone") this._drawSafetyZone(obj, rect);
      else this._drawEquipment(obj, rect, family);
    }
  }

  _inset(rect, d) {
    return { x: rect.x + d, y: rect.y + d, width: Math.max(0.05, rect.width - 2 * d), height: Math.max(0.05, rect.height - 2 * d) };
  }

  // Icon + name centred along the rectangle's long axis, sized to fit.
  _objectLabel(obj, rect, { icon = true, color = "var(--object-text)", max = 0.4, plate = false } = {}) {
    const horizontal = rect.width >= rect.height;
    const long = horizontal ? rect.width : rect.height;
    const short = horizontal ? rect.height : rect.width;
    const text = obj.name || objectType(obj.type).label;
    const iconSize = icon ? Math.min(0.55, short * 0.6) : 0;
    const gap = icon ? iconSize * 0.3 : 0;
    const size = Math.min(max, short * 0.34, (long * 0.88 - iconSize - gap) / Math.max(text.length * 0.58, 1));
    const cx = rect.x + rect.width / 2, cy = rect.y + rect.height / 2;
    const g = el("g", { transform: `translate(${cx} ${cy})${horizontal ? "" : " rotate(-90)"}` }, this.gObjectLabels);
    if (size < 0.13) {
      if (icon && iconSize >= 0.2) el("use", { href: `#ob-${obj.type}`, x: -iconSize / 2, y: -iconSize / 2, width: iconSize, height: iconSize, color }, g);
      return;
    }
    const textWidth = text.length * size * 0.58;
    const total = iconSize + gap + textWidth;
    const left = -total / 2;
    if (plate) {
      el("rect", { x: left - 0.1, y: -Math.max(iconSize, size) / 2 - 0.08, width: total + 0.2, height: Math.max(iconSize, size) + 0.16, rx: 0.1, fill: "var(--floor)", "fill-opacity": 0.82 }, g);
    }
    if (icon) el("use", { href: `#ob-${obj.type}`, x: left, y: -iconSize / 2, width: iconSize, height: iconSize, color }, g);
    const t = el("text", { x: left + iconSize + gap, y: size * 0.35, "font-size": size, "font-weight": 700, fill: color }, g);
    t.textContent = text;
  }

  _drawEquipment(obj, rect, family) {
    const r = this._inset(rect, 0.06);
    if (obj.type === "rack" || family === "storage") {
      el("rect", { ...r, rx: 0.08, fill: `url(#${this.uid}-shelf)`, stroke: "var(--storage-stroke)", "stroke-width": 0.05 }, this.gEquipment);
      this._objectLabel(obj, r, { icon: false, plate: true, max: 0.34 });
      return;
    }
    const fill = family === "production" ? "var(--production-fill)" : "var(--facility-fill)";
    const stroke = family === "production" ? "var(--production-stroke)" : "var(--facility-stroke)";
    el("rect", { ...r, rx: 0.14, fill, stroke, "stroke-width": 0.06 }, this.gEquipment);
    if (obj.type === "conveyor") {
      const horizontal = r.width >= r.height;
      const n = Math.floor((horizontal ? r.width : r.height) / 0.5);
      for (let i = 1; i < n; i++) {
        const t = (horizontal ? r.x : r.y) + i * 0.5;
        el("line", horizontal
          ? { x1: t, y1: r.y + 0.08, x2: t, y2: r.y + r.height - 0.08 }
          : { x1: r.x + 0.08, y1: t, x2: r.x + r.width - 0.08, y2: t },
        this.gEquipment).setAttribute("stroke", "var(--production-stroke)");
      }
    }
    this._objectLabel(obj, r, { plate: obj.type === "conveyor" });
  }

  _drawArea(obj, rect) {
    el("rect", { ...rect, fill: "var(--area-fill)", stroke: "var(--area-stroke)", "stroke-width": 0.05, "stroke-dasharray": "0.3 0.18" }, this.gAreas);
    // Name tag in the top-left corner so lanes through the area stay readable.
    const size = Math.min(0.34, rect.height * 0.3);
    const text = obj.name || objectType(obj.type).label;
    if (size < 0.14 || text.length * size * 0.58 + size * 1.4 > rect.width) {
      this._objectLabel(obj, this._inset(rect, 0.1), { color: "var(--area-stroke)", max: 0.3 });
      return;
    }
    const g = el("g", { transform: `translate(${rect.x + 0.14} ${rect.y + 0.14})` }, this.gObjectLabels);
    el("use", { href: `#ob-${obj.type}`, x: 0, y: 0, width: size * 1.1, height: size * 1.1, color: "var(--area-stroke)" }, g);
    const t = el("text", { x: size * 1.35, y: size * 0.88, "font-size": size, "font-weight": 700, "letter-spacing": 0.02, fill: "var(--area-stroke)" }, g);
    t.textContent = text.toUpperCase();
  }

  _drawSafetyZone(obj, rect) {
    if (obj.type === "crosswalk") {
      // Zebra bars across the short side, repeated along the long side.
      const horizontal = rect.width >= rect.height;
      const long = horizontal ? rect.width : rect.height;
      const bar = 0.2, pitch = 0.4;
      for (let t = (long % pitch) / 2 + 0.1; t + bar <= long + 1e-6; t += pitch) {
        el("rect", horizontal
          ? { x: rect.x + t, y: rect.y + 0.08, width: bar, height: rect.height - 0.16 }
          : { x: rect.x + 0.08, y: rect.y + t, width: rect.width - 0.16, height: bar },
        this.gSafety).setAttribute("fill", "var(--crosswalk)");
      }
      return;
    }
    el("rect", { ...rect, fill: "rgba(201,162,39,0.06)", stroke: "var(--zone-slow)", "stroke-width": 0.06, "stroke-dasharray": "0.24 0.14" }, this.gSafety);
    const limit = speedLimit(obj);
    const text = `${obj.name || "Slow zone"}${limit ? ` · ≤ ${limit} m/s` : ""}`;
    const size = Math.min(0.3, rect.height * 0.35);
    if (size >= 0.14 && text.length * size * 0.58 + 0.3 < rect.width) {
      const t = el("text", { x: rect.x + 0.14, y: rect.y + rect.height - 0.14, "font-size": size, "font-weight": 700, fill: "var(--zone-slow)" }, this.gObjectLabels);
      t.textContent = text;
    }
  }

  _drawGraph() {
    const { width, height } = this.layout;
    let edges = "";
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        if (!this.isDrivable(x, y)) continue;
        const [cx, cy] = this.cellCenter(x, y);
        for (const [dx, dy] of [[1, 0], [0, 1]]) {
          const nx = x + dx, ny = y + dy;
          if (this.moveAllowed(x, y, nx, ny) || this.moveAllowed(nx, ny, x, y)) {
            const [ex, ey] = this.cellCenter(nx, ny);
            edges += `M${cx} ${this.sy(cy)}L${ex} ${this.sy(ey)}`;
          }
        }
        const dir = this.stations.has(`${x},${y}`) ? null : ONE_WAY[this.grid[y][x]];
        if (dir) {
          // chevron pointing along the travel direction
          const angle = Math.atan2(-dir[1], dir[0]) * 180 / Math.PI;
          el("path", {
            d: "M-0.13 -0.16 L0.07 0 L-0.13 0.16",
            transform: `translate(${cx} ${this.sy(cy)}) rotate(${angle})`,
            fill: "none", stroke: "var(--lane-node)", "stroke-width": 0.07,
            "stroke-linecap": "round", "stroke-linejoin": "round",
          }, this.gGraph);
        } else if (!this.stations.has(`${x},${y}`)) {
          el("circle", { cx, cy: this.sy(cy), r: 0.06, fill: "var(--lane-node)" }, this.gGraph);
        }
      }
    }
    el("path", { d: edges, stroke: "var(--lane-edge)", "stroke-width": 0.05, fill: "none", "stroke-linecap": "round" }, this.gGraph, true);
    this.gGraph.insertBefore(this.gGraph.lastChild, this.gGraph.firstChild);
  }

  _drawZones() {
    // Label each connected human-only region.
    const { width, height } = this.layout;
    const seen = new Set();
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        if (this.grid[y][x] !== "H" || seen.has(`${x},${y}`)) continue;
        const stack = [[x, y]];
        const cells = [];
        seen.add(`${x},${y}`);
        while (stack.length) {
          const [cx, cy] = stack.pop();
          cells.push([cx, cy]);
          for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
            const nx = cx + dx, ny = cy + dy, key = `${nx},${ny}`;
            if (this._char(nx, ny) === "H" && !seen.has(key)) { seen.add(key); stack.push([nx, ny]); }
          }
        }
        // Label on the zone's longest straight strip: the bounding-box
        // centre of a ring or L-shaped zone is not part of the zone.
        const spot = labelPlacement(cells);
        if (!spot || spot.length < 3) continue;
        const size = Math.min(0.42, spot.thickness * 0.45);
        const icon = size * 1.15;
        const gap = size * 0.35;
        const fits = (text) => icon + gap + text.length * size * 0.68 <= spot.length * this.cs * 0.9;
        const label = fits("HUMAN ONLY") ? "HUMAN ONLY" : fits("HUMAN") ? "HUMAN" : null;
        if (!label) continue;
        const mx = spot.cx * this.cs, my = spot.cy * this.cs;
        const g = el("g", { transform: `translate(${mx} ${this.sy(my)})${spot.horizontal ? "" : " rotate(-90)"}` }, this.gZones);
        const textWidth = label.length * size * 0.68;
        const left = -(icon + gap + textWidth) / 2;
        el("use", { href: "#i-person", x: left, y: -icon / 2, width: icon, height: icon, color: "rgba(201,162,39,0.9)" }, g);
        const text = el("text", { x: left + icon + gap, y: size * 0.36, "font-size": size, "font-weight": 800, "letter-spacing": 0.04, fill: "rgba(201,162,39,0.9)" }, g);
        text.textContent = label;
      }
    }
  }

  _drawStations() {
    for (const s of this.layout.stations) {
      const r = this.cellRect(s.x, s.y, 0.08);
      const g = el("g", { class: `station station-${s.type}` }, this.gStations);
      el("rect", { ...r, rx: 0.14, fill: "#262a30", stroke: "var(--station)", "stroke-width": 0.05 }, g);
      el("use", { href: `#st-${s.type}`, x: r.x + 0.2 * this.cs, y: r.y + 0.2 * this.cs, width: 0.44 * this.cs, height: 0.44 * this.cs, color: "var(--text)" }, g);
      const [cx] = this.cellCenter(s.x, s.y);
      const above = s.y < this.layout.height - 1 && !this.isDrivable(s.x, s.y + 1);
      const ly = above ? r.y - 0.14 : r.y + r.height + 0.36;
      const label = el("text", { x: cx, y: ly, "font-size": 0.3, "font-weight": 600, fill: "var(--text-2)", "text-anchor": "middle", "paint-order": "stroke", stroke: "var(--floor)", "stroke-width": 0.12 }, this.gStationLabels);
      label.textContent = s.label || s.id;
    }
  }

  // ------------------------------------------------------------ dynamic

  setState(state) {
    this.state = state;
    const byId = new Map(state.robots.map((r) => [r.id, r]));

    // Reservations
    this.gReserved.innerHTML = "";
    for (const r of state.robots) {
      if (this.selected && this.selected !== r.id) continue;
      for (const [x, y] of r.reserved) {
        el("rect", { ...this.cellRect(x, y, 0.06), rx: 0.1, fill: r.color, "fill-opacity": 0.16, stroke: r.color, "stroke-opacity": 0.55, "stroke-width": 0.035 }, this.gReserved);
      }
    }

    // Paths + goals + via points
    this.gPaths.innerHTML = "";
    this.pathNodes = new Map();
    for (const r of state.robots) {
      const dim = this.selected && this.selected !== r.id;
      const g = el("g", { class: dim ? "dimmed" : "" }, this.gPaths);
      if (r.path.length > 1 && !["ARRIVED", "DOCKED"].includes(r.status)) {
        const line = el("polyline", {
          points: r.path.map(([x, y]) => `${x},${this.sy(y)}`).join(" "),
          fill: "none", stroke: r.color, "stroke-width": this.selected === r.id ? 0.12 : 0.08,
          "stroke-opacity": this.selected === r.id ? 0.95 : 0.6, "stroke-linejoin": "round", "stroke-linecap": "round",
        }, g);
        this.pathNodes.set(r.id, { line, rest: r.path.slice(1) });
      }
      const [gx, gy] = this.cellCenter(r.goal[0], r.goal[1]);
      if (!["ARRIVED", "DOCKED"].includes(r.status)) {
        el("circle", { cx: gx, cy: this.sy(gy), r: 0.3, fill: "none", stroke: r.color, "stroke-width": 0.07, "stroke-dasharray": "0.14 0.1" }, g);
      }
      for (const [vx, vy] of r.via) {
        el("rect", { x: vx - 0.13, y: this.sy(vy) - 0.13, width: 0.26, height: 0.26, transform: `rotate(45 ${vx} ${this.sy(vy)})`, fill: r.color, stroke: "var(--floor)", "stroke-width": 0.04 }, g);
      }
    }

    // Predicted conflicts + active negotiations
    this.gConflicts.innerHTML = "";
    for (const n of state.negotiations) {
      const w = byId.get(n.winner), l = byId.get(n.loser);
      if (!w || !l) continue;
      el("line", { x1: w.x, y1: this.sy(w.y), x2: l.x, y2: this.sy(l.y), class: "negotiation-line", "stroke-width": 0.06 }, this.gConflicts);
    }
    const seen = new Set();
    for (const c of [...state.predicted, ...state.negotiations]) {
      const key = c.pair.slice().sort().join("|");
      if (seen.has(key)) continue;
      seen.add(key);
      const [x, y] = c.position;
      const g = el("g", { transform: `translate(${x} ${this.sy(y)})` }, this.gConflicts);
      const ring = el("circle", { r: 0.45, class: "conflict-ring", "stroke-width": 0.06 }, g);
      el("animate", { attributeName: "r", values: "0.35;0.7;0.35", dur: "1.2s", repeatCount: "indefinite" }, ring);
      el("animate", { attributeName: "stroke-opacity", values: "1;0.2;1", dur: "1.2s", repeatCount: "indefinite" }, ring);
      el("circle", { r: 0.09, fill: "var(--critical)" }, g);
    }
  }

  // Called every animation frame with interpolated robot poses.
  drawRobots(robots) {
    const alive = new Set();
    for (const r of robots) {
      alive.add(r.id);
      let node = this.robotNodes.get(r.id);
      if (!node) node = this._createRobot(r);
      const x = r.x, y = this.sy(r.y);
      node.group.setAttribute("transform", `translate(${x} ${y})`);
      node.body.setAttribute("transform", `rotate(${-r.heading * 180 / Math.PI})`);
      node.group.classList.toggle("dimmed", !!this.selected && this.selected !== r.id);
      const ring = STATUS_RING[r.status] || "";
      if (node.ringKind !== ring) {
        node.ring.setAttribute("class", `status-ring ${ring}`);
        node.ring.style.display = ring ? "" : "none";
        node.ringKind = ring;
      }
      node.select.style.display = this.selected === r.id ? "" : "none";

      // keep the path glued to the robot between snapshots
      const p = this.pathNodes && this.pathNodes.get(r.id);
      if (p) p.line.setAttribute("points", [[r.x, r.y], ...p.rest].map(([px, py]) => `${px},${this.sy(py)}`).join(" "));
    }
    for (const [id, node] of this.robotNodes) {
      if (!alive.has(id)) { node.group.remove(); this.robotNodes.delete(id); }
    }
  }

  _createRobot(r) {
    const group = el("g", { class: "robot", "data-id": r.id }, this.gRobots);
    const select = el("circle", { r: 0.62, fill: "none", stroke: "var(--text)", "stroke-width": 0.05, "stroke-opacity": 0.8 }, group);
    const ring = el("circle", { r: 0.5, class: "status-ring" }, group);
    const body = el("g", { class: "robot-body" }, group);
    el("circle", { r: 0.37, fill: r.color, stroke: "#0b0c0e", "stroke-width": 0.06 }, body);
    el("path", { d: "M0.4 -0.13 L0.56 0 L0.4 0.13 Z", fill: r.color, stroke: "#0b0c0e", "stroke-width": 0.03 }, body);
    const label = el("text", { y: 0.1, "font-size": 0.27, "font-weight": 800, fill: "#fff", "text-anchor": "middle" }, group);
    label.textContent = r.id;
    group.addEventListener("click", (event) => { event.stopPropagation(); this.onSelect(r.id); });
    const node = { group, body, ring, select, ringKind: null };
    this.robotNodes.set(r.id, node);
    return node;
  }

  select(id) {
    this.selected = id;
    if (this.state) this.setState(this.state);
  }

  // ------------------------------------------------------------ layers / view

  setLayer(name, visible) {
    if (visible) this.hidden.delete(name); else this.hidden.add(name);
    this._applyLayers();
  }

  _applyLayers() {
    for (const g of this.svg.querySelectorAll("[data-layer]")) {
      g.classList.toggle("layer-hidden", this.hidden.has(g.dataset.layer));
    }
  }

  fit() {
    if (!this.layout) return;
    const pad = 0.8;
    this.view = { x: -pad, y: -pad, w: this.worldW + 2 * pad, h: this.worldH + 2 * pad };
    this._applyView();
  }

  _applyView() {
    const { x, y, w, h } = this.view;
    this.svg.setAttribute("viewBox", `${x} ${y} ${w} ${h}`);
    this.svg.setAttribute("preserveAspectRatio", "xMidYMid meet");
  }

  _initPanZoom() {
    const svg = this.svg;
    let drag = null;
    svg.addEventListener("wheel", (event) => {
      if (!this.view) return;
      event.preventDefault();
      const pt = this._toWorld(event);
      const factor = Math.exp(event.deltaY * 0.0015);
      const v = this.view;
      const w = Math.min(Math.max(v.w * factor, 4), this.worldW * 3);
      const scale = w / v.w;
      this.view = { x: pt.x - (pt.x - v.x) * scale, y: pt.y - (pt.y - v.y) * scale, w, h: v.h * scale };
      this._applyView();
    }, { passive: false });
    svg.addEventListener("pointerdown", (event) => {
      if (!this.view) return;
      const pan = event.button === 1 || (event.button === 0 && (this.panWithLeft || !this.editHandler));
      if (!pan && event.button === 0 && this.editHandler) {
        const cell = this.cellAt(event);
        svg.setPointerCapture(event.pointerId);
        this.editHandler.down(cell, event);
        return;
      }
      if (!pan) return;
      if (event.button === 1) event.preventDefault();
      drag = { start: this._toWorld(event), moved: false, id: event.pointerId };
    });
    svg.addEventListener("pointermove", (event) => {
      if (!drag) {
        if (this.editHandler) this.editHandler.move(this.cellAt(event), event);
        return;
      }
      const pt = this._toWorld(event);
      const dx = pt.x - drag.start.x, dy = pt.y - drag.start.y;
      if (!drag.moved && Math.hypot(dx, dy) < 0.15) return;
      if (!drag.moved) { drag.moved = true; svg.setPointerCapture(drag.id); svg.classList.add("dragging"); }
      this.view.x -= dx; this.view.y -= dy;
      this._applyView();
    });
    const end = (event) => {
      if (!drag && this.editHandler) {
        this.editHandler.up(this.cellAt(event), event);
        return;
      }
      if (drag && !drag.moved && event.type === "pointerup" && !event.target.closest(".robot")) this.onSelect(null);
      drag = null;
      svg.classList.remove("dragging");
    };
    svg.addEventListener("pointerup", end);
    svg.addEventListener("pointercancel", end);
    svg.addEventListener("pointerleave", () => { if (!drag && this.editHandler) this.editHandler.leave(); });
    svg.addEventListener("auxclick", (event) => event.preventDefault());
  }

  // Cell under the pointer, or null outside the warehouse.
  cellAt(event) {
    if (!this.layout) return null;
    const pt = this._toWorld(event);
    const x = Math.floor(pt.x / this.cs);
    const y = Math.floor((this.worldH - pt.y) / this.cs);
    if (x < 0 || y < 0 || x >= this.layout.width || y >= this.layout.height) return null;
    return [x, y];
  }

  // Static robot start/goal markers (editor).
  drawRobotMarkers(markers) {
    this.gMarkers.innerHTML = "";
    for (const m of markers) {
      if (m.start) {
        const [x, y] = this.cellCenter(m.start[0], m.start[1]);
        const g = el("g", { transform: `translate(${x} ${this.sy(y)})` }, this.gMarkers);
        el("circle", { r: 0.34, fill: m.color, stroke: "#0b0c0e", "stroke-width": 0.05, "fill-opacity": 0.9 }, g);
        const t = el("text", { y: 0.1, "font-size": 0.26, "font-weight": 800, fill: "#fff", "text-anchor": "middle" }, g);
        t.textContent = m.id;
      }
      if (m.goal) {
        const [x, y] = this.cellCenter(m.goal[0], m.goal[1]);
        el("circle", { cx: x, cy: this.sy(y), r: 0.3, fill: "none", stroke: m.color, "stroke-width": 0.07, "stroke-dasharray": "0.14 0.1" }, this.gMarkers);
      }
    }
  }

  _toWorld(event) {
    const pt = this.svg.createSVGPoint();
    pt.x = event.clientX; pt.y = event.clientY;
    const m = this.svg.getScreenCTM();
    return m ? pt.matrixTransform(m.inverse()) : { x: 0, y: 0 };
  }
}
