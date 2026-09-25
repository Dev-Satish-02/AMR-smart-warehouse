// Pure station / robot operations on an editor document (no DOM).
//
// A document is { grid, stations, robots } where grid[y][x] has y = 0 at the
// bottom (see grid-ops.js). Robot start/goal are a station id (string) or an
// [x, y] cell; via is a list of [x, y] lane cells.

import { CHAR_DIR, LANE, SHELF, FLOOR, inBounds, clusterRects } from "./grid-ops.js";
import { objectType } from "./catalog.js";

export const STATION_TYPES = ["loading", "unloading", "workstation", "charging", "parking"];
export const STATION_PREFIX = { loading: "L", unloading: "U", workstation: "W", charging: "C", parking: "P" };
export const STATION_NAMES = { loading: "Dock", unloading: "Ship", workstation: "Pick", charging: "Charge", parking: "Parking" };

// First free id of the form <prefix><n>.
export function nextId(existing, prefix) {
  const taken = new Set(existing);
  for (let n = 1; ; n++) if (!taken.has(`${prefix}${n}`)) return `${prefix}${n}`;
}

export function stationAt(doc, x, y) {
  return doc.stations.find((s) => s.x === x && s.y === y) || null;
}

export function resolveRef(doc, ref) {
  if (ref === null || ref === undefined) return null;
  if (typeof ref === "string") {
    const s = doc.stations.find((st) => st.id === ref);
    return s ? [s.x, s.y] : null;
  }
  return [ref[0], ref[1]];
}

// Reference for a clicked cell: the station there, else the cell itself.
export function refAt(doc, x, y) {
  const s = stationAt(doc, x, y);
  return s ? s.id : [x, y];
}

export function describeRef(doc, ref) {
  if (typeof ref === "string") {
    const s = doc.stations.find((st) => st.id === ref);
    return s ? (s.label || s.id) : `${ref} (missing)`;
  }
  return ref ? `(${ref[0]}, ${ref[1]})` : "—";
}

// One-way lane cells beside a station whose direction runs past it
// (perpendicular) can never enter or leave it: make them junctions.
// Returns the number of cells changed.
export function connectStation(grid, x, y) {
  let changed = 0;
  for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
    const nx = x + dx, ny = y + dy;
    if (!inBounds(grid, nx, ny)) continue;
    const dir = CHAR_DIR[grid[ny][nx]];
    if (dir && dir[0] * dx + dir[1] * dy === 0) {
      grid[ny][nx] = LANE;
      changed++;
    }
  }
  return changed;
}

export function addStation(doc, x, y, type) {
  const prefix = STATION_PREFIX[type] || "P";
  const id = nextId(doc.stations.map((s) => s.id), prefix);
  const number = id.slice(prefix.length);
  const station = { id, type, x, y, label: `${STATION_NAMES[type] || "Station"} ${number}` };
  doc.stations.push(station);
  connectStation(doc.grid, x, y);
  return station;
}

export function moveStation(doc, id, x, y) {
  const s = doc.stations.find((st) => st.id === id);
  if (!s || stationAt(doc, x, y)) return false;
  s.x = x;
  s.y = y;
  connectStation(doc.grid, x, y);
  return true;
}

export function removeStation(doc, id) {
  const before = doc.stations.length;
  doc.stations = doc.stations.filter((s) => s.id !== id);
  return doc.stations.length !== before;
}

// Rename a station and every robot reference to it. Returns false if the
// new id is empty or already taken.
export function renameStation(doc, oldId, newId) {
  newId = String(newId).trim();
  if (!newId || (newId !== oldId && doc.stations.some((s) => s.id === newId))) return false;
  const s = doc.stations.find((st) => st.id === oldId);
  if (!s) return false;
  s.id = newId;
  for (const r of doc.robots) {
    if (r.start === oldId) r.start = newId;
    if (r.goal === oldId) r.goal = newId;
    if (r.home === oldId) r.home = newId;
  }
  for (const f of doc.flows || []) {
    f.from = f.from.map((ref) => (ref === oldId ? newId : ref));
    f.to = f.to.map((ref) => (ref === oldId ? newId : ref));
  }
  return true;
}

export function addRobot(doc, start, goal) {
  const robot = { id: nextId(doc.robots.map((r) => r.id), "R"), start, goal, via: [] };
  doc.robots.push(robot);
  return robot;
}

// A dispatched robot waits at its home (usually a parking station) and
// takes orders from the mission flows.
export function addDispatchRobot(doc, home) {
  const robot = { id: nextId(doc.robots.map((r) => r.id), "R"), mode: "dispatch", home, start: home, via: [], battery: 100 };
  doc.robots.push(robot);
  return robot;
}

// Switch a robot between "fixed" (start -> goal route) and "dispatch".
export function setRobotMode(robot, mode) {
  if (mode === "dispatch") {
    robot.mode = "dispatch";
    robot.home = robot.start;
    robot.battery = robot.battery ?? 100;
    delete robot.goal;
    delete robot.loop;
    robot.via = [];
  } else {
    robot.mode = "fixed";
    robot.start = robot.home ?? robot.start;
    delete robot.home;
    delete robot.battery;
    robot.goal = robot.goal ?? null;
  }
  return robot;
}

// Home and start move together for dispatched robots.
export function setRobotRef(robot, field, ref) {
  if (robot.mode === "dispatch" && (field === "home" || field === "start")) {
    robot.home = ref;
    robot.start = ref;
  } else {
    robot[field] = ref;
  }
}

// ================================================================== flows

export function addFlow(doc) {
  doc.flows = doc.flows || [];
  const flow = { id: nextId(doc.flows.map((f) => f.id), "F"), name: `Flow ${doc.flows.length + 1}`, from: [], to: [], rate: 20, priority: "normal", enabled: true };
  doc.flows.push(flow);
  return flow;
}

export function removeFlow(doc, id) {
  const before = (doc.flows || []).length;
  doc.flows = (doc.flows || []).filter((f) => f.id !== id);
  return doc.flows.length !== before;
}

export function renameRobot(doc, oldId, newId) {
  newId = String(newId).trim();
  if (!newId || (newId !== oldId && doc.robots.some((r) => r.id === newId))) return false;
  const r = doc.robots.find((rb) => rb.id === oldId);
  if (!r) return false;
  r.id = newId;
  return true;
}

export function removeRobot(doc, id) {
  const before = doc.robots.length;
  doc.robots = doc.robots.filter((r) => r.id !== id);
  return doc.robots.length !== before;
}

// Via-point toggle: clicking an existing via-point removes it, otherwise the
// cell is appended. Returns "added" | "removed".
export function toggleVia(robot, x, y) {
  robot.via = robot.via || [];
  const index = robot.via.findIndex((v) => v[0] === x && v[1] === y);
  if (index >= 0) {
    robot.via.splice(index, 1);
    return "removed";
  }
  robot.via.push([x, y]);
  return "added";
}

// ================================================================== objects
//
// Objects are named rectangles { id, type, name, x, y, w, h[, speed_limit] }
// with (x, y) the bottom-left cell. Types come from the catalogue.

const FAMILY_ORDER = { zone: 0, storage: 1, production: 1, facility: 1, area: 2 };

// Topmost object on a cell: safety zones first, then equipment, then areas.
export function objectAt(doc, x, y) {
  let best = null;
  for (const obj of doc.objects || []) {
    if (x < obj.x || y < obj.y || x >= obj.x + obj.w || y >= obj.y + obj.h) continue;
    const rank = FAMILY_ORDER[objectType(obj.type).family] ?? 1;
    if (!best || rank <= best.rank) best = { obj, rank };
  }
  return best ? best.obj : null;
}

const RACK_NAME = /^Rack ([A-Z])-(\d+)$/;
const pad = (n) => String(n).padStart(2, "0");

// Next free default name: racks continue "Rack A-01, A-02, ..."; other
// types are "<Label> 1, 2, ...".
export function nextObjectName(doc, type) {
  const names = new Set((doc.objects || []).map((o) => o.name));
  if (type === "rack") {
    let letter = "A", number = 0;
    for (const name of names) {
      const m = RACK_NAME.exec(name);
      if (!m) continue;
      if (m[1] > letter || (m[1] === letter && Number(m[2]) > number)) { letter = m[1]; number = Number(m[2]); }
    }
    let n = number + 1;
    while (names.has(`Rack ${letter}-${pad(n)}`)) n++;
    return `Rack ${letter}-${pad(n)}`;
  }
  const label = objectType(type).label;
  for (let n = 1; ; n++) if (!names.has(`${label} ${n}`)) return `${label} ${n}`;
}

function clampRect(doc, rect) {
  const width = doc.grid[0].length, height = doc.grid.length;
  const w = Math.max(1, Math.min(rect.w, width));
  const h = Math.max(1, Math.min(rect.h, height));
  return {
    x: Math.max(0, Math.min(rect.x, width - w)),
    y: Math.max(0, Math.min(rect.y, height - h)),
    w, h,
  };
}

export function addObject(doc, type, rect, name = null) {
  doc.objects = doc.objects || [];
  const obj = {
    id: nextId(doc.objects.map((o) => o.id), "O"),
    type,
    name: name || nextObjectName(doc, type),
    ...clampRect(doc, rect),
  };
  doc.objects.push(obj);
  return obj;
}

export function moveObject(doc, id, x, y) {
  const obj = (doc.objects || []).find((o) => o.id === id);
  if (!obj) return false;
  Object.assign(obj, clampRect(doc, { x, y, w: obj.w, h: obj.h }));
  return true;
}

export function resizeObject(doc, id, w, h) {
  const obj = (doc.objects || []).find((o) => o.id === id);
  if (!obj) return false;
  Object.assign(obj, clampRect(doc, { x: obj.x, y: obj.y, w: Math.round(w), h: Math.round(h) }));
  return true;
}

export function renameObject(doc, id, name) {
  name = String(name).trim();
  const obj = (doc.objects || []).find((o) => o.id === id);
  if (!obj || !name) return false;
  obj.name = name;
  return true;
}

export function removeObject(doc, id) {
  const before = (doc.objects || []).length;
  doc.objects = (doc.objects || []).filter((o) => o.id !== id);
  return doc.objects.length !== before;
}

// Turn painted shelf cells into named rack objects. Each connected block is
// split into rectangles (scanning from the top-left); racks are lettered by
// row band (A = top band) and numbered left to right: Rack A-01, A-02, B-01...
// The shelf cells become floor under the new rack objects.
export function shelvesToRacks(doc) {
  const grid = doc.grid;
  const height = grid.length, width = grid[0].length;
  const used = grid.map((row) => row.map(() => false));
  const rects = [];
  for (let y = height - 1; y >= 0; y--) {
    for (let x = 0; x < width; x++) {
      if (grid[y][x] !== SHELF || used[y][x]) continue;
      let w = 1;
      while (x + w < width && grid[y][x + w] === SHELF && !used[y][x + w]) w++;
      let h = 1;
      const rowFree = (yy) => {
        for (let xx = x; xx < x + w; xx++) if (grid[yy][xx] !== SHELF || used[yy][xx]) return false;
        return true;
      };
      while (y - h >= 0 && rowFree(y - h)) h++;
      for (let yy = y - h + 1; yy <= y; yy++) for (let xx = x; xx < x + w; xx++) used[yy][xx] = true;
      rects.push({ x, y: y - h + 1, w, h, top: y });
    }
  }
  rects.sort((a, b) => b.top - a.top || a.x - b.x);
  const taken = new Set((doc.objects || []).map((o) => o.name));
  let letter = "A".charCodeAt(0) - 1, number = 0, band = null;
  const created = [];
  for (const r of rects) {
    if (r.top !== band) { band = r.top; letter++; number = 0; }
    let name;
    do { number++; name = `Rack ${String.fromCharCode(letter)}-${pad(number)}`; } while (taken.has(name));
    taken.add(name);
    created.push(addObject(doc, "rack", { x: r.x, y: r.y, w: r.w, h: r.h }, name));
    for (let yy = r.y; yy < r.y + r.h; yy++) for (let xx = r.x; xx < r.x + r.w; xx++) grid[yy][xx] = FLOOR;
  }
  return created;
}

// Where a walkway crosses robot lanes, add crosswalks (one per crossing),
// skipping lane cells already covered by a crosswalk. Returns the new objects.
export function addCrosswalks(doc, cells) {
  const covered = (x, y) => (doc.objects || []).some((o) => o.type === "crosswalk"
    && x >= o.x && y >= o.y && x < o.x + o.w && y < o.y + o.h);
  const open = cells.filter(([x, y]) => !covered(x, y));
  return clusterRects(open).map((rect) => addObject(doc, "crosswalk", rect));
}
