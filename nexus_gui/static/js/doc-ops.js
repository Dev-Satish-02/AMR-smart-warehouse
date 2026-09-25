// Pure station / robot operations on an editor document (no DOM).
//
// A document is { grid, stations, robots } where grid[y][x] has y = 0 at the
// bottom (see grid-ops.js). Robot start/goal are a station id (string) or an
// [x, y] cell; via is a list of [x, y] lane cells.

import { CHAR_DIR, LANE, inBounds } from "./grid-ops.js";

export const STATION_TYPES = ["loading", "unloading", "workstation", "charging"];
export const STATION_PREFIX = { loading: "L", unloading: "U", workstation: "W", charging: "C" };
export const STATION_NAMES = { loading: "Dock", unloading: "Ship", workstation: "Pick", charging: "Charge" };

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
  }
  return true;
}

export function addRobot(doc, start, goal) {
  const robot = { id: nextId(doc.robots.map((r) => r.id), "R"), start, goal, via: [] };
  doc.robots.push(robot);
  return robot;
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
