// Pure layout-editing operations (no DOM), shared by the editor and its tests.
//
// Grids are arrays of rows indexed grid[y][x] with y = 0 at the BOTTOM,
// matching nexus/layout.py. Layout files store rows top-first; use
// gridFromRows / rowsFromGrid to convert.

export const FLOOR = ".";
export const WALL = "#";
export const SHELF = "S";
export const HUMAN = "H";
export const LANE = "+";

export const DIR_CHAR = { "1,0": ">", "-1,0": "<", "0,1": "^", "0,-1": "v" };
export const CHAR_DIR = { ">": [1, 0], "<": [-1, 0], "^": [0, 1], "v": [0, -1] };

const key = (x, y) => `${x},${y}`;

export function isLane(ch) {
  return ch === LANE || ch in CHAR_DIR;
}

export function gridFromRows(rows) {
  return [...rows].reverse().map((row) => row.split(""));
}

export function rowsFromGrid(grid) {
  return [...grid].reverse().map((row) => row.join(""));
}

export function cloneGrid(grid) {
  return grid.map((row) => row.slice());
}

export function inBounds(grid, x, y) {
  return y >= 0 && y < grid.length && x >= 0 && x < grid[0].length;
}

export function blankGrid(width, height, walls = true) {
  const grid = [];
  for (let y = 0; y < height; y++) {
    const row = [];
    for (let x = 0; x < width; x++) {
      const edge = x === 0 || y === 0 || x === width - 1 || y === height - 1;
      row.push(walls && edge ? WALL : FLOOR);
    }
    grid.push(row);
  }
  return grid;
}

// ------------------------------------------------------------------ shapes

// Axis-locked straight run from a toward b. Returns { cells, dir } where dir
// is the unit step along the run (null for a single cell).
export function lineCells(a, b) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  if (dx === 0 && dy === 0) return { cells: [[a[0], a[1]]], dir: null };
  const horizontal = Math.abs(dx) >= Math.abs(dy);
  const dir = horizontal ? [Math.sign(dx), 0] : [0, Math.sign(dy)];
  const length = horizontal ? Math.abs(dx) : Math.abs(dy);
  const cells = [];
  for (let i = 0; i <= length; i++) cells.push([a[0] + dir[0] * i, a[1] + dir[1] * i]);
  return { cells, dir };
}

export function rectCells(a, b) {
  const cells = [];
  const [x0, x1] = [Math.min(a[0], b[0]), Math.max(a[0], b[0])];
  const [y0, y1] = [Math.min(a[1], b[1]), Math.max(a[1], b[1])];
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) cells.push([x, y]);
  return cells;
}

// Right-hand side of travel direction d (east -> south, north -> east).
export function rightOf(d) {
  return [d[1], -d[0]];
}

// Cells of an N-lane one-way road dragged from a to b. The dragged line is
// the road's left edge; further lanes are added to the right of travel.
// Right-hand traffic: the right ceil(N/2) lanes run forward, the left
// floor(N/2) lanes run backward. Returns [{ x, y, dir }].
export function roadCells(a, b, lanes, fallbackDir = [1, 0]) {
  const { cells, dir } = lineCells(a, b);
  const d = dir || fallbackDir;
  const right = rightOf(d);
  const backward = Math.floor(lanes / 2);
  const out = [];
  for (let i = 0; i < lanes; i++) {
    const laneDir = i < backward ? [-d[0], -d[1]] : d;
    for (const [x, y] of cells) out.push({ x: x + right[0] * i, y: y + right[1] * i, dir: laneDir, lane: i });
  }
  return out;
}

// ------------------------------------------------------------------ painting

// Paint plain cells (shelf, wall, human, floor, two-way lane).
// Returns the number of cells changed.
export function paintCells(grid, cells, ch) {
  let changed = 0;
  for (const [x, y] of cells) {
    if (!inBounds(grid, x, y) || grid[y][x] === ch) continue;
    grid[y][x] = ch;
    changed++;
  }
  return changed;
}

function perpendicular(a, b) {
  return a[0] * b[0] + a[1] * b[1] === 0;
}

// Paint a one-way road and create intersections automatically:
//   * a road cell becomes a two-way junction ('+') when a crossing lane
//     ('+' or a perpendicular one-way lane) or a station touches its side;
//   * a perpendicular one-way lane just beyond either end of the road
//     becomes '+', so a road ending at another road forms a T-junction.
// stationCells: Set of "x,y" keys. Returns the number of cells changed.
export function paintRoad(grid, road, stationCells = new Set()) {
  const before = cloneGrid(grid);
  const painted = new Map();
  for (const cell of road) {
    if (!inBounds(grid, cell.x, cell.y)) continue;
    painted.set(key(cell.x, cell.y), cell);
    grid[cell.y][cell.x] = DIR_CHAR[`${cell.dir[0]},${cell.dir[1]}`];
  }

  const crossing = (x, y, d) => {
    if (!inBounds(grid, x, y)) return false;
    if (stationCells.has(key(x, y))) return true;
    const ch = before[y][x];
    if (ch === LANE) return true;
    return ch in CHAR_DIR && perpendicular(CHAR_DIR[ch], d);
  };

  for (const cell of painted.values()) {
    const d = cell.dir;
    // Overlapping a perpendicular road is a crossing; redrawing over an old
    // two-way lane is not (the new road's direction wins).
    const old = before[cell.y][cell.x];
    let junction = old in CHAR_DIR && perpendicular(CHAR_DIR[old], d);
    for (const side of [rightOf(d), [-rightOf(d)[0], -rightOf(d)[1]]]) {
      const nx = cell.x + side[0], ny = cell.y + side[1];
      if (painted.has(key(nx, ny))) continue;
      if (crossing(nx, ny, d)) junction = true;
    }
    if (junction) grid[cell.y][cell.x] = LANE;
  }

  // T-junctions at the ends of each lane.
  const lanes = new Map();
  for (const cell of painted.values()) {
    if (!lanes.has(cell.lane)) lanes.set(cell.lane, []);
    lanes.get(cell.lane).push(cell);
  }
  for (const cells of lanes.values()) {
    const d = cells[0].dir;
    const along = (c) => c.x * d[0] + c.y * d[1];
    cells.sort((p, q) => along(p) - along(q));
    for (const [end, step] of [[cells[0], -1], [cells[cells.length - 1], 1]]) {
      const nx = end.x + d[0] * step, ny = end.y + d[1] * step;
      if (!inBounds(grid, nx, ny) || painted.has(key(nx, ny))) continue;
      const ch = grid[ny][nx];
      if (ch in CHAR_DIR && perpendicular(CHAR_DIR[ch], d)) grid[ny][nx] = LANE;
    }
  }

  let changed = 0;
  for (let y = 0; y < grid.length; y++) for (let x = 0; x < grid[0].length; x++) if (grid[y][x] !== before[y][x]) changed++;
  return changed;
}

// ------------------------------------------------------------------ resize

// Resize keeping the bottom-left corner fixed. New cells are floor (or wall
// on the new outer edge when walls is true and the old edge was walled).
// Returns { grid, stations, dropped } with out-of-bounds stations removed.
export function resizeLayout(grid, stations, width, height) {
  const out = [];
  for (let y = 0; y < height; y++) {
    const row = [];
    for (let x = 0; x < width; x++) row.push(inBounds(grid, x, y) ? grid[y][x] : FLOOR);
    out.push(row);
  }
  const kept = stations.filter((s) => s.x < width && s.y < height);
  const dropped = stations.filter((s) => s.x >= width || s.y >= height).map((s) => s.id);
  return { grid: out, stations: kept, dropped };
}

// ------------------------------------------------------------------ stats

export function countCells(grid) {
  const counts = { lane: 0, oneWay: 0, junction: 0, shelf: 0, wall: 0, human: 0 };
  for (const row of grid) {
    for (const ch of row) {
      if (ch === LANE) { counts.lane++; counts.junction++; }
      else if (ch in CHAR_DIR) { counts.lane++; counts.oneWay++; }
      else if (ch === SHELF) counts.shelf++;
      else if (ch === WALL) counts.wall++;
      else if (ch === HUMAN) counts.human++;
    }
  }
  return counts;
}

// ------------------------------------------------------------------ labels

// Where to put a text label for a connected region of cells (e.g. a
// human-only zone). The label goes on the region's longest straight strip
// so it always sits on the region itself, even for rings or L-shapes whose
// bounding-box centre is empty. Returns
//   { cx, cy, length, thickness, horizontal }
// with cx, cy in cell units (cell (x, y) spans [x, x+1] x [y, y+1]).
export function labelPlacement(cells, maxThickness = 6) {
  const set = new Set(cells.map(([x, y]) => key(x, y)));
  const has = (x, y) => set.has(key(x, y));
  let best = null;

  const consider = (horizontal, a0, a1, b) => {
    // Run along the major axis from a0..a1 at minor coordinate b.
    const covers = (bb) => {
      for (let a = a0; a <= a1; a++) if (!(horizontal ? has(a, bb) : has(bb, a))) return false;
      return true;
    };
    let lo = b, hi = b;
    while (hi - lo + 1 < maxThickness && covers(hi + 1)) hi++;
    while (hi - lo + 1 < maxThickness && covers(lo - 1)) lo--;
    const length = a1 - a0 + 1;
    const thickness = hi - lo + 1;
    const mid = (a0 + a1 + 1) / 2, across = (lo + hi + 1) / 2;
    const candidate = { length, thickness, horizontal, cx: horizontal ? mid : across, cy: horizontal ? across : mid };
    const better = !best
      || length > best.length
      || (length === best.length && thickness > best.thickness)
      || (length === best.length && thickness === best.thickness && horizontal && !best.horizontal)
      || (length === best.length && thickness === best.thickness && horizontal === best.horizontal && candidate.cy > best.cy);
    if (better) best = candidate;
  };

  for (const [x, y] of cells) {
    if (!has(x - 1, y)) {           // start of a horizontal run
      let x1 = x;
      while (has(x1 + 1, y)) x1++;
      consider(true, x, x1, y);
    }
    if (!has(x, y - 1)) {           // start of a vertical run
      let y1 = y;
      while (has(x, y1 + 1)) y1++;
      consider(false, y, y1, x);
    }
  }
  return best;
}

// ------------------------------------------------------------------ walkways

export const WALKWAY = "W";

// Paint a pedestrian walkway without cutting robot roads: lane cells (and
// station cells) are left as they are and returned as `crossings`, where the
// editor places crosswalks. Returns { changed, crossings }.
export function paintWalkway(grid, cells, stationCells = new Set()) {
  let changed = 0;
  const crossings = [];
  for (const [x, y] of cells) {
    if (!inBounds(grid, x, y)) continue;
    if (isLane(grid[y][x]) || stationCells.has(key(x, y))) {
      if (isLane(grid[y][x])) crossings.push([x, y]);
      continue;
    }
    if (grid[y][x] !== WALKWAY) { grid[y][x] = WALKWAY; changed++; }
  }
  return { changed, crossings };
}

// Group cells into 4-connected clusters and return each cluster's bounding
// rectangle { x, y, w, h } (bottom-left cell + size).
export function clusterRects(cells) {
  const left = new Set(cells.map(([x, y]) => key(x, y)));
  const rects = [];
  for (const [sx, sy] of cells) {
    if (!left.has(key(sx, sy))) continue;
    left.delete(key(sx, sy));
    const stack = [[sx, sy]];
    let x0 = sx, x1 = sx, y0 = sy, y1 = sy;
    while (stack.length) {
      const [x, y] = stack.pop();
      x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const k = key(x + dx, y + dy);
        if (left.has(k)) { left.delete(k); stack.push([x + dx, y + dy]); }
      }
    }
    rects.push({ x: x0, y: y0, w: x1 - x0 + 1, h: y1 - y0 + 1 });
  }
  return rects;
}
