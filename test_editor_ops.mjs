// Layout-editor grid operations (nexus_gui/static/js/grid-ops.js).
// Run:  node --test test_editor_ops.mjs

import test from "node:test";
import assert from "node:assert/strict";

import {
  blankGrid, gridFromRows, rowsFromGrid, lineCells, rectCells, roadCells,
  paintCells, paintRoad, resizeLayout, countCells, labelPlacement,
} from "./nexus_gui/static/js/grid-ops.js";

const rows = (grid) => rowsFromGrid(grid).join("\n");

test("rows <-> grid keeps row 0 at the top", () => {
  const grid = gridFromRows(["ab", "cd"]);
  assert.equal(grid[0][0], "c"); // y = 0 is the bottom row
  assert.deepEqual(rowsFromGrid(grid), ["ab", "cd"]);
});

test("blank grid has a wall border", () => {
  assert.equal(rows(blankGrid(4, 3)), "####\n#..#\n####");
  assert.equal(rows(blankGrid(3, 2, false)), "...\n...");
});

test("lines lock to the dominant axis", () => {
  assert.deepEqual(lineCells([1, 1], [4, 2]), { cells: [[1, 1], [2, 1], [3, 1], [4, 1]], dir: [1, 0] });
  assert.deepEqual(lineCells([2, 3], [2, 1]).dir, [0, -1]);
  assert.deepEqual(lineCells([2, 2], [2, 2]), { cells: [[2, 2]], dir: null });
  assert.equal(rectCells([3, 1], [1, 2]).length, 6);
});

test("one-lane road points along the drag", () => {
  const grid = blankGrid(6, 3, false);
  paintRoad(grid, roadCells([1, 1], [4, 1], 1));
  assert.equal(rows(grid), "......\n.>>>>.\n......");
});

test("two-lane road keeps right: eastbound lane is on the south side", () => {
  const grid = blankGrid(6, 4, false);
  paintRoad(grid, roadCells([0, 2], [5, 2], 2));
  // dragged line (y=2) is the left edge -> westbound; y=1 eastbound
  assert.equal(rows(grid), "......\n<<<<<<\n>>>>>>\n......");
});

test("two-lane road going north: northbound lane is on the east side", () => {
  const grid = blankGrid(4, 4, false);
  paintRoad(grid, roadCells([1, 0], [1, 3], 2));
  assert.equal(rows(grid), ".v^.\n.v^.\n.v^.\n.v^.");
});

test("crossing roads become two-way junctions", () => {
  const grid = blankGrid(7, 7, false);
  paintRoad(grid, roadCells([0, 4], [6, 4], 2)); // E-W road on y = 4, 3
  paintRoad(grid, roadCells([3, 0], [3, 6], 2)); // N-S road on x = 3, 4
  const r = rowsFromGrid(grid);
  // the 2x2 overlap is '+'
  for (const [x, y] of [[3, 3], [4, 3], [3, 4], [4, 4]]) assert.equal(grid[y][x], "+", `(${x},${y})`);
  // road cells away from the crossing keep their direction
  assert.equal(grid[3][0], ">");
  assert.equal(grid[4][0], "<");
  assert.equal(grid[0][3], "v");
  assert.equal(grid[0][4], "^");
  assert.equal(r.length, 7);
});

test("a road ending at another road forms a T-junction", () => {
  const grid = blankGrid(6, 5, false);
  paintRoad(grid, roadCells([4, 0], [4, 4], 1)); // northbound road x = 4
  paintRoad(grid, roadCells([0, 2], [3, 2], 1)); // eastbound road ends next to it
  assert.equal(grid[2][4], "+"); // junction created on the crossed road
  assert.equal(grid[2][3], ">");
  assert.equal(grid[1][4], "^");
});

test("a parallel road next to an existing one does not create junctions", () => {
  const grid = blankGrid(6, 4, false);
  paintRoad(grid, roadCells([0, 1], [5, 1], 1));
  paintRoad(grid, roadCells([5, 2], [0, 2], 1));
  assert.equal(rows(grid), "......\n<<<<<<\n>>>>>>\n......");
});

test("a lane cell beside a station becomes a junction so robots can enter it", () => {
  const grid = blankGrid(5, 3, false);
  paintRoad(grid, roadCells([0, 1], [4, 1], 1), new Set(["2,2"]));
  assert.equal(rows(grid), ".....\n>>+>>\n.....");
});

test("redrawing over an old two-way lane makes it one-way", () => {
  const grid = gridFromRows(["....", "++++", "...."]);
  paintRoad(grid, roadCells([0, 1], [3, 1], 1));
  assert.equal(rows(grid), "....\n>>>>\n....");
});

test("plain painting and counting", () => {
  const grid = blankGrid(5, 5);
  assert.equal(paintCells(grid, rectCells([1, 1], [2, 3]), "S"), 6);
  assert.equal(paintCells(grid, rectCells([1, 1], [2, 3]), "S"), 0); // idempotent
  paintCells(grid, [[3, 1], [3, 2]], "+");
  paintCells(grid, [[99, 99]], "#"); // out of bounds is ignored
  const c = countCells(grid);
  assert.equal(c.shelf, 6);
  assert.equal(c.lane, 2);
  assert.equal(c.wall, 16);
});

test("resize keeps the bottom-left corner and drops stations outside", () => {
  const grid = gridFromRows(["+S", "#+"]);
  const stations = [{ id: "A", x: 1, y: 1 }, { id: "B", x: 0, y: 0 }];
  const out = resizeLayout(grid, stations, 1, 3);
  assert.equal(rows(out.grid), ".\n+\n#");
  assert.deepEqual(out.stations.map((s) => s.id), ["B"]);
  assert.deepEqual(out.dropped, ["A"]);
});

// cells of every `ch` in a rows-picture (rows[0] is the top)
const cellsOf = (rows, ch = "H") => {
  const grid = gridFromRows(rows);
  const out = [];
  grid.forEach((row, y) => row.forEach((c, x) => { if (c === ch) out.push([x, y]); }));
  return out;
};
const inside = (spot, cells) => cells.some(([x, y]) => spot.cx >= x && spot.cx <= x + 1 && spot.cy >= y && spot.cy <= y + 1);

test("label for a ring-shaped zone sits on the ring, not in its empty middle", () => {
  const ring = cellsOf([
    "HHHHHHHHHH",
    "H........H",
    "H........H",
    "H........H",
    "HHHHHHHHHH",
  ]);
  const spot = labelPlacement(ring);
  assert.equal(spot.horizontal, true);
  assert.equal(spot.length, 10);
  assert.deepEqual([spot.cx, spot.cy], [5, 4.5]); // centre of the top band
  assert.ok(inside(spot, ring));
});

test("label for an L-shaped zone goes on its longest arm", () => {
  const ell = cellsOf([
    "H.....",
    "H.....",
    "H.....",
    "H.....",
    "H.....",
    "H.....",
    "HHHH..",
  ]);
  const spot = labelPlacement(ell);
  assert.equal(spot.horizontal, false); // the 7-cell vertical arm beats the 4-cell foot
  assert.equal(spot.length, 7);
  assert.deepEqual([spot.cx, spot.cy], [0.5, 3.5]);
});

test("label for a solid block is centred and knows the block thickness", () => {
  const block = cellsOf(["HHHHHH", "HHHHHH", "HHHHHH"]);
  const spot = labelPlacement(block);
  assert.deepEqual([spot.horizontal, spot.length, spot.thickness], [true, 6, 3]);
  assert.deepEqual([spot.cx, spot.cy], [3, 1.5]);
});

test("a thick frame puts the label mid-band", () => {
  const frame = cellsOf([
    "HHHHHHHH",
    "HHHHHHHH",
    "HH....HH",
    "HH....HH",
    "HHHHHHHH",
    "HHHHHHHH",
  ]);
  const spot = labelPlacement(frame);
  assert.equal(spot.thickness, 2);
  assert.equal(spot.cy, 5); // between the two top rows (y = 4 and 5)
  assert.ok(inside(spot, frame));
});
