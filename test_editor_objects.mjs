// Named layout objects: racks, equipment, areas, safety zones
// (nexus_gui/static/js/doc-ops.js + catalog.js).
// Run:  node --test test_editor_objects.mjs

import test from "node:test";
import assert from "node:assert/strict";

import { CATALOG, objectCells, speedLimit } from "./nexus_gui/static/js/catalog.js";
import { gridFromRows, rowsFromGrid, paintWalkway, clusterRects, lineCells } from "./nexus_gui/static/js/grid-ops.js";
import {
  objectAt, nextObjectName, addObject, moveObject, resizeObject, renameObject, removeObject, shelvesToRacks,
  addCrosswalks,
} from "./nexus_gui/static/js/doc-ops.js";

// Same shape as /api/catalog (subset).
Object.assign(CATALOG, {
  object_types: {
    rack: { label: "Rack", family: "storage", blocking: true },
    smt_line: { label: "SMT Line", family: "production", blocking: true },
    kitting_area: { label: "Kitting Area", family: "area", blocking: false },
    crosswalk: { label: "Crosswalk", family: "zone", blocking: false, speed_limit: 0.3 },
  },
});

const doc = (w = 12, h = 8) => ({ grid: gridFromRows(Array(h).fill(".".repeat(w))), stations: [], robots: [], objects: [] });

test("racks are auto-named A-01, A-02... and continue after renames", () => {
  const d = doc();
  assert.equal(addObject(d, "rack", { x: 0, y: 0, w: 2, h: 1 }).name, "Rack A-01");
  assert.equal(addObject(d, "rack", { x: 3, y: 0, w: 2, h: 1 }).name, "Rack A-02");
  d.objects.push({ id: "X", type: "rack", name: "Rack C-07", x: 9, y: 7, w: 1, h: 1 });
  assert.equal(nextObjectName(d, "rack"), "Rack C-08");
});

test("other objects are named '<Label> n'", () => {
  const d = doc();
  const a = addObject(d, "smt_line", { x: 0, y: 2, w: 6, h: 1 });
  const b = addObject(d, "smt_line", { x: 0, y: 4, w: 6, h: 1 });
  assert.deepEqual([a.name, b.name, a.id, b.id], ["SMT Line 1", "SMT Line 2", "O1", "O2"]);
});

test("objects can be renamed to anything non-empty", () => {
  const d = doc();
  const o = addObject(d, "smt_line", { x: 0, y: 0, w: 3, h: 1 });
  assert.equal(renameObject(d, o.id, "  Line 1 (Radar boards)  "), true);
  assert.equal(o.name, "Line 1 (Radar boards)");
  assert.equal(renameObject(d, o.id, "   "), false);
  assert.equal(o.name, "Line 1 (Radar boards)");
});

test("moving and resizing stay inside the warehouse", () => {
  const d = doc(10, 6);
  const o = addObject(d, "rack", { x: 2, y: 2, w: 3, h: 2 });
  moveObject(d, o.id, 9, 9);
  assert.deepEqual([o.x, o.y], [7, 4]);
  resizeObject(d, o.id, 50, 0);
  assert.deepEqual([o.w, o.h], [10, 1]);
  assert.equal(o.x, 0);
  assert.equal(objectCells(o).length, 10);
});

test("hit testing prefers safety zones, then equipment, then areas", () => {
  const d = doc();
  const area = addObject(d, "kitting_area", { x: 0, y: 0, w: 8, h: 8 });
  const rack = addObject(d, "rack", { x: 2, y: 2, w: 2, h: 2 });
  const walk = addObject(d, "crosswalk", { x: 3, y: 0, w: 1, h: 8 });
  assert.equal(objectAt(d, 3, 3), walk);
  assert.equal(objectAt(d, 2, 2), rack);
  assert.equal(objectAt(d, 6, 6), area);
  assert.equal(objectAt(d, 10, 7), null);
  assert.equal(removeObject(d, walk.id), true);
  assert.equal(objectAt(d, 3, 3), rack);
});

test("speed limit: object override, else the type default", () => {
  assert.equal(speedLimit({ type: "crosswalk" }), 0.3);
  assert.equal(speedLimit({ type: "crosswalk", speed_limit: 0.2 }), 0.2);
  assert.equal(speedLimit({ type: "rack" }), null);
});

test("painted shelves become named racks, lettered by row band", () => {
  const d = {
    grid: gridFromRows([
      "SS.SS.",
      "SS.SS.",
      "......",
      "SSSSS.",
    ]),
    stations: [], robots: [], objects: [],
  };
  const racks = shelvesToRacks(d);
  assert.deepEqual(
    racks.map((r) => [r.name, r.x, r.y, r.w, r.h]),
    [["Rack A-01", 0, 2, 2, 2], ["Rack A-02", 3, 2, 2, 2], ["Rack B-01", 0, 0, 5, 1]],
  );
  assert.deepEqual(rowsFromGrid(d.grid), ["......", "......", "......", "......"]);
  assert.equal(shelvesToRacks(d).length, 0); // nothing left to convert
});

test("an L-shaped shelf block splits into rectangles", () => {
  const d = { grid: gridFromRows(["S..", "S..", "SSS"]), stations: [], robots: [], objects: [] };
  const racks = shelvesToRacks(d);
  const cells = racks.reduce((n, r) => n + r.w * r.h, 0);
  assert.equal(cells, 5);
  assert.equal(new Set(racks.map((r) => r.name)).size, racks.length);
});

test("a walkway painted across a road leaves the lanes and marks the crossing", () => {
  const d = {
    grid: gridFromRows([
      "..........",
      "<<<<<<<<<<",
      ">>>>>>>>>>",
      "..........",
    ]),
    stations: [], robots: [], objects: [],
  };
  const { cells } = lineCells([4, 0], [4, 3]); // vertical walkway at x = 4
  const { changed, crossings } = paintWalkway(d.grid, cells);
  assert.equal(changed, 2);
  assert.deepEqual(rowsFromGrid(d.grid), ["....W.....", "<<<<<<<<<<", ">>>>>>>>>>", "....W....."]);
  const made = addCrosswalks(d, crossings);
  assert.deepEqual(made.map((o) => [o.type, o.x, o.y, o.w, o.h, o.name]), [["crosswalk", 4, 1, 1, 2, "Crosswalk 1"]]);
  // painting the same walkway again doesn't duplicate the crosswalk
  assert.equal(addCrosswalks(d, paintWalkway(d.grid, cells).crossings).length, 0);
});

test("crossings are grouped into one crosswalk per road", () => {
  assert.deepEqual(clusterRects([[1, 1], [1, 2], [5, 1], [5, 2], [6, 1], [6, 2]]),
    [{ x: 1, y: 1, w: 1, h: 2 }, { x: 5, y: 1, w: 2, h: 2 }]);
});
