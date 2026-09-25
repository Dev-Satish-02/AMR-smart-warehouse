// Editor station / robot operations (nexus_gui/static/js/doc-ops.js).
// Run:  node --test test_editor_docs.mjs

import test from "node:test";
import assert from "node:assert/strict";

import { gridFromRows, rowsFromGrid } from "./nexus_gui/static/js/grid-ops.js";
import {
  nextId, stationAt, resolveRef, refAt, describeRef, connectStation, addStation, moveStation,
  removeStation, renameStation, addRobot, renameRobot, removeRobot, toggleVia,
} from "./nexus_gui/static/js/doc-ops.js";

const doc = (rows) => ({ grid: gridFromRows(rows), stations: [], robots: [] });

test("ids fill the first gap", () => {
  assert.equal(nextId([], "R"), "R1");
  assert.equal(nextId(["R1", "R2", "R4"], "R"), "R3");
  assert.equal(nextId(["L1"], "U"), "U1");
});

test("a station beside a passing one-way road turns that cell into a junction", () => {
  const d = doc([".....", ">>>>>", "....."]);
  const s = addStation(d, 2, 2, "loading"); // above the road
  assert.deepEqual([s.id, s.label, s.x, s.y], ["L1", "Dock 1", 2, 2]);
  assert.deepEqual(rowsFromGrid(d.grid), [".....", ">>+>>", "....."]);
});

test("a station at the end of a one-way road leaves the road alone", () => {
  const grid = gridFromRows(["....", ">>>.", "...."]);
  assert.equal(connectStation(grid, 3, 1), 0); // lane flows into the station
  assert.deepEqual(rowsFromGrid(grid), ["....", ">>>.", "...."]);
});

test("station ids are numbered per type", () => {
  const d = doc(["....", "...."]);
  addStation(d, 0, 0, "loading");
  addStation(d, 1, 0, "loading");
  const c = addStation(d, 2, 0, "charging");
  assert.deepEqual(d.stations.map((s) => s.id), ["L1", "L2", "C1"]);
  assert.equal(c.label, "Charge 1");
  assert.equal(stationAt(d, 1, 0).id, "L2");
  assert.equal(stationAt(d, 3, 0), null);
});

test("moving a station onto another station is refused", () => {
  const d = doc(["....", "...."]);
  addStation(d, 0, 0, "loading");
  addStation(d, 3, 0, "unloading");
  assert.equal(moveStation(d, "L1", 3, 0), false);
  assert.equal(moveStation(d, "L1", 1, 1), true);
  assert.deepEqual(resolveRef(d, "L1"), [1, 1]);
});

test("renaming a station updates robot references", () => {
  const d = doc(["....", "...."]);
  addStation(d, 0, 0, "loading");
  addStation(d, 3, 0, "unloading");
  const r = addRobot(d, "L1", "U1");
  assert.equal(renameStation(d, "L1", "U1"), false); // taken
  assert.equal(renameStation(d, "L1", "  "), false); // empty
  assert.equal(renameStation(d, "L1", "DOCK_A"), true);
  assert.equal(r.start, "DOCK_A");
  assert.equal(r.goal, "U1");
  assert.equal(describeRef(d, "DOCK_A"), "Dock 1");
});

test("removing a station leaves robots pointing at a missing station", () => {
  const d = doc(["...."]);
  addStation(d, 0, 0, "loading");
  addRobot(d, "L1", [3, 0]);
  assert.equal(removeStation(d, "L1"), true);
  assert.equal(resolveRef(d, "L1"), null);
  assert.equal(describeRef(d, "L1"), "L1 (missing)");
  assert.equal(describeRef(d, [3, 0]), "(3, 0)");
});

test("refAt prefers the station on a cell", () => {
  const d = doc(["...."]);
  addStation(d, 1, 0, "workstation");
  assert.equal(refAt(d, 1, 0), "W1");
  assert.deepEqual(refAt(d, 2, 0), [2, 0]);
});

test("robots: add, rename, remove", () => {
  const d = doc(["...."]);
  const a = addRobot(d, [0, 0], [3, 0]);
  const b = addRobot(d, [1, 0], [2, 0]);
  assert.deepEqual([a.id, b.id], ["R1", "R2"]);
  assert.equal(renameRobot(d, "R2", "R1"), false);
  assert.equal(renameRobot(d, "R2", "AMR-7"), true);
  assert.equal(removeRobot(d, "R1"), true);
  assert.deepEqual(d.robots.map((r) => r.id), ["AMR-7"]);
  assert.equal(addRobot(d, [0, 0], [1, 0]).id, "R1"); // gap reused
});

test("via-points toggle in click order", () => {
  const r = { id: "R1", via: [] };
  assert.equal(toggleVia(r, 2, 3), "added");
  assert.equal(toggleVia(r, 5, 3), "added");
  assert.equal(toggleVia(r, 7, 1), "added");
  assert.equal(toggleVia(r, 5, 3), "removed");
  assert.deepEqual(r.via, [[2, 3], [7, 1]]);
});
