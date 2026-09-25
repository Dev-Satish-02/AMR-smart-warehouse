// Builds layouts/bel_warehouse.json: "BEL Warehouse (Prototype)", a generic
// electronics-manufacturing plant for AMR fleet demos.
//
//   node tools/build_bel_layout.mjs
//
// Uses the editor's own operations (roads, junctions, walkway crosswalks,
// station hook-up), so the result follows exactly the editor's rules. After
// generating, edit and rename freely in the GUI. Re-running this script
// OVERWRITES layouts/bel_warehouse.json with the original version.
//
// Coordinates: 1 cell = 1 m, (0, 0) is the bottom-left (south-west) corner.

import { execFileSync } from "node:child_process";
import { writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import { CATALOG } from "../nexus_gui/static/js/catalog.js";
import * as ops from "../nexus_gui/static/js/grid-ops.js";
import * as docs from "../nexus_gui/static/js/doc-ops.js";

const ROOT = join(dirname(fileURLToPath(import.meta.url)), "..");
const PYTHON = join(ROOT, ".venv", "bin", "python");

// Object catalogue straight from nexus/catalog.py.
Object.assign(CATALOG, JSON.parse(execFileSync(PYTHON, ["-c",
  "import json; from nexus.catalog import catalog; print(json.dumps(catalog()))"], { cwd: ROOT }).toString()));

const W = 64, H = 40;
const doc = { grid: ops.blankGrid(W, H, true), stations: [], robots: [], objects: [] };
const g = doc.grid;
const stationCells = () => new Set(doc.stations.map((s) => `${s.x},${s.y}`));

// ---------------------------------------------------------------- helpers

const rect = (x0, y0, x1, y1) => ops.rectCells([x0, y0], [x1, y1]);
const paint = (cells, ch) => ops.paintCells(g, cells, ch);
const road = (a, b, lanes = 2) => ops.paintRoad(g, ops.roadCells(a, b, lanes), stationCells());
const walkway = (a, b) => {
  const cells = a[0] === b[0] || a[1] === b[1] ? ops.lineCells(a, b).cells : rect(a[0], a[1], b[0], b[1]);
  const { crossings } = ops.paintWalkway(g, cells, stationCells());
  docs.addCrosswalks(doc, crossings);
};
const object = (type, name, x0, y0, x1, y1, extra = {}) => {
  const obj = docs.addObject(doc, type, { x: x0, y: y0, w: x1 - x0 + 1, h: y1 - y0 + 1 }, name);
  Object.assign(obj, extra);
  return obj;
};
const station = (id, type, label, x, y) => {
  if (g[y][x] === ops.WALL) g[y][x] = ops.FLOOR;
  doc.stations.push({ id, type, x, y, label });
  docs.connectStation(g, x, y);
};
const robot = (id, start, goal, extra = {}) => {
  const r = { id, start, via: [], ...extra };
  if (goal !== undefined) r.goal = goal;
  doc.robots.push(r);
};

// ---------------------------------------------------------------- robot roads
//
// Two-lane roads (keep right): south and north mains, five north-south
// cross roads, and a spur to the dispatch docks. Crossings become junctions.

road([2, 5], [54, 5]);        // south main: y=5 westbound, y=4 eastbound
road([54, 32], [2, 32]);      // north main: y=32 eastbound, y=33 westbound
for (const x of [2, 12, 31, 38, 53]) road([x, 4], [x, 33]); // cross roads x / x+1
road([53, 9], [61, 9]);       // dispatch spur: y=9 westbound, y=8 eastbound
paint(rect(60, 8, 61, 9), ops.LANE); // turning pad at the dock end

// Component stores: one-way single-lane aisles between rack columns,
// alternating north / south so robots circulate without head-on traffic.
road([16, 6], [16, 31], 1);   // aisle 1 northbound
road([20, 31], [20, 6], 1);   // aisle 2 southbound
road([24, 6], [24, 31], 1);   // aisle 3 northbound
road([28, 31], [28, 6], 1);   // aisle 4 southbound

// ---------------------------------------------------------------- racks
//
// Rack columns sit between a robot aisle and a pedestrian walkway, so every
// rack face is reachable by both AMRs and pickers. Two racks per column with
// a pick gap at y = 18-19.

const rackColumns = [15, 17, 19, 21, 23, 25, 27, 29];
rackColumns.forEach((x, i) => {
  const letter = String.fromCharCode(65 + i);
  object("rack", `Rack ${letter}-01`, x, 8, x, 17);
  object("rack", `Rack ${letter}-02`, x, 20, x, 29);
});

// ---------------------------------------------------------------- stations

// Receiving docks on the west wall.
[10, 16, 22, 28].forEach((y, i) => station(`RD${i + 1}`, "loading", `Receiving Dock ${i + 1}`, 1, y));
station("INS", "workstation", "Inspection Bay", 11, 25);
station("QRN", "workstation", "Quarantine Cage", 11, 9);

// Stores pick points in the rack gaps, one per aisle.
station("STA", "workstation", "Stores Pick A", 15, 18);
station("STB", "workstation", "Stores Pick B", 21, 19);
station("STC", "workstation", "Stores Pick C", 23, 18);
station("STD", "workstation", "Stores Pick D", 29, 19);

station("KIN", "workstation", "Kitting In", 33, 12);
station("KOUT", "workstation", "Kitting Out", 33, 22);

station("S1IN", "workstation", "SMT-1 Feeder", 40, 27);
station("S1OUT", "workstation", "SMT-1 Output", 52, 26);
station("S2IN", "workstation", "SMT-2 Feeder", 40, 22);
station("S2OUT", "workstation", "SMT-2 Output", 52, 21);
station("ASM", "workstation", "Assembly Cell", 40, 15);
station("QCIN", "workstation", "QC Intake", 40, 8);
station("QCOUT", "workstation", "QC Release", 52, 11);

station("PK1", "workstation", "Packing In 1", 55, 24);
station("PK2", "workstation", "Packing In 2", 55, 16);
station("FG", "workstation", "Finished Goods", 55, 29);
station("DD1", "unloading", "Dispatch Dock 1", 62, 8);
station("DD2", "unloading", "Dispatch Dock 2", 62, 9);

[16, 18, 20, 22].forEach((x, i) => station(`CH${i + 1}`, "charging", `Charger ${i + 1}`, x, 3));
// AMR parking bays (dispatched robots' homes) next to the charging bay.
for (let i = 0; i < 10; i++) station(`P${i + 1}`, "parking", `Parking ${i + 1}`, 26 + i, 3);

// ---------------------------------------------------------------- walkways
//
// A 2 m pedestrian spine along the north of the building, walkways beside
// every rack column and through kitting / production. Where they cross robot
// roads the editor rule adds crosswalks automatically.

walkway([1, 34], [62, 35]);                       // north spine
for (const x of [14, 18, 22, 26, 30]) walkway([x, 7], [x, 33]); // stores pick faces
walkway([37, 6], [37, 33]);                       // kitting operators
walkway([41, 7], [41, 30]);                       // production operators (west)
walkway([42, 24], [51, 24]);                      // between SMT lines
walkway([42, 18], [51, 18]);                      // between SMT-2 and assembly
walkway([42, 12], [51, 12]);                      // between assembly and test
walkway([56, 26], [61, 26]);                      // packing operators

// ---------------------------------------------------------------- equipment & areas

// Receiving
object("inspection_area", "Incoming Inspection", 4, 20, 11, 30);
object("test_bay", "Inspection Bench", 5, 23, 9, 26);
object("quarantine_area", "Quarantine", 4, 7, 11, 13);
object("stores_cage", "Quarantine Cage", 5, 8, 9, 11);

// Kitting
object("kitting_area", "Kitting", 33, 6, 37, 31);
object("assembly_bench", "Kitting Bench 1", 35, 9, 36, 14);
object("assembly_bench", "Kitting Bench 2", 35, 19, 36, 25);

// Production (ESD protected)
object("esd_area", "ESD Protected Area", 40, 6, 52, 31);
object("smt_line", "SMT Line 1", 42, 26, 51, 27);
object("smt_line", "SMT Line 2", 42, 21, 51, 22);
object("assembly_bench", "Assembly Bench 1", 42, 14, 45, 16);
object("assembly_bench", "Assembly Bench 2", 48, 14, 51, 16);
object("test_bay", "Test & QC Bay", 42, 7, 51, 10);

// Packing & dispatch
object("packing_station", "Packing Station 1", 57, 22, 59, 25);
object("packing_station", "Packing Station 2", 57, 14, 59, 17);
object("conveyor", "Outbound Conveyor", 57, 19, 61, 20);
object("stores_cage", "Finished Goods Store", 57, 28, 61, 31);
object("staging_area", "Dispatch Staging", 55, 6, 62, 12);

// Service band (south)
object("staging_area", "AMR Charging Bay", 14, 1, 24, 3);
object("staging_area", "AMR Parking", 26, 1, 35, 3);
object("forklift_parking", "Forklift Parking", 40, 1, 47, 3);
object("machine", "Maintenance Workshop", 50, 1, 58, 3);

// North band: offices and people areas
object("control_room", "Fleet Control Room", 2, 36, 9, 38);
object("office", "Production Office", 11, 36, 19, 38);
paint(rect(21, 36, 27, 38), ops.HUMAN);            // break area (people only)
object("office", "Training Room", 29, 36, 36, 38);
object("test_bay", "Quality Lab", 38, 36, 47, 38);
object("office", "Stores Office", 49, 36, 55, 38);
object("office", "Security", 57, 36, 62, 38);

// Safety zones on robot roads near people
object("slow_zone", "Receiving Apron", 2, 8, 3, 30, { speed_limit: 0.5 });
object("slow_zone", "ESD Feeder Road", 38, 19, 39, 29, { speed_limit: 0.5 });

// ---------------------------------------------------------------- robots
//
// Nine dispatched AMRs take transport orders from the mission flows below
// (battery levels vary so charging shows up early in a demo). Three
// fixed-route robots show the other modes: a plain loop, a loop through a
// via-point, and a one-shot trip.

const batteries = [100, 92, 85, 78, 70, 64, 55, 40, 33];
batteries.forEach((battery, i) => robot(`R${i + 1}`, `P${i + 1}`, undefined, {
  mode: "dispatch", home: `P${i + 1}`, battery,
  ...(i === 4 ? { max_speed: 0.8 } : {}),   // R5: slower heavy-load AMR
}));
robot("R10", "FG", "DD2", { loop: true });                         // finished-goods shuttle
robot("R11", "RD4", "ASM", { loop: true, via: [[32, 30]] });        // receiving -> assembly via the north side
robot("R12", "CH4", "P10");                                         // one-shot: charger -> parking

// ---------------------------------------------------------------- mission flows
//
// Transport orders along the material flow (orders per hour).

const flows = [
  // ~145 orders/hour: about 80% of what nine AMRs deliver here, so queues stay short.
  ["Inbound receipt", ["RD1", "RD2", "RD3"], ["INS"], 20, "normal"],
  ["Put-away to stores", ["INS"], ["STA", "STB", "STC", "STD"], 20, "normal"],
  ["Kit picking", ["STA", "STB", "STC", "STD"], ["KIN"], 20, "normal"],
  ["SMT line feeding", ["KOUT"], ["S1IN", "S2IN"], 22, "high"],
  ["Assembly feeding", ["KOUT"], ["ASM"], 8, "normal"],
  ["WIP to test", ["S1OUT", "S2OUT"], ["QCIN"], 18, "normal"],
  ["QC release to packing", ["QCOUT"], ["PK1", "PK2"], 14, "normal"],
  ["QC rejects to quarantine", ["QCOUT"], ["QRN"], 3, "low"],
  ["Packed goods to store", ["PK1", "PK2"], ["FG"], 10, "low"],
  ["Outbound dispatch", ["FG"], ["DD1"], 10, "normal"],
].map(([name, from, to, rate, priority], i) => ({ id: `F${i + 1}`, name, from, to, rate, priority, enabled: true }));

// ---------------------------------------------------------------- write

const layout = {
  version: 1,
  name: "BEL Warehouse (Prototype)",
  cell_size: 1,
  width: W,
  height: H,
  rows: ops.rowsFromGrid(g),
  stations: doc.stations,
  robots: doc.robots,
  objects: doc.objects,
  flows,
  fleet: { generator: true, seed: 42, charge_rate: 2.5 },
  simulation: { max_time: 3600 },
};
const out = join(ROOT, "layouts", "bel_warehouse.json");
writeFileSync(out, JSON.stringify(layout, null, 2) + "\n");
console.log(`wrote ${out}: ${doc.objects.length} objects, ${doc.stations.length} stations, ${doc.robots.length} robots, ${flows.length} flows`);
