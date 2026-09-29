"""
Grid warehouse layout: the single source of truth for the GUI editor and the
grid simulation.

A layout is a JSON document:

    {
      "version": 1,
      "name": "Fulfillment Center A",
      "cell_size": 1.0,
      "width": 30, "height": 20,          # in cells
      "rows": ["#####...", ...],           # rows[0] is the TOP row (y = height - 1)
      "stations": [{"id": "L1", "type": "loading", "x": 1, "y": 4, "label": "Dock 1"}],
      "robots": [{"id": "R1", "start": "L1", "goal": "U2", "via": [[5, 4]]}],
      "objects": [{"id": "O1", "type": "rack", "name": "Rack A-01", "x": 4, "y": 6, "w": 6, "h": 2}],
      "simulation": {"time_step": 0.1}
    }

Cell characters:

    .   empty floor (not drivable)
    #   wall
    S   shelf
    H   human-only area
    W   pedestrian walkway (people only, robots never enter)
    +   two-way robot lane
    > < ^ v   one-way robot lane (direction of travel)

Station cells are drivable, but only as a path start or goal (robots never
drive *through* a station). Robot start/goal may be a station id or an [x, y]
cell. Cell (x, y) has its centre at ((x + 0.5) * cell_size, (y + 0.5) * cell_size).

Objects are named rectangles (see nexus/catalog.py): blocking equipment and
racks make their cells undrivable; areas are labels only; safety zones
(crosswalk, slow zone) cap robot speed on the cells they cover.
"""

from __future__ import annotations

import json
import math
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import yaml

from nexus.catalog import OBJECT_TYPES, object_type

Cell = Tuple[int, int]

FLOOR = "."
WALL = "#"
SHELF = "S"
HUMAN = "H"
WALKWAY = "W"
LANE = "+"
ONE_WAY = {">": (1, 0), "<": (-1, 0), "^": (0, 1), "v": (0, -1)}

CELL_TYPES = {
    FLOOR: "floor",
    WALL: "wall",
    SHELF: "shelf",
    HUMAN: "human",
    WALKWAY: "walkway",
    LANE: "lane",
    ">": "lane",
    "<": "lane",
    "^": "lane",
    "v": "lane",
}

STATION_TYPES = ("loading", "unloading", "workstation", "charging", "parking")

ROBOT_MODES = ("fixed", "dispatch")
PRIORITIES = ("high", "normal", "low")

# Fleet / mission settings (per layout, editable in the GUI).
DEFAULT_FLEET = {
    "generator": True,       # create orders from the mission flows
    "seed": 7,               # repeatable demos
    "battery_low": 30.0,     # % : finish the current order, then charge
    "battery_critical": 8.0, # % : abandon a pickup and charge now
    "charge_target": 95.0,   # % : back to work
    "charge_rate": 2.0,      # % per second on a charger (demo speed)
    "drain_per_m": 0.1,      # % per metre driven
    "drain_idle": 0.01,      # % per second while parked
    "service_time": 3.0,     # s to load / unload at a station
    "max_queue": 40,
}

# Categorical robot identity colours, fixed order, validated for CVD
# separation and contrast on the dark map surface. Robots are always
# labelled with their ID on the map, so colour is never the only cue.
ROBOT_COLORS = [
    "#3987e5", "#d95926", "#199e70", "#c98500",
    "#d55181", "#008300", "#9085e9", "#e66767",
]

DEFAULT_SIMULATION = {
    "time_step": 0.1,
    "max_time": 600.0,
    "max_speed": 1.0,
    "acceleration": 1.2,
    "angular_speed": 2.0,
    "turn_penalty": 1.0,
    "reservation_lookahead": 3,
    # Coordination strategy: "nexus" (default) or "stop_and_wait" (the
    # classical baseline used for benchmarking).
    "strategy": "nexus",
}

STRATEGIES = ("nexus", "stop_and_wait")


class LayoutError(ValueError):
    pass


class Layout:
    """Parsed, validated view over a layout document."""

    def __init__(self, data: Dict[str, Any]):
        self.data = normalize(data)
        d = self.data
        self.name: str = d["name"]
        self.cell_size: float = float(d["cell_size"])
        self.width: int = int(d["width"])
        self.height: int = int(d["height"])
        # grid[y][x], y = 0 is the bottom row
        self.grid: List[str] = list(reversed(d["rows"]))
        self.stations: Dict[str, Dict[str, Any]] = {s["id"]: s for s in d["stations"]}
        self.station_cells: Dict[Cell, Dict[str, Any]] = {
            (int(s["x"]), int(s["y"])): s for s in d["stations"]
        }
        self.robots: List[Dict[str, Any]] = d["robots"]
        self.simulation: Dict[str, Any] = d["simulation"]
        self.objects: List[Dict[str, Any]] = d["objects"]
        self.flows: List[Dict[str, Any]] = d["flows"]
        self.fleet: Dict[str, Any] = d["fleet"]

        # Cells covered by blocking objects, and speed caps from safety zones.
        self.blocked: Dict[Cell, Dict[str, Any]] = {}
        self.speed_caps: Dict[Cell, float] = {}
        for obj in self.objects:
            info = object_type(obj["type"])
            for cell in object_cells(obj):
                if not self.in_bounds(cell):
                    continue
                if info.get("blocking"):
                    self.blocked.setdefault(cell, obj)
                limit = obj.get("speed_limit", info.get("speed_limit"))
                if limit is not None:
                    self.speed_caps[cell] = min(float(limit), self.speed_caps.get(cell, float("inf")))

    # ------------------------------------------------------------------
    # Cells
    # ------------------------------------------------------------------

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def char(self, cell: Cell) -> str:
        x, y = cell
        return self.grid[y][x]

    def is_station(self, cell: Cell) -> bool:
        return cell in self.station_cells

    def is_lane(self, cell: Cell) -> bool:
        return (
            self.in_bounds(cell)
            and CELL_TYPES.get(self.char(cell)) == "lane"
            and cell not in self.blocked
        )

    def is_drivable(self, cell: Cell) -> bool:
        return (
            self.in_bounds(cell)
            and cell not in self.blocked
            and (self.is_station(cell) or self.is_lane(cell))
        )

    def speed_limit(self, cell: Cell) -> Optional[float]:
        """Robot speed cap (m/s) from safety zones on this cell, if any."""
        return self.speed_caps.get(cell)

    def cost_factor(self, cell: Cell) -> float:
        """Planner cost multiplier: slow cells cost as long as they take to cross."""
        limit = self.speed_caps.get(cell)
        if limit is None or limit <= 0:
            return 1.0
        return max(1.0, float(self.simulation.get("max_speed", 1.0)) / limit)

    def direction(self, cell: Cell) -> Optional[Tuple[int, int]]:
        """Allowed travel direction of a one-way lane cell, else None."""
        if not self.in_bounds(cell) or self.is_station(cell):
            return None
        return ONE_WAY.get(self.char(cell))

    def move_allowed(self, a: Cell, b: Cell) -> bool:
        """4-connected move a -> b respecting one-way lanes on both cells."""
        if not (self.is_drivable(a) and self.is_drivable(b)):
            return False
        step = (b[0] - a[0], b[1] - a[1])
        if abs(step[0]) + abs(step[1]) != 1:
            return False
        for cell in (a, b):
            required = self.direction(cell)
            if required is not None and required != step:
                return False
        return True

    def cell_center(self, cell: Cell) -> Tuple[float, float]:
        return ((cell[0] + 0.5) * self.cell_size, (cell[1] + 0.5) * self.cell_size)

    def world_to_cell(self, point) -> Cell:
        x = int(math.floor(float(point[0]) / self.cell_size))
        y = int(math.floor(float(point[1]) / self.cell_size))
        return (min(max(x, 0), self.width - 1), min(max(y, 0), self.height - 1))

    # ------------------------------------------------------------------
    # Robots
    # ------------------------------------------------------------------

    def resolve_cell(self, ref: Union[str, List[float], Tuple[float, float], None]) -> Optional[Cell]:
        """Station id or [x, y] cell -> cell."""
        if ref is None:
            return None
        if isinstance(ref, str):
            station = self.stations.get(ref)
            if station is None:
                return None
            return (int(station["x"]), int(station["y"]))
        return (int(ref[0]), int(ref[1]))

    def robot_color(self, index: int, robot: Dict[str, Any]) -> str:
        return robot.get("color") or ROBOT_COLORS[index % len(ROBOT_COLORS)]

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate(self) -> List[Dict[str, Any]]:
        """
        Human-readable issues: {severity, message, target, cells}.
        Severity 'error' blocks simulation; 'warning' does not. `cells`
        lists the [x, y] cells the editor should highlight.
        """
        return self.analyse()[0]

    def analyse(self) -> Tuple[List[Dict[str, Any]], Dict[str, Dict[str, Any]]]:
        """
        (issues, routes). routes maps robot id -> {"outbound": [[x, y], ...]
        or None, "back": [...] or None (loop robots only)}: the lane route
        the planner would drive from start through the via-points to goal.
        """
        issues: List[Dict[str, Any]] = []
        routes: Dict[str, Dict[str, Any]] = {}

        def add(severity: str, message: str, target: str = "", cells=()):
            issues.append({
                "severity": severity,
                "message": message,
                "target": target,
                "cells": [[int(c[0]), int(c[1])] for c in cells],
            })

        names: Dict[str, str] = {}
        for obj in self.objects:
            cells = object_cells(obj)
            label = obj.get("name") or obj["id"]
            if obj["type"] not in OBJECT_TYPES:
                add("error", f"{label}: unknown object type '{obj['type']}'", obj["id"])
            if not all(self.in_bounds(c) for c in cells):
                add("error", f"{label} extends outside the warehouse", obj["id"],
                    [c for c in cells if self.in_bounds(c)][:50])
            limit = obj.get("speed_limit", object_type(obj["type"]).get("speed_limit"))
            if limit is not None and not (0 < float(limit) <= 5):
                add("error", f"{label}: speed limit must be between 0 and 5 m/s", obj["id"])
            key = label.strip().lower()
            if key in names:
                add("warning", f"Two objects are named '{label}'", obj["id"])
            else:
                names[key] = obj["id"]

        for station in self.data["stations"]:
            cell = (int(station["x"]), int(station["y"]))
            if not self.in_bounds(cell):
                add("error", f"Station {station['id']} is outside the warehouse", station["id"])
                continue
            if cell in self.blocked:
                owner = self.blocked[cell]
                add("error", f"Station {station['id']} is inside {owner.get('name') or owner['id']}", station["id"], [cell])
                continue
            if not any(self.move_allowed(cell, n) or self.move_allowed(n, cell) for n in _neighbours(cell)):
                add("warning", f"Station {station['id']} is not connected to any lane", station["id"], [cell])

        # One-way lanes that robots can enter but never leave.
        dead_ends = [
            (x, y)
            for y in range(self.height)
            for x in range(self.width)
            if self.is_lane((x, y)) and not self.is_station((x, y))
            and not any(self.move_allowed((x, y), n) for n in _neighbours((x, y)))
        ]
        if dead_ends:
            add("warning", f"{len(dead_ends)} lane cell(s) are dead ends: robots can't drive out of them",
                "lanes", dead_ends[:200])

        starts: Dict[Cell, str] = {}
        routable = []
        for robot in self.robots:
            rid = robot["id"]
            start = self.resolve_cell(robot.get("start"))
            goal = self.resolve_cell(robot.get("goal"))
            ok = True
            if robot["mode"] == "dispatch":
                if start is None:
                    add("error", f"{rid}: home is not set or unknown", rid)
                elif not self.in_bounds(start) or not self.is_drivable(start):
                    add("error", f"{rid}: home {list(start)} is not a lane or station", rid, [start] if self.in_bounds(start) else [])
                elif start in starts:
                    add("error", f"{rid}: shares its home with {starts[start]}", rid, [start])
                else:
                    starts[start] = rid
                continue
            if start is None:
                add("error", f"{rid}: start is not set or unknown", rid)
                ok = False
            elif not self.in_bounds(start) or not self.is_drivable(start):
                add("error", f"{rid}: start {list(start)} is not a lane or station", rid, [start] if self.in_bounds(start) else [])
                ok = False
            elif start in starts:
                add("error", f"{rid}: shares its start cell with {starts[start]}", rid, [start])
                ok = False
            else:
                starts[start] = rid
            if goal is None:
                add("error", f"{rid}: goal is not set or unknown", rid)
                ok = False
            elif not self.in_bounds(goal) or not self.is_drivable(goal):
                add("error", f"{rid}: goal {list(goal)} is not a lane or station", rid, [goal] if self.in_bounds(goal) else [])
                ok = False
            for via in robot.get("via", []):
                cell = (int(via[0]), int(via[1]))
                if not self.in_bounds(cell) or not self.is_lane(cell):
                    add("error", f"{rid}: via point {list(via)} is not a lane cell", rid, [cell] if self.in_bounds(cell) else [])
                    ok = False
            if ok:
                routable.append((robot, start, goal))

        self._validate_flows(add)
        if self.simulation.get("strategy", "nexus") not in STRATEGIES:
            add("error", f"Unknown coordination strategy '{self.simulation.get('strategy')}' "
                f"(use {' or '.join(STRATEGIES)})", "simulation")

        goals: Dict[Cell, str] = {}
        for robot in self.robots:
            if robot["mode"] == "dispatch":
                continue
            goal = self.resolve_cell(robot.get("goal"))
            if goal is not None and goal in goals:
                add("warning", f"{robot['id']}: shares its goal with {goals[goal]}; one will wait", robot["id"], [goal])
            elif goal is not None:
                goals[goal] = robot["id"]

        # Reachability over the lane graph (one-way lanes included).
        if routable:
            from algorithms.lane_grid import LaneGrid

            grid = LaneGrid(self, turn_penalty=float(self.simulation.get("turn_penalty", 1.0)))

            def route(stops: List[Cell]) -> Optional[List[List[int]]]:
                cells: List[List[int]] = []
                for a, b in zip(stops, stops[1:]):
                    leg = grid.plan(self.cell_center(a), self.cell_center(b))
                    if not leg:
                        return None
                    leg_cells = [list(self.world_to_cell(p)) for p in leg]
                    cells.extend(leg_cells if not cells else leg_cells[1:])
                return cells

            for robot, start, goal in routable:
                via = [(int(v[0]), int(v[1])) for v in robot.get("via", [])]
                outbound = route([start] + via + [goal])
                back = route([goal, start]) if robot.get("loop") and outbound else None
                routes[robot["id"]] = {"outbound": outbound, "back": back}
                if outbound is None:
                    add("warning", f"{robot['id']}: no lane route from start to goal"
                        + (" through its via-points" if via else ""), robot["id"], [start, goal])
                elif robot.get("loop") and back is None:
                    add("warning", f"{robot['id']}: can reach its goal but has no lane route back", robot["id"], [goal, start])

        return issues, routes

    def _validate_flows(self, add):
        dispatched = [r for r in self.robots if r["mode"] == "dispatch"]
        chargers = [s for s in self.data["stations"] if s["type"] == "charging"]
        if dispatched and not chargers:
            add("warning", "Dispatched robots have no charging station to go to", "fleet")
        if self.flows and not dispatched and any(f["enabled"] for f in self.flows):
            add("warning", "Mission flows are defined but no robot is in Dispatched mode", "fleet")

        reach: Dict[Cell, set] = {}
        for flow in self.flows:
            label = flow["name"] or flow["id"]
            unknown = [ref for ref in flow["from"] + flow["to"] if ref not in self.stations]
            if unknown:
                add("error", f"Flow '{label}': unknown station(s) {', '.join(unknown)}", flow["id"])
                continue
            if not flow["from"] or not flow["to"]:
                add("error", f"Flow '{label}' needs at least one source and one destination station", flow["id"])
                continue
            missing = []
            for src in flow["from"]:
                a = self.resolve_cell(src)
                if a not in reach:
                    reach[a] = self.reachable_from(a)
                for dst in flow["to"]:
                    if dst != src and self.resolve_cell(dst) not in reach[a]:
                        missing.append(f"{src}→{dst}")
            if missing:
                add("warning", f"Flow '{label}': no lane route for {', '.join(missing[:4])}"
                    + (" …" if len(missing) > 4 else ""), flow["id"])
            if all(dst in flow["from"] for dst in flow["to"]) and len(set(flow["from"] + flow["to"])) == 1:
                add("warning", f"Flow '{label}' starts and ends at the same station", flow["id"])

    def reachable_from(self, start: Cell) -> set:
        """Cells reachable over the lane graph from start (stations are ends only)."""
        seen = {start}
        frontier = [start]
        while frontier:
            cell = frontier.pop()
            if cell != start and self.is_station(cell):
                continue
            for n in _neighbours(cell):
                if n not in seen and self.move_allowed(cell, n):
                    seen.add(n)
                    frontier.append(n)
        return seen

    def to_dict(self) -> Dict[str, Any]:
        return deepcopy(self.data)


def object_cells(obj: Dict[str, Any]) -> List[Cell]:
    """Cells covered by an object rectangle (x, y = bottom-left cell)."""
    return [
        (obj["x"] + dx, obj["y"] + dy)
        for dy in range(int(obj["h"]))
        for dx in range(int(obj["w"]))
    ]


def _neighbours(cell: Cell) -> List[Cell]:
    x, y = cell
    return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]


# ======================================================================
# Normalisation / IO
# ======================================================================

def normalize(data: Dict[str, Any]) -> Dict[str, Any]:
    d = deepcopy(data)
    d.setdefault("version", 1)
    d.setdefault("name", "Untitled warehouse")
    d.setdefault("cell_size", 1.0)
    width = int(d.get("width", 0))
    height = int(d.get("height", 0))
    if width <= 0 or height <= 0:
        raise LayoutError("Layout width and height must be positive")
    d["width"], d["height"] = width, height

    rows = d.get("rows") or [LANE * width for _ in range(height)]
    if len(rows) != height:
        raise LayoutError(f"Expected {height} rows, got {len(rows)}")
    fixed = []
    for row in rows:
        row = (row + FLOOR * width)[:width]
        fixed.append("".join(ch if ch in CELL_TYPES else FLOOR for ch in row))
    d["rows"] = fixed

    stations = []
    seen = set()
    for index, station in enumerate(d.get("stations", [])):
        station = dict(station)
        station.setdefault("id", f"P{index + 1}")
        if station["id"] in seen:
            raise LayoutError(f"Duplicate station id {station['id']}")
        seen.add(station["id"])
        station.setdefault("type", "workstation")
        station.setdefault("label", station["id"])
        station["x"], station["y"] = int(station["x"]), int(station["y"])
        stations.append(station)
    d["stations"] = stations

    robots = []
    seen = set()
    for index, robot in enumerate(d.get("robots", [])):
        robot = dict(robot)
        robot.setdefault("id", f"R{index + 1}")
        if robot["id"] in seen:
            raise LayoutError(f"Duplicate robot id {robot['id']}")
        seen.add(robot["id"])
        robot["via"] = [[int(v[0]), int(v[1])] for v in robot.get("via", [])]
        # Dispatched robots have a home (parking) and take orders; fixed-route
        # robots drive start -> goal (optionally looping / via-points).
        mode = robot.get("mode")
        if mode not in ROBOT_MODES:
            mode = "dispatch" if robot.get("home") is not None and robot.get("goal") is None else "fixed"
        robot["mode"] = mode
        if mode == "dispatch":
            if robot.get("home") is None:
                robot["home"] = robot.get("start")
            robot["start"] = robot["home"]
            robot["battery"] = max(0.0, min(100.0, float(robot.get("battery", 100.0))))
        robots.append(robot)
    d["robots"] = robots

    flows = []
    seen = set()
    for index, flow in enumerate(d.get("flows") or []):
        flow = dict(flow)
        flow.setdefault("id", f"F{index + 1}")
        if flow["id"] in seen:
            raise LayoutError(f"Duplicate flow id {flow['id']}")
        seen.add(flow["id"])
        flow.setdefault("name", flow["id"])
        for key in ("from", "to"):
            refs = flow.get(key) or []
            flow[key] = [refs] if isinstance(refs, str) else [str(r) for r in refs]
        flow["rate"] = max(0.0, float(flow.get("rate", 10.0)))
        if flow.get("priority") not in PRIORITIES:
            flow["priority"] = "normal"
        flow["enabled"] = bool(flow.get("enabled", True))
        flows.append(flow)
    d["flows"] = flows

    fleet = dict(DEFAULT_FLEET)
    fleet.update(d.get("fleet") or {})
    d["fleet"] = fleet

    objects = []
    seen = set()
    for index, obj in enumerate(d.get("objects") or []):
        obj = dict(obj)
        obj.setdefault("id", f"O{index + 1}")
        if obj["id"] in seen:
            raise LayoutError(f"Duplicate object id {obj['id']}")
        seen.add(obj["id"])
        obj.setdefault("type", "machine")
        obj.setdefault("name", object_type(obj["type"])["label"])
        for field in ("x", "y"):
            obj[field] = int(obj.get(field, 0))
        for field in ("w", "h"):
            obj[field] = max(1, int(obj.get(field, 1)))
        if "speed_limit" in obj and obj["speed_limit"] is not None:
            obj["speed_limit"] = float(obj["speed_limit"])
        objects.append(obj)
    d["objects"] = objects

    simulation = dict(DEFAULT_SIMULATION)
    simulation.update(d.get("simulation") or {})
    d["simulation"] = simulation
    return d


def load_layout(path: Union[str, Path]) -> Layout:
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        if path.suffix.lower() in {".yaml", ".yml"}:
            raw = yaml.safe_load(handle) or {}
        else:
            raw = json.load(handle)
    if "world" in raw and "rows" not in raw:
        raw = import_irsim(raw, name=path.stem)
    return Layout(raw)


def save_layout(layout: Union[Layout, Dict[str, Any]], path: Union[str, Path]) -> str:
    data = layout.to_dict() if isinstance(layout, Layout) else normalize(layout)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
        handle.write("\n")
    return str(path)


def blank_layout(width: int = 30, height: int = 20, name: str = "New warehouse") -> Dict[str, Any]:
    rows = []
    for y in range(height - 1, -1, -1):
        rows.append("".join(WALL if x in (0, width - 1) or y in (0, height - 1) else FLOOR for x in range(width)))
    return normalize({"name": name, "width": width, "height": height, "rows": rows})


# ======================================================================
# IR-SIM import
# ======================================================================

def _segment_distance(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def import_irsim(config: Dict[str, Any], name: str = "Imported warehouse") -> Dict[str, Any]:
    """
    Rasterise an IR-SIM world YAML (as used by configs/*.yaml) into a grid layout.
    Obstacle outlines become shelf cells, all other cells become two-way lanes,
    and robot starts/goals become stations.
    """
    world = config.get("world", {})
    width = int(math.ceil(float(world.get("width", 30))))
    height = int(math.ceil(float(world.get("height", 20))))

    segments = []
    for obstacle in config.get("obstacle") or []:
        for shape in obstacle.get("shape", []):
            vertices = shape.get("vertices") or []
            for a, b in zip(vertices, vertices[1:]):
                segments.append((tuple(a[:2]), tuple(b[:2])))

    grid = [[LANE for _ in range(width)] for _ in range(height)]
    for y in range(height):
        for x in range(width):
            centre = (x + 0.5, y + 0.5)
            if any(_segment_distance(centre, a, b) <= 0.5 + 1e-9 for a, b in segments):
                grid[y][x] = SHELF

    stations: List[Dict[str, Any]] = []
    station_at: Dict[Cell, str] = {}

    def station_for(point, kind):
        cell = (min(int(point[0]), width - 1), min(int(point[1]), height - 1))
        if cell not in station_at:
            sid = f"P{len(stations) + 1}"
            station_at[cell] = sid
            stations.append({"id": sid, "type": kind, "x": cell[0], "y": cell[1], "label": sid})
            grid[cell[1]][cell[0]] = LANE
        return station_at[cell]

    robots = []
    for group in config.get("robot") or []:
        states = group.get("state") or []
        goals = group.get("goal") or []
        for i, (state, goal) in enumerate(zip(states, goals)):
            robot = {
                "id": f"R{len(robots) + 1}",
                "start": station_for(state, "loading"),
                "goal": station_for(goal, "unloading"),
            }
            robots.append(robot)

    # A station that is both some robot's start and another's goal is a workstation.
    starts = {r["start"] for r in robots}
    goals_used = {r["goal"] for r in robots}
    for station in stations:
        if station["id"] in starts and station["id"] in goals_used:
            station["type"] = "workstation"

    return normalize({
        "name": name,
        "width": width,
        "height": height,
        "rows": ["".join(row) for row in reversed(grid)],
        "stations": stations,
        "robots": robots,
        "simulation": {"time_step": float(world.get("step_time", 0.1))},
    })


__all__ = [
    "Layout",
    "LayoutError",
    "STATION_TYPES",
    "CELL_TYPES",
    "load_layout",
    "save_layout",
    "blank_layout",
    "import_irsim",
    "normalize",
]
