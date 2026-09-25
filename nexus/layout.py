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
      "simulation": {"time_step": 0.1}
    }

Cell characters:

    .   empty floor (not drivable)
    #   wall
    S   shelf
    H   human-only area
    +   two-way robot lane
    > < ^ v   one-way robot lane (direction of travel)

Station cells are drivable, but only as a path start or goal (robots never
drive *through* a station). Robot start/goal may be a station id or an [x, y]
cell. Cell (x, y) has its centre at ((x + 0.5) * cell_size, (y + 0.5) * cell_size).
"""

from __future__ import annotations

import json
import math
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import yaml

Cell = Tuple[int, int]

FLOOR = "."
WALL = "#"
SHELF = "S"
HUMAN = "H"
LANE = "+"
ONE_WAY = {">": (1, 0), "<": (-1, 0), "^": (0, 1), "v": (0, -1)}

CELL_TYPES = {
    FLOOR: "floor",
    WALL: "wall",
    SHELF: "shelf",
    HUMAN: "human",
    LANE: "lane",
    ">": "lane",
    "<": "lane",
    "^": "lane",
    "v": "lane",
}

STATION_TYPES = ("loading", "unloading", "workstation", "charging")

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
}


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
        return self.in_bounds(cell) and CELL_TYPES.get(self.char(cell)) == "lane"

    def is_drivable(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and (self.is_station(cell) or self.is_lane(cell))

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

    def validate(self) -> List[Dict[str, str]]:
        """Human-readable issues. Severity 'error' blocks simulation."""
        issues: List[Dict[str, str]] = []

        def add(severity: str, message: str, target: str = ""):
            issues.append({"severity": severity, "message": message, "target": target})

        for station in self.data["stations"]:
            cell = (int(station["x"]), int(station["y"]))
            if not self.in_bounds(cell):
                add("error", f"Station {station['id']} is outside the warehouse", station["id"])
                continue
            if not any(self.move_allowed(cell, n) or self.move_allowed(n, cell) for n in _neighbours(cell)):
                add("warning", f"Station {station['id']} is not connected to any lane", station["id"])

        starts: Dict[Cell, str] = {}
        for robot in self.robots:
            rid = robot["id"]
            start = self.resolve_cell(robot.get("start"))
            goal = self.resolve_cell(robot.get("goal"))
            if start is None:
                add("error", f"{rid}: start is not set or unknown", rid)
            elif not self.is_drivable(start):
                add("error", f"{rid}: start {list(start)} is not a lane or station", rid)
            elif start in starts:
                add("error", f"{rid}: shares its start cell with {starts[start]}", rid)
            else:
                starts[start] = rid
            if goal is None:
                add("error", f"{rid}: goal is not set or unknown", rid)
            elif not self.is_drivable(goal):
                add("error", f"{rid}: goal {list(goal)} is not a lane or station", rid)
            for via in robot.get("via", []):
                if not self.is_drivable((int(via[0]), int(via[1]))):
                    add("error", f"{rid}: via point {list(via)} is not a lane cell", rid)

        goals: Dict[Cell, str] = {}
        for robot in self.robots:
            goal = self.resolve_cell(robot.get("goal"))
            if goal is not None and goal in goals:
                add("warning", f"{robot['id']}: shares its goal with {goals[goal]}; one will wait", robot["id"])
            elif goal is not None:
                goals[goal] = robot["id"]

        return issues

    def to_dict(self) -> Dict[str, Any]:
        return deepcopy(self.data)


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
        robots.append(robot)
    d["robots"] = robots

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
