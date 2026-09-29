"""
Benchmark: NEXUS coordination vs a classical stop-and-wait baseline on
scenarios with overlapping paths.

    python -m nexus.benchmark                 # full suite, 3 seeds
    python -m nexus.benchmark --quick         # 1 seed, small scenarios only
    python -m nexus.benchmark --scenarios intersection corridor --seeds 5

Both strategies run the SAME layout, robots and task list; only
simulation.strategy differs ("nexus" vs "stop_and_wait", see
nexus/grid_simulation.py). Both keep the same safety layer (cell
reservations, deadlock back-off), so zero collisions is required of both.

Metrics per run
    completion_time   total task completion time: when the last task finishes
    mean_task_time    average time from task creation to completion
    stops             full stops caused by traffic (braking to a standstill)
    waiting_share     share of robot time spent waiting for other robots
    collisions        safety violations (robots closer than 0.7 m)
Reduction = 1 - NEXUS / stop-and-wait (positive = NEXUS faster).
"""

from __future__ import annotations

import argparse
import json
import os
import random
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from nexus.grid_simulation import GridSimulation
from nexus.layout import Layout, load_layout

ROOT = Path(__file__).resolve().parent.parent
STRATEGIES = ("stop_and_wait", "nexus")
TIMEOUT = 5400.0  # simulated seconds


# ======================================================================
# Layout building helpers
# ======================================================================

ONE_WAY = {">": (1, 0), "<": (-1, 0), "^": (0, 1), "v": (0, -1)}


class GridBuilder:
    """Small helper to draw benchmark layouts (y = 0 at the bottom)."""

    def __init__(self, width: int, height: int, name: str):
        self.w, self.h, self.name = width, height, name
        self.cells = [["." for _ in range(width)] for _ in range(height)]
        self.stations: List[Dict[str, Any]] = []
        self.robots: List[Dict[str, Any]] = []

    def paint(self, x0, y0, x1, y1, ch):
        for y in range(min(y0, y1), max(y0, y1) + 1):
            for x in range(min(x0, x1), max(x0, x1) + 1):
                self.cells[y][x] = ch

    def station(self, sid, kind, x, y, label=None):
        """Place a station and turn perpendicular one-way neighbours into
        junctions so robots can enter and leave (same rule as the editor)."""
        self.stations.append({"id": sid, "type": kind, "x": x, "y": y, "label": label or sid})
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.w and 0 <= ny < self.h:
                d = ONE_WAY.get(self.cells[ny][nx])
                if d and d[0] * dx + d[1] * dy == 0:
                    self.cells[ny][nx] = "+"

    def layout(self, **extra) -> Dict[str, Any]:
        data = {
            "name": self.name, "width": self.w, "height": self.h,
            "rows": ["".join(r) for r in reversed(self.cells)],
            "stations": self.stations, "robots": self.robots,
            "fleet": {"generator": False, "battery_low": 5.0, "battery_critical": 1.0, "drain_per_m": 0.0, "drain_idle": 0.0},
            "simulation": {"max_time": TIMEOUT},
        }
        data.update(extra)
        return data


def _tasks(rng: random.Random, n: int, pairs: Callable[[random.Random], Tuple[str, str]]) -> List[Tuple[str, str]]:
    return [pairs(rng) for _ in range(n)]


# ======================================================================
# Scenarios
# ======================================================================

def intersection(seed: int, per_side: int = 2) -> Tuple[Dict[str, Any], List[Tuple[str, str]]]:
    """Four 2-lane roads meeting at one intersection; per_side AMRs park at
    each road end and every order goes to another side, so every trip
    crosses the centre."""
    g = GridBuilder(25, 25, "Benchmark: 4-way intersection")
    g.paint(1, 11, 23, 11, ">"); g.paint(1, 12, 23, 12, "<")      # east / west (keep right)
    g.paint(12, 1, 12, 23, "^"); g.paint(11, 1, 11, 23, "v")      # north / south
    g.paint(11, 11, 12, 12, "+")                                  # the intersection
    # turning pads at each road end so robots can reach both lanes
    g.paint(1, 11, 1, 12, "+"); g.paint(23, 11, 23, 12, "+"); g.paint(11, 1, 12, 1, "+"); g.paint(11, 23, 12, 23, "+")
    sides = {
        "W": [((0, 11), (0, 12)), [(1, 10), (1, 13), (2, 10), (2, 13)][:per_side]],
        "E": [((24, 11), (24, 12)), [(23, 10), (23, 13), (22, 10), (22, 13)][:per_side]],
        "S": [((11, 0), (12, 0)), [(10, 1), (13, 1), (10, 2), (13, 2)][:per_side]],
        "N": [((11, 24), (12, 24)), [(10, 23), (13, 23), (10, 22), (13, 22)][:per_side]],
    }
    transfer: Dict[str, List[str]] = {}
    for side, (docks, parks) in sides.items():
        transfer[side] = []
        for i, (x, y) in enumerate(docks):
            sid = f"{side}{i + 1}"
            g.station(sid, "workstation", x, y)
            transfer[side].append(sid)
        for i, (x, y) in enumerate(parks):
            pid = f"P{side}{i + 1}"
            g.station(pid, "parking", x, y)
            g.robots.append({"id": f"R{len(g.robots) + 1}", "mode": "dispatch", "home": pid})
    rng = random.Random(seed)

    def pair(r):
        a, b = r.sample(list(transfer), 2)
        return r.choice(transfer[a]), r.choice(transfer[b])

    return g.layout(), _tasks(rng, 16 * per_side, pair)


def corridor(seed: int) -> Tuple[Dict[str, Any], List[Tuple[str, str]]]:
    """Two work areas joined by a single two-way corridor (choke point) with
    one passing bay; 6 AMRs carry orders in both directions."""
    g = GridBuilder(31, 11, "Benchmark: choke-point corridor")
    for base in (2, 22):
        for c in (0, 3, 6):
            g.paint(base + c, 2, base + c, 8, "+")
        for r in (2, 5, 8):
            g.paint(base, r, base + 6, r, "+")
    g.paint(9, 5, 21, 5, "+")          # the corridor
    g.paint(15, 6, 16, 6, "+")         # passing bay
    left, right = [], []
    for i, y in enumerate((2, 5, 8)):
        g.station(f"L{i + 1}", "workstation", 1, y); left.append(f"L{i + 1}")
        g.station(f"R{i + 1}", "workstation", 29, y); right.append(f"R{i + 1}")
    for i, x in enumerate((2, 5, 8)):
        g.station(f"PL{i + 1}", "parking", x, 9)
        g.robots.append({"id": f"R{len(g.robots) + 1}", "mode": "dispatch", "home": f"PL{i + 1}"})
    for i, x in enumerate((22, 25, 28)):
        g.station(f"PR{i + 1}", "parking", x, 9)
        g.robots.append({"id": f"R{len(g.robots) + 1}", "mode": "dispatch", "home": f"PR{i + 1}"})
    rng = random.Random(seed)

    def pair(r):
        return (r.choice(left), r.choice(right)) if r.random() < 0.5 else (r.choice(right), r.choice(left))

    return g.layout(), _tasks(rng, 24, pair)


def rack_aisles(seed: int) -> Tuple[Dict[str, Any], List[Tuple[str, str]]]:
    """Rack block with one-way picking aisles and two-way cross aisles;
    8 AMRs carry picks to packing, crossing each other's aisles."""
    g = GridBuilder(25, 21, "Benchmark: rack aisles")
    for y in (1, 10, 19):
        g.paint(1, y, 23, y, "+")
    aisles = (2, 6, 10, 14, 18, 22)
    for i, x in enumerate(aisles):
        g.paint(x, 2, x, 9, "^" if i % 2 == 0 else "v")
        g.paint(x, 11, x, 18, "^" if i % 2 == 0 else "v")
    for x0 in (3, 7, 11, 15, 19):
        g.paint(x0, 2, x0 + 2, 9, "S")
        g.paint(x0, 11, x0 + 2, 18, "S")
    picks = []
    for i, x in enumerate(aisles[:-1]):
        for y in (5, 14):
            sid = f"K{len(picks) + 1}"
            g.station(sid, "workstation", x + 1, y)
            picks.append(sid)
    packs = []
    for i, x in enumerate((4, 8, 12, 16, 20)):
        g.station(f"PK{i + 1}", "unloading", x, 0)
        packs.append(f"PK{i + 1}")
    for i, x in enumerate((3, 5, 7, 9, 15, 17, 19, 21)):
        g.station(f"P{i + 1}", "parking", x, 20)
        g.robots.append({"id": f"R{len(g.robots) + 1}", "mode": "dispatch", "home": f"P{i + 1}"})
    rng = random.Random(seed)
    return g.layout(), _tasks(rng, 32, lambda r: (r.choice(picks), r.choice(packs)))


def bel_batch(seed: int) -> Tuple[Dict[str, Any], List[Tuple[str, str]]]:
    """BEL Warehouse (Prototype): a batch of 40 orders drawn from its flows."""
    layout = load_layout(ROOT / "layouts" / "bel_warehouse.json").to_dict()
    layout["fleet"] = {**layout["fleet"], "generator": False}
    layout["simulation"] = {**layout["simulation"], "max_time": TIMEOUT}
    # Fixed-route shuttles would never "finish"; the batch uses the fleet only.
    layout["robots"] = [r for r in layout["robots"] if r["mode"] == "dispatch"]
    flows = [f for f in layout["flows"] if f["enabled"]]
    rng = random.Random(seed)

    def pair(r):
        f = r.choice(flows)
        a = r.choice(f["from"])
        return a, r.choice([t for t in f["to"] if t != a])

    return layout, _tasks(rng, 40, pair)


def fixed_layout(name: str) -> Callable[[int], Tuple[Dict[str, Any], List]]:
    def build(seed: int):
        layout = load_layout(ROOT / "layouts" / f"{name}.json").to_dict()
        layout["simulation"] = {**layout["simulation"], "max_time": TIMEOUT}
        return layout, []
    return build


SCENARIOS: Dict[str, Dict[str, Any]] = {
    "intersection": {"build": intersection, "title": "4-way intersection (8 AMRs, 32 orders)", "seeded": True},
    "intersection_12": {"build": lambda seed: intersection(seed, 3), "title": "4-way intersection, busy (12 AMRs, 48 orders)", "seeded": True},
    "intersection_16": {"build": lambda seed: intersection(seed, 4), "title": "4-way intersection, peak (16 AMRs, 64 orders)", "seeded": True, "heavy": True},
    "corridor": {"build": corridor, "title": "Choke-point corridor + passing bay (6 AMRs, 24 orders)", "seeded": True},
    "rack_aisles": {"build": rack_aisles, "title": "Rack aisles with crossings (8 AMRs, 32 orders)", "seeded": True},
    "head_on": {"build": fixed_layout("head_on_2"), "title": "Head-on swap (2 robots)", "seeded": False},
    "cross_traffic": {"build": fixed_layout("cross_traffic_6"), "title": "Cross traffic (6 robots)", "seeded": False},
    "bel_batch": {"build": bel_batch, "title": "BEL Warehouse batch (9 AMRs, 40 orders)", "seeded": True, "heavy": True},
}


# ======================================================================
# Running
# ======================================================================

def run_one(scenario: str, strategy: str, seed: int) -> Dict[str, Any]:
    """Run one scenario with one strategy; returns the metrics of that run."""
    layout_dict, tasks = SCENARIOS[scenario]["build"](seed)
    layout_dict = dict(layout_dict)
    layout_dict["simulation"] = {**layout_dict.get("simulation", {}), "strategy": strategy}
    sim = GridSimulation(Layout(layout_dict))
    started = time.perf_counter()

    arrivals: Dict[str, float] = {}
    if tasks:
        for pickup, dropoff in tasks:
            sim.fleet.create(pickup, dropoff, "normal", now=0.0)
        done = lambda: sim.fleet.counters["completed"] >= len(tasks)
    else:
        done = lambda: sim.status == "COMPLETED"

    while not done() and sim.time < TIMEOUT and sim.status not in ("INVALID",):
        sim.step()
        if not tasks:
            for agent in sim.agents:
                if agent.robot_id not in arrivals and sim.meta[agent.robot_id]["trips"] >= 1:
                    arrivals[agent.robot_id] = sim.time

    if tasks:
        leads = [m.completed_at - m.created for m in sim.fleet.missions.values() if m.status == "completed"]
    else:
        leads = list(arrivals.values())
    time_in = {}
    for agent in sim.agents:
        for k, v in sim.meta[agent.robot_id]["time_in"].items():
            time_in[k] = time_in.get(k, 0.0) + v
    total = max(sum(time_in.values()), 1e-9)
    return {
        "scenario": scenario, "strategy": strategy, "seed": seed,
        "completed": bool(done()),
        "completion_time": round(sim.time, 1),
        "mean_task_time": round(sum(leads) / len(leads), 1) if leads else None,
        "tasks": len(tasks) or len(sim.agents),
        "stops": sim.counters["stops"],
        "waiting_share": round(time_in.get("waiting", 0.0) / total, 4),
        "collisions": sim.counters["safety_violations"],
        "deadlocks": sim.counters["deadlocks_resolved"],
        "reroutes": sim.counters["reroutes"],
        "negotiations": sim.counters["negotiations"],
        "wall_seconds": round(time.perf_counter() - started, 1),
    }


def _mean(values: List[float]) -> Optional[float]:
    values = [v for v in values if v is not None]
    return round(sum(values) / len(values), 2) if values else None


def summarise(runs: List[Dict[str, Any]]) -> Dict[str, Any]:
    scenarios = []
    for name in dict.fromkeys(r["scenario"] for r in runs):
        row = {"scenario": name, "title": SCENARIOS[name]["title"]}
        for strategy in STRATEGIES:
            mine = [r for r in runs if r["scenario"] == name and r["strategy"] == strategy]
            row[strategy] = {
                "runs": len(mine),
                "all_completed": all(r["completed"] for r in mine),
                "completion_time": _mean([r["completion_time"] for r in mine]),
                "mean_task_time": _mean([r["mean_task_time"] for r in mine]),
                "stops": _mean([r["stops"] for r in mine]),
                "waiting_share": _mean([r["waiting_share"] for r in mine]),
                "collisions": sum(r["collisions"] for r in mine),
                "deadlocks": _mean([r["deadlocks"] for r in mine]),
            }
        base, nexus = row["stop_and_wait"], row["nexus"]
        reduction = lambda a, b: round(1 - b / a, 4) if a and b is not None else None
        row["reduction"] = {
            "completion_time": reduction(base["completion_time"], nexus["completion_time"]),
            "mean_task_time": reduction(base["mean_task_time"], nexus["mean_task_time"]),
            "stops": reduction(base["stops"], nexus["stops"]),
        }
        scenarios.append(row)
    # Only scenarios where every run of BOTH strategies finished count toward
    # the headline: a timed-out run has no real completion time.
    valid = [s for s in scenarios if s["stop_and_wait"]["all_completed"] and s["nexus"]["all_completed"]]
    excluded = [s["scenario"] for s in scenarios if s not in valid]
    base_total = sum(s["stop_and_wait"]["completion_time"] or 0 for s in valid)
    nexus_total = sum(s["nexus"]["completion_time"] or 0 for s in valid)
    reductions = [s["reduction"]["completion_time"] for s in valid if s["reduction"]["completion_time"] is not None]
    return {
        "scenarios": scenarios,
        "overall": {
            "total_completion_reduction": round(1 - nexus_total / base_total, 4) if base_total else None,
            "mean_scenario_reduction": round(sum(reductions) / len(reductions), 4) if reductions else None,
            "worst_scenario_reduction": min(reductions) if reductions else None,
            "scenarios_counted": len(valid),
            "excluded_timeouts": excluded,
            "timeouts": {k: sum(1 for r in runs if r["strategy"] == k and not r["completed"]) for k in STRATEGIES},
            "collisions": sum(s[k]["collisions"] for s in scenarios for k in STRATEGIES),
            "target": 0.20,
        },
    }


def run_suite(names: List[str], seeds: int = 3, jobs: Optional[int] = None) -> Dict[str, Any]:
    work = []
    for name in names:
        seed_list = list(range(1, seeds + 1)) if SCENARIOS[name]["seeded"] else [1]
        for seed in seed_list:
            for strategy in STRATEGIES:
                work.append((name, strategy, seed))
    started = time.perf_counter()
    jobs = jobs or min(len(work), os.cpu_count() or 2)
    if jobs <= 1:
        runs = [run_one(*w) for w in work]
    else:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            runs = list(pool.map(run_one, *zip(*work)))
    return {"runs": runs, "summary": summarise(runs), "seeds": seeds,
            "wall_seconds": round(time.perf_counter() - started, 1)}


def format_table(result: Dict[str, Any]) -> str:
    pct = lambda v: "—" if v is None else f"{v * 100:+.1f}%"
    num = lambda v, fmt="{:.1f}": "—" if v is None else fmt.format(v)
    lines = [
        f"{'Scenario':52s} {'Stop&wait':>10s} {'NEXUS':>10s} {'Reduction':>10s} {'Stops S&W':>10s} {'Stops NX':>9s} {'Collisions':>10s}",
        "-" * 116,
    ]
    for s in result["summary"]["scenarios"]:
        b, n = s["stop_and_wait"], s["nexus"]
        flag = "" if b["all_completed"] and n["all_completed"] else "  (timeouts: not counted)"
        lines.append(
            f"{s['title'][:52]:52s} {num(b['completion_time']):>9s}s {num(n['completion_time']):>9s}s "
            f"{pct(s['reduction']['completion_time']):>10s} {num(b['stops']):>10s} {num(n['stops']):>9s} "
            f"{b['collisions'] + n['collisions']:>10d}{flag}")
    o = result["summary"]["overall"]
    lines += [
        "-" * 116,
        f"Over {o['scenarios_counted']} scenario(s) where every run finished: total task completion time reduction "
        f"{pct(o['total_completion_reduction'])}   mean per scenario {pct(o['mean_scenario_reduction'])}   "
        f"worst {pct(o['worst_scenario_reduction'])}   target {pct(o['target'])}",
        f"Collisions: {o['collisions']}   timed-out runs: stop-and-wait {o['timeouts']['stop_and_wait']}, "
        f"NEXUS {o['timeouts']['nexus']}"
        + (f"   (not counted: {', '.join(o['excluded_timeouts'])})" if o["excluded_timeouts"] else ""),
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="NEXUS vs classical stop-and-wait benchmark")
    parser.add_argument("--scenarios", nargs="*", choices=list(SCENARIOS), help="default: all")
    parser.add_argument("--seeds", type=int, default=3, help="task-list seeds per scenario (default 3)")
    parser.add_argument("--quick", action="store_true", help="1 seed, skip the heavy BEL batch")
    parser.add_argument("--jobs", type=int, default=None, help="parallel processes (default: CPU count)")
    parser.add_argument("--out", default=str(ROOT / "data" / "benchmark" / "latest.json"))
    args = parser.parse_args()
    names = args.scenarios or [n for n in SCENARIOS if not (args.quick and SCENARIOS[n].get("heavy"))]
    seeds = 1 if args.quick else args.seeds
    result = run_suite(names, seeds=seeds, jobs=args.jobs)
    print(format_table(result))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    print(f"\nSaved {out} ({result['wall_seconds']} s)")


if __name__ == "__main__":
    main()
