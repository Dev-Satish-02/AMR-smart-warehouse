"""
Benchmark harness and the classical stop-and-wait baseline
(nexus/benchmark.py, simulation.strategy = "stop_and_wait").

Checks the baseline behaves like a traditional stop-and-wait controller
(fixed priority, waits until the other robot has cleared the shared path,
never reroutes), that the full-stop metric counts, that every benchmark
scenario is valid, and that the harness reports correct comparisons.

Run:  python test_benchmark.py
"""

import json
import tempfile
from pathlib import Path

from nexus.benchmark import SCENARIOS, format_table, run_one, run_suite, summarise
from nexus.grid_simulation import GridSimulation
from nexus.layout import Layout, load_layout


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def with_strategy(layout_dict, strategy):
    d = dict(layout_dict)
    d["simulation"] = {**d.get("simulation", {}), "strategy": strategy}
    return d


def crossing(delay_cells=0):
    """Plus-shaped crossing: A drives east, B north; B starts delay_cells further back."""
    size = 21 + delay_cells
    cy = 10 + delay_cells
    grid = [["." for _ in range(size)] for _ in range(size)]
    for x in range(size):
        grid[cy][x] = ">"
    for y in range(size):
        grid[y][10] = "^"
    grid[cy][10] = "+"
    return {"name": "crossing", "width": size, "height": size, "rows": ["".join(r) for r in reversed(grid)],
            "robots": [{"id": "A", "start": [0, cy], "goal": [size - 1, cy]},
                       {"id": "B", "start": [10, 0], "goal": [10, size - 1]}]}


def main():
    print("=== NEXUS benchmark harness + stop-and-wait baseline ===")
    failures = 0

    # ---------------------------------------------------------- strategy setting
    failures += check(Layout(crossing()).simulation["strategy"] == "nexus", "default strategy is NEXUS")
    bad = with_strategy(crossing(), "teleport")
    failures += check(any("Unknown coordination strategy" in i["message"] for i in Layout(bad).validate()),
                      "unknown strategy is a validation error")

    # ---------------------------------------------------------- baseline behaviour
    sim = GridSimulation(Layout(with_strategy(load_layout("layouts/cross_traffic_6.json").to_dict(), "stop_and_wait")))
    priority_ok = released_ok = True
    seen_conflicts = 0
    previous = {}
    for _ in range(3000):
        sim.step()
        for pair, info in sim.active_conflicts.items():
            seen_conflicts += pair not in previous
            if info["loser"].robot_id < info["winner"].robot_id:
                priority_ok = False
        for pair, info in previous.items():
            # A head-on deadlock hands the wait over to the shared back-off
            # layer (the loser backs into a side cell and keeps waiting).
            if pair not in sim.active_conflicts and info["loser"].robot_id not in sim.yielding:
                winner = info["winner"]
                still = set(sim._remaining_cells(winner)) | set(sim._occupied(winner))
                if info["shared"] & still and winner.intent != "ARRIVED":
                    released_ok = False
        previous = dict(sim.active_conflicts)
        if sim.status != "RUNNING":
            break
    m = sim.metrics()
    failures += check(seen_conflicts > 0 and priority_ok, f"baseline: fixed priority by robot ID ({seen_conflicts} conflicts)")
    failures += check(released_ok, "baseline: a robot waits until the other has cleared every shared cell")
    failures += check(m["reroutes"] == 0, "baseline never reroutes")
    failures += check(sim.status == "COMPLETED" and m["safety_violations"] == 0, f"baseline finishes safely (t={m['time']} s)")

    # ---------------------------------------------------------- full-stop metric
    for strategy in ("stop_and_wait", "nexus"):
        sim = GridSimulation(Layout(with_strategy(crossing(0), strategy)))
        sim.run(max_steps=600)
        failures += check(sim.counters["stops"] >= 1 and sim.meta["B"]["stops"] >= 1 and sim.meta["A"]["stops"] == 0,
                          f"{strategy}: a robot braking to a standstill at the crossing counts as a full stop")
    sim = GridSimulation(Layout(crossing(4)))
    sim.run(max_steps=600)
    failures += check(sim.counters["stops"] == 0, "no full stop when the crossing is clear in time")

    # ---------------------------------------------------------- scenarios
    for name, scenario in SCENARIOS.items():
        layout_dict, tasks = scenario["build"](1)
        layout = Layout(layout_dict)
        errors = [i["message"] for i in layout.validate() if i["severity"] == "error"]
        reachable = all(layout.resolve_cell(b) in layout.reachable_from(layout.resolve_cell(a)) for a, b in tasks)
        again, _ = scenario["build"](1)
        failures += check(not errors and reachable and again == layout_dict,
                          f"scenario '{name}' is valid, every task is reachable, and it builds identically")
    a1 = SCENARIOS["intersection"]["build"](1)[1]
    a2 = SCENARIOS["intersection"]["build"](2)[1]
    failures += check(a1 != a2, "different seeds give different task lists")

    # ---------------------------------------------------------- harness
    run = run_one("head_on", "stop_and_wait", 1)
    keys = {"completion_time", "mean_task_time", "stops", "waiting_share", "collisions", "deadlocks", "completed"}
    failures += check(keys <= set(run) and run["completed"] and run["collisions"] == 0, "run_one reports every metric")

    fake = [
        {"scenario": "head_on", "strategy": "stop_and_wait", "seed": 1, "completed": True, "completion_time": 100.0,
         "mean_task_time": 50.0, "stops": 10, "waiting_share": 0.2, "collisions": 0, "deadlocks": 0},
        {"scenario": "head_on", "strategy": "nexus", "seed": 1, "completed": True, "completion_time": 75.0,
         "mean_task_time": 40.0, "stops": 2, "waiting_share": 0.05, "collisions": 0, "deadlocks": 0},
    ]
    s = summarise(fake)
    row = s["scenarios"][0]
    failures += check(row["reduction"]["completion_time"] == 0.25 and row["reduction"]["stops"] == 0.8
                      and s["overall"]["total_completion_reduction"] == 0.25,
                      "reduction = 1 - NEXUS / stop-and-wait (100 s → 75 s is 25%)")

    timed_out = [dict(r, scenario="cross_traffic") for r in fake]
    timed_out[1] = dict(timed_out[1], completed=False, completion_time=5400.0)
    s2 = summarise(fake + timed_out)
    failures += check(s2["overall"]["excluded_timeouts"] == ["cross_traffic"] and s2["overall"]["scenarios_counted"] == 1
                      and s2["overall"]["total_completion_reduction"] == 0.25 and s2["overall"]["timeouts"]["nexus"] == 1,
                      "scenarios with a timed-out run are reported but not counted in the headline")

    result = run_suite(["head_on", "cross_traffic"], seeds=1, jobs=1)
    table = format_table(result)
    failures += check(len(result["runs"]) == 4 and "Head-on swap" in table and "Cross traffic" in table
                      and "total task completion time reduction" in table, "run_suite runs both strategies and prints a table")
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "out.json"
        path.write_text(json.dumps(result))
        failures += check(json.loads(path.read_text())["summary"]["overall"]["collisions"] == 0, "results serialise to JSON")

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
