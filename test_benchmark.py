"""
Benchmark harness and the classical stop-and-wait baseline
(nexus/benchmark.py, simulation.strategy = "stop_and_wait").

Checks the baselines behave like traditional controllers (stop-and-wait, zone lock)
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

    # ---------------------------------------------------------- crossing: stop vs flow through
    def drive(strategy, delay=0):
        sim = GridSimulation(Layout(with_strategy(crossing(delay), strategy)))
        cy = 10 + delay
        standstill, slowest, order = [], {"A": 9.0, "B": 9.0}, []
        for _ in range(800):
            sim.step()
            for agent in sim.agents:
                if sim.time > 2.0 and agent.intent != "ARRIVED":
                    slowest[agent.robot_id] = min(slowest[agent.robot_id], agent.robot.speed)
                if (10, cy) in sim._occupied(agent) and agent.robot_id not in order:
                    order.append(agent.robot_id)
            b = sim._agent("B")
            if sim.meta["B"]["status"] == "WAITING" and b.robot.speed < 1e-3:
                standstill.append(sim._cell(b.state.position))
            if sim.status != "RUNNING":
                break
        return sim, standstill, slowest, order

    sim, standstill, _, _ = drive("stop_and_wait")
    failures += check(sim.meta["B"]["stops"] >= 1 and sim.meta["A"]["stops"] == 0,
                      "stop_and_wait: braking to a standstill at the crossing counts as a full stop")
    failures += check(bool(standstill) and set(standstill) == {(10, 9)},
                      f"stop_and_wait: the robot drives up to the shared cell and stops at its entry ({sorted(set(standstill))})")
    base_time = sim.time
    sim, _, slowest, order = drive("nexus")
    m = sim.metrics()
    failures += check(sim.counters["stops"] == 0 and slowest["B"] > 0.15 and order[:1] == ["A"]
                      and m["safety_violations"] == 0 and sim.min_separation >= 1.0,
                      f"nexus: B slows (min {slowest['B']:.2f} m/s) and crosses behind A without stopping")
    failures += check(sim.time < base_time, f"nexus crossing is quicker ({sim.time:.1f} s vs {base_time:.1f} s)")
    sim = GridSimulation(Layout(crossing(4)))
    sim.run(max_steps=600)
    failures += check(sim.counters["stops"] == 0, "no full stop when the crossing is clear in time")

    # ---------------------------------------------------------- lane change / cost-aware rerouting
    sim = GridSimulation(Layout(load_layout("layouts/head_on_2.json").to_dict()))
    sim.run(max_steps=1000)
    lane = [e["text"] for e in sim.events if "changes lane" in e["text"]]
    failures += check(lane and "detour vs" in lane[0] and sim.counters["stops"] == 0 and sim.counters["backoffs"] == 0,
                      f"nexus: head-on on a two-way floor -> lane change instead of meeting and backing off ({lane[:1]})")
    sim = GridSimulation(Layout(crossing(0)))
    for _ in range(40):
        sim.step()
    a, b = sim._agent("A"), sim._agent("B")
    failures += check(not sim._detour(b, set(sim._remaining_cells(a)), 0.5, sim.time, "test")
                      and not sim._detour(b, set(sim._remaining_cells(a)), 1.5, sim.time, "test"),
                      "no detour when it costs more than the expected wait")

    # ---------------------------------------------------------- crossing order never closes a wait loop
    sim = GridSimulation(Layout(load_layout("layouts/cross_traffic_6.json").to_dict()))
    r1, r2, r3 = (sim._agent(r) for r in ("R1", "R2", "R3"))
    sim._waits_for_graph = lambda: {"R1": {"R2"}, "R2": {"R3"}, "R3": set()}
    w, l = sim._acyclic_order(r3, r1)   # R1 giving way to R3 is fine (R3 waits for nobody)
    failures += check(w is r3 and l is r1, "acyclic order kept when no loop")
    w, l = sim._acyclic_order(r1, r3)   # R3 giving way to R1 would close R1->R2->R3->R1
    failures += check(w is r3 and l is r1, "order flipped when it would close a wait loop")
    sim._waits_for_graph = lambda: {"R1": {"R3"}, "R3": {"R1"}}
    failures += check(sim._acyclic_order(r1, r3) == (None, None), "no order when both would close a loop")

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

    # ---------------------------------------------------------- zone-lock baseline
    sim = GridSimulation(Layout(with_strategy(load_layout("layouts/cross_traffic_6.json").to_dict(), "zone_lock")))
    shared_zone = False
    for _ in range(4000):
        sim.step()
        owners = {}
        for agent in sim.agents:
            if agent.robot_id in sim.yielding:
                continue
            for zone in {sim._zone(c) for c in sim._occupied(agent)} - {None}:
                if owners.setdefault(zone, agent.robot_id) != agent.robot_id:
                    shared_zone = True
        if sim.status != "RUNNING":
            break
    m = sim.metrics()
    failures += check(sim.status == "COMPLETED" and m["safety_violations"] == 0 and m["reroutes"] == 0 and not shared_zone,
                      f"zone_lock: one robot per block, no rerouting, finishes safely (t={m['time']} s)")
    failures += check(m["stops"] >= len(sim.agents), f"zone_lock: robots stop at zone control points ({m['stops']} stops)")
    run = run_one("city_grid", "zone_lock", 1)
    failures += check(run["completed"] and run["collisions"] == 0, f"zone_lock: city grid finishes ({run['completion_time']} s)")

    # ---------------------------------------------------------- space-time planning (NEXUS)
    from algorithms.spacetime import SpaceTimePlanner
    st = SpaceTimePlanner(Layout(crossing(0)))
    free = st.plan((0, 10), (20, 10), 0.0, {})
    busy = st.plan((0, 10), (20, 10), 0.0, {(10, 10): [(8.0, 14.0)]})
    failures += check(bool(free) and abs(free[-1][1] - 20.0) < 1e-6 and bool(busy) and busy[-1][1] >= 24.0 - 1e-6
                      and all(t < 8.0 or t > 14.0 for c, t in busy if c == (10, 10)),
                      f"space-time planner waits for a reserved crossing ({free[-1][1]:.1f} s free, {busy[-1][1]:.1f} s busy)")
    sim = GridSimulation(Layout(crossing(0)))
    plan = sim.st_plans.get("B")
    failures += check(plan is not None and len(plan["times"]) == len(sim._agent("B").planned_path),
                      "NEXUS robots start with a space-time plan (broadcast to peers)")

    # ---------------------------------------------------------- liveness (both strategies)
    for strategy in ("stop_and_wait", "nexus"):
        run = run_one("rack_aisles", strategy, 1)
        failures += check(run["completed"] and run["collisions"] == 0,
                          f"{strategy}: rack aisles seed 1 finishes (formerly a gridlock) in {run['completion_time']} s")

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
    failures += check(len(result["runs"]) == 6 and "Head-on swap" in table and "Cross traffic" in table
                      and "total completion time" in table and "Zone lock" in table,
                      "run_suite runs all three strategies and prints a table")
    o = result["summary"]["overall"]
    failures += check(result["summary"]["primary"] == "zone_lock" and set(o["by_baseline"]) == {"zone_lock", "stop_and_wait"}
                      and o["total_completion_reduction"] == o["by_baseline"]["zone_lock"]["total_completion_reduction"],
                      "headline is NEXUS vs zone lock; stop-and-wait reported alongside")
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "out.json"
        path.write_text(json.dumps(result))
        failures += check(json.loads(path.read_text())["summary"]["overall"]["collisions"] == 0, "results serialise to JSON")

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
