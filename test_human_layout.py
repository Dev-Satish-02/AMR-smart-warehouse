"""
Human-aware layouts: walkways, named objects (racks, equipment, areas) and
safety zones (crosswalks, slow zones).

Checks
    * walkways and blocking equipment are never driven on; areas are
    * the planner detours around slow zones when a comparable route exists,
      and drives through them when it has to
    * robots obey zone speed limits, braking smoothly before the zone
    * validation: stations inside equipment, objects outside the map,
      unknown types, bad speed limits, duplicate names
    * /api/catalog and object save/reload

Run:  python test_human_layout.py
"""

from fastapi.testclient import TestClient

from nexus.catalog import OBJECT_TYPES
from nexus.grid_simulation import GridSimulation
from nexus.layout import Layout
from nexus_gui.server import LAYOUT_DIR, app

TMP_NAME = "_test_human_tmp"


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def corridor(objects=(), rows=None, **extra):
    """20 x 5: two parallel two-way lanes (y=1 and y=3) joined at both ends."""
    rows = rows or [
        "....................",
        "++++++++++++++++++++",
        "+..................+",
        "++++++++++++++++++++",
        "....................",
    ]
    layout = {
        "name": "Corridor", "width": 20, "height": 5, "rows": rows,
        "stations": [{"id": "A", "type": "loading", "x": 0, "y": 0},
                     {"id": "B", "type": "unloading", "x": 19, "y": 0}],
        "robots": [{"id": "R1", "start": "A", "goal": "B"}],
        "objects": list(objects),
    }
    layout.update(extra)
    return layout


def route(layout_dict):
    issues, routes = Layout(layout_dict).analyse()
    return issues, (routes.get("R1") or {}).get("outbound")


def drive(layout_dict, steps=900):
    sim = GridSimulation(Layout(layout_dict))
    samples = []
    for _ in range(steps):
        sim.step()
        agent = sim.agents[0]
        samples.append((sim._cell(agent.state.position), agent.robot.speed))
        if sim.status != "RUNNING":
            break
    return sim, samples


def main():
    print("=== NEXUS human-aware layouts ===")
    failures = 0

    # ------------------------------------------------------------ drivability
    walk = corridor(rows=[
        "....................",
        "+++++++WW+++++++++++",
        "+..................+",
        "++++++++++++++++++++",
        "....................",
    ])
    _, path = route(walk)
    failures += check(path is not None and not any(c[1] == 3 and c[0] in (7, 8) for c in path),
                      "walkway cells (W) are never on a robot route")
    failures += check(not Layout(walk).is_drivable((7, 3)), "walkway is not drivable")

    machine = {"id": "M", "type": "smt_line", "name": "SMT Line 1", "x": 5, "y": 3, "w": 3, "h": 1}
    lay = Layout(corridor([machine]))
    _, path = route(corridor([machine]))
    failures += check(not lay.is_drivable((6, 3)) and path is not None and [6, 3] not in path,
                      "blocking equipment removes its lane cells from routing")

    area = {"id": "K", "type": "kitting_area", "name": "Kitting", "x": 0, "y": 2, "w": 20, "h": 3}
    failures += check(Layout(corridor([area])).is_drivable((6, 3)), "areas are labels only: lanes inside stay drivable")

    # ------------------------------------------------------------ planner vs slow zones
    _, plain = route(corridor())
    top_used = any(c[1] == 3 for c in plain)
    slow_on_used = {"id": "Z", "type": "slow_zone", "name": "Assembly aisle", "x": 2, "y": 1 if not top_used else 3, "w": 16, "h": 1, "speed_limit": 0.3}
    _, detour = route(corridor([slow_on_used]))
    failures += check(not any(c[1] == slow_on_used["y"] and 2 <= c[0] < 18 for c in detour),
                      "planner takes the parallel lane instead of a slow zone")

    cw_both = [
        {"id": "C1", "type": "crosswalk", "name": "Crosswalk 1", "x": 9, "y": 0, "w": 2, "h": 5},
    ]
    _, through = route(corridor(cw_both))
    failures += check(through is not None and any(c[0] in (9, 10) for c in through),
                      "planner still crosses a crosswalk when every route does")

    # ------------------------------------------------------------ speed limits
    sim, samples = drive(corridor(cw_both))
    in_zone = [v for c, v in samples if c[0] in (9, 10)]
    changes = [abs(b[1] - a[1]) for a, b in zip(samples, samples[1:])]
    failures += check(sim.status == "COMPLETED", f"robot completes the trip through the crosswalk (t={sim.time:.1f}s)")
    failures += check(in_zone and max(in_zone) <= 0.33, f"robot crosses at <= 0.3 m/s (max {max(in_zone):.2f})")
    failures += check(max(changes) <= 0.3, f"speed changes smoothly (largest step {max(changes):.2f} m/s per 0.1 s)")

    fast_sim, _ = drive(corridor())
    failures += check(sim.time > fast_sim.time, f"crosswalk costs time: {sim.time:.1f}s vs {fast_sim.time:.1f}s without")

    custom = [dict(cw_both[0], speed_limit=0.15)]
    _, samples = drive(corridor(custom))
    in_zone = [v for c, v in samples if c[0] in (9, 10)]
    failures += check(max(in_zone) <= 0.18, f"per-zone speed limit override is obeyed (max {max(in_zone):.2f})")

    # ------------------------------------------------------------ validation
    buried = corridor([dict(machine, x=0, y=0, w=2, h=1)])
    msgs = [i["message"] for i in Layout(buried).validate()]
    failures += check(any("Station A is inside SMT Line 1" in m for m in msgs), "station inside equipment is an error")

    outside = corridor([dict(machine, x=18, w=5)])
    msgs = [i["message"] for i in Layout(outside).validate()]
    failures += check(any("outside the warehouse" in m for m in msgs), "object sticking out of the map is an error")

    unknown = corridor([dict(machine, type="teleporter")])
    failures += check(any("unknown object type" in i["message"] for i in Layout(unknown).validate()), "unknown object type is an error")

    bad_limit = corridor([dict(cw_both[0], speed_limit=0)])
    failures += check(any("speed limit" in i["message"] for i in Layout(bad_limit).validate()), "zero speed limit is an error")

    dupes = corridor([machine, dict(machine, id="M2", y=1)])
    failures += check(any("Two objects are named" in i["message"] for i in Layout(dupes).validate()), "duplicate names are a warning")

    # ------------------------------------------------------------ API
    tmp = LAYOUT_DIR / f"{TMP_NAME}.json"
    try:
        with TestClient(app) as client:
            cat = client.get("/api/catalog").json()
            failures += check(set(cat["object_types"]) == set(OBJECT_TYPES) and "zone" in cat["families"],
                              f"/api/catalog lists {len(cat['object_types'])} object types")
            saved = corridor([machine, area, cw_both[0]])
            saved["objects"][0]["name"] = "SMT Line 1 (Radar PCBs)"
            client.put(f"/api/layouts/{TMP_NAME}", json=saved)
            back = client.get(f"/api/layouts/{TMP_NAME}").json()["layout"]
            failures += check([o["name"] for o in back["objects"]] == ["SMT Line 1 (Radar PCBs)", "Kitting", "Crosswalk 1"]
                              and back["objects"][2]["w"] == 2, "objects and custom names survive save/reload")
    finally:
        if tmp.exists():
            tmp.unlink()

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
