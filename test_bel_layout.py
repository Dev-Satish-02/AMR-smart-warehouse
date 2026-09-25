"""
BEL Warehouse (Prototype) demo layout (layouts/bel_warehouse.json).

Checks that the demo layout is clean and realistic, and that it runs:
    * validates with no errors or warnings; every robot route and return leg
      is reachable
    * every rack face is reachable by a robot (lane) and by a person (walkway)
    * every place a walkway crosses a robot road is a marked crosswalk
    * contains the plant's areas, equipment, stations and safety zones
    * 10 simulated minutes: robots stay on lanes, never touch, keep moving;
      dispatched AMRs deliver orders, recharge on their own, queue stays short

Run:  python test_bel_layout.py
"""

from collections import Counter

from nexus.catalog import object_type
from nexus.grid_simulation import GridSimulation
from nexus.layout import load_layout, object_cells

PATH = "layouts/bel_warehouse.json"
SIM_SECONDS = 600


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def main():
    print("=== BEL Warehouse (Prototype) ===")
    failures = 0
    layout = load_layout(PATH)
    issues, routes = layout.analyse()

    failures += check(layout.name == "BEL Warehouse (Prototype)", f"layout name is '{layout.name}'")
    failures += check(issues == [], f"no validation issues {[i['message'] for i in issues]}")
    fixed = [r for r in layout.robots if r["mode"] == "fixed"]
    dispatched = [r for r in layout.robots if r["mode"] == "dispatch"]
    unreachable = [rid for rid, r in routes.items() if r["outbound"] is None]
    no_return = [r["id"] for r in fixed if r.get("loop") and routes[r["id"]]["back"] is None]
    failures += check(not unreachable and not no_return and len(routes) == len(fixed),
                      f"all {len(routes)} fixed-route robots have reachable routes and return legs")
    failures += check(len(dispatched) >= 6 and len(layout.flows) >= 8,
                      f"{len(dispatched)} dispatched AMRs and {len(layout.flows)} mission flows")

    # ---------------------------------------------------------- racks: robots + people
    racks = [o for o in layout.objects if o["type"] == "rack"]
    def touches(obj, predicate):
        cells = set(object_cells(obj))
        return any(
            predicate((x + dx, y + dy))
            for x, y in cells
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if (x + dx, y + dy) not in cells and layout.in_bounds((x + dx, y + dy))
        )
    robot_side = [r["name"] for r in racks if not touches(r, layout.is_lane)]
    people_side = [r["name"] for r in racks if not touches(r, lambda c: layout.char(c) == "W")]
    failures += check(len(racks) >= 12 and not robot_side and not people_side,
                      f"all {len(racks)} racks face both a robot lane and a pedestrian walkway")

    # ---------------------------------------------------------- walkway crossings are crosswalks
    crosswalk = set()
    for obj in layout.objects:
        if obj["type"] == "crosswalk":
            crosswalk.update(object_cells(obj))
    unmarked = []
    for y in range(layout.height):
        for x in range(layout.width):
            if not layout.is_lane((x, y)):
                continue
            for dx, dy in ((0, 1), (1, 0)):
                # run of lane cells through (x, y) along this axis
                a = (x, y)
                while layout.is_lane((a[0] - dx, a[1] - dy)):
                    a = (a[0] - dx, a[1] - dy)
                b = (x, y)
                while layout.is_lane((b[0] + dx, b[1] + dy)):
                    b = (b[0] + dx, b[1] + dy)
                before, after = (a[0] - dx, a[1] - dy), (b[0] + dx, b[1] + dy)
                walk = lambda c: layout.in_bounds(c) and layout.char(c) == "W"
                if walk(before) and walk(after) and (x, y) not in crosswalk:
                    unmarked.append((x, y))
    failures += check(len(crosswalk) > 0 and not unmarked,
                      f"every walkway/road crossing is a crosswalk ({len(crosswalk)} crosswalk cells, unmarked: {unmarked[:5]})")

    # ---------------------------------------------------------- plant contents
    types = Counter(o["type"] for o in layout.objects)
    stations = Counter(s["type"] for s in layout.data["stations"])
    needed = {"rack": 12, "smt_line": 2, "test_bay": 1, "assembly_bench": 2, "packing_station": 2,
              "conveyor": 1, "control_room": 1, "forklift_parking": 1, "esd_area": 1, "kitting_area": 1,
              "inspection_area": 1, "quarantine_area": 1, "crosswalk": 4, "slow_zone": 1}
    missing = {t: n for t, n in needed.items() if types[t] < n}
    failures += check(not missing, f"plant equipment and areas present ({dict(types)})")
    failures += check(all(stations[t] >= 1 for t in ("loading", "unloading", "workstation", "charging", "parking")),
                      f"stations of every type ({dict(stations)})")
    names = [o["name"] for o in layout.objects]
    failures += check(len(names) == len(set(names)) and all(names), "every object has a unique name")
    failures += check(all(all(layout.in_bounds(c) for c in object_cells(o)) for o in layout.objects), "every object is inside the building")
    modes = {
        "dispatched": bool(dispatched),
        "loop": any(r.get("loop") for r in layout.robots),
        "one-shot": any(not r.get("loop") for r in fixed),
        "via-points": any(r.get("via") for r in layout.robots),
        "custom speed": any("max_speed" in r for r in layout.robots),
        "one-way": any(c in "<>^v" for row in layout.data["rows"] for c in row),
    }
    failures += check(all(modes.values()), f"demonstrates every routing mode {modes}")

    # ---------------------------------------------------------- simulation
    sim = GridSimulation(layout)
    off_lane = illegal = 0
    previous, stalled, worst = {}, Counter(), 0.0
    lowest = 100.0
    for _ in range(int(SIM_SECONDS / sim.time_step)):
        sim.step()
        for agent in sim.agents:
            cell = layout.world_to_cell(agent.state.position)
            if not layout.is_drivable(cell):
                off_lane += 1
            last = previous.get(agent.robot_id)
            if last is not None and last != cell and not layout.move_allowed(last, cell):
                illegal += 1
            previous[agent.robot_id] = cell
            status = sim.meta[agent.robot_id]["status"]
            stalled[agent.robot_id] = stalled[agent.robot_id] + sim.time_step if status in ("WAITING", "YIELDING") else 0.0
            worst = max(worst, stalled[agent.robot_id])
            if sim.meta[agent.robot_id]["battery"] is not None:
                lowest = min(lowest, sim.meta[agent.robot_id]["battery"])
        if sim.status != "RUNNING":
            break
    m = sim.metrics()
    failures += check(off_lane == 0 and illegal == 0, f"robots stay on lanes and obey one-way rules ({SIM_SECONDS} s)")
    failures += check(m["safety_violations"] == 0, f"no robot contact (min gap {m['min_separation']} m)")
    failures += check(m["trips_completed"] >= 20, f"steady material flow: {m['trips_completed']} trips in {SIM_SECONDS} s")
    failures += check(worst <= 60.0, f"no robot stuck for more than a minute (longest wait {worst:.1f} s)")
    f = sim.fleet.metrics()
    failures += check(f["completed"] >= 15, f"dispatched missions delivered: {f['completed']} ({f['per_hour']}/h)")
    failures += check(f["charges"] >= 1 and lowest >= layout.fleet["battery_critical"],
                      f"robots recharge on their own ({f['charges']} charges, lowest battery {lowest:.0f}%)")
    failures += check(f["queued"] <= 15, f"order queue stays short ({f['queued']} waiting, avg wait {f['avg_wait']} s)")
    errors = [e["text"] for e in sim.events if e["kind"] == "error"]
    failures += check(not errors, f"no simulation errors {errors[:3]}")

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
