"""
Robots, stations and routes as edited in the layout editor.

Covers what step 3 of the editor relies on:
    * Layout.analyse() returns each robot's planned lane route, through its
      via-points in order, plus the return leg for looping robots
    * /api/validate returns those routes for the live preview
    * robot settings (via, loop, max_speed) and station labels survive save/reload
    * the simulation actually drives through the via-points, in order, and
      respects a robot's max speed

Run:  python test_editor_routes.py
"""

from fastapi.testclient import TestClient

from nexus.grid_simulation import GridSimulation
from nexus.layout import Layout
from nexus_gui.server import LAYOUT_DIR, app

TMP_NAME = "_test_routes_tmp"


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def loop_layout(**robot):
    """
    11 x 7: a one-way ring road (clockwise) with a two-way spine through
    the middle, docks on the left and right.
        y=5  +>>>>+>>>>+
        y=3  ^....+....v     (spine at x=5 is two-way)
        y=1  +<<<<+<<<<+
    """
    rows = [
        "...........",
        "+>>>>+>>>>+",
        "^....+....v",
        "^....+....v",
        "^....+....v",
        "+<<<<+<<<<+",
        "...........",
    ]
    base = {"id": "R1", "start": "A", "goal": "B"}
    base.update(robot)
    return {
        "name": "Ring", "width": 11, "height": 7, "rows": rows,
        "stations": [
            {"id": "A", "type": "loading", "x": 0, "y": 6, "label": "Dock A"},
            {"id": "B", "type": "unloading", "x": 10, "y": 0, "label": "Ship B"},
        ],
        "robots": [base],
    }


def main():
    print("=== NEXUS editor: robots, stations, routes ===")
    failures = 0

    # ------------------------------------------------------------ planning
    issues, routes = Layout(loop_layout()).analyse()
    outbound = routes["R1"]["outbound"]
    failures += check(not issues, f"ring layout has no issues {issues}")
    failures += check(outbound[0] == [0, 6] and outbound[-1] == [10, 0], "route runs from start station to goal station")
    failures += check(routes["R1"]["back"] is None, "no return leg for a non-looping robot")
    steps = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(outbound, outbound[1:])]
    failures += check(all(abs(dx) + abs(dy) == 1 for dx, dy in steps), "route moves one cell at a time, 4-connected")

    via = [[5, 3]]
    _, routes = Layout(loop_layout(via=via)).analyse()
    through = routes["R1"]["outbound"]
    failures += check([5, 3] in through, "route passes through the via-point on the spine")

    via2 = [[2, 1], [8, 5]]  # bottom road first (west-bound), then top road
    issues2, routes = Layout(loop_layout(via=via2)).analyse()
    ordered = routes["R1"]["outbound"]
    failures += check(
        [2, 1] in ordered and [8, 5] in ordered and ordered.index([2, 1]) < ordered.index([8, 5]),
        "via-points are visited in the order given",
    )

    _, routes = Layout(loop_layout(loop=True)).analyse()
    back = routes["R1"]["back"]
    failures += check(back is not None and back[0] == [10, 0] and back[-1] == [0, 6], "looping robot gets a return leg")

    blocked = loop_layout()
    blocked["rows"][2] = "S....+....v"  # cut the up-bound left side of the ring
    issues, routes = Layout(blocked).analyse()
    failures += check(routes["R1"]["outbound"] is not None, "spine still connects after cutting one side")
    unreachable = loop_layout(via=[[0, 3]])
    unreachable["rows"][3] = "S....+....v"
    issues, routes = Layout(unreachable).analyse()
    failures += check(any("via point" in i["message"] for i in issues), "via-point on a shelf is reported")


    # ------------------------------------------------------------ API
    tmp_path = LAYOUT_DIR / f"{TMP_NAME}.json"
    try:
        with TestClient(app) as client:
            body = client.post("/api/validate", json=loop_layout(via=via, loop=True, max_speed=0.6)).json()
            failures += check(set(body) >= {"issues", "routes"} and body["routes"]["R1"]["outbound"][0] == [0, 6],
                              "/api/validate returns routes for the preview")
            broken = client.post("/api/validate", json={"width": 0, "height": 1}).json()
            failures += check(broken["routes"] == {} and broken["issues"][0]["cells"] == [],
                              "malformed layout returns empty routes and highlight cells")

            saved = loop_layout(via=via, loop=True, max_speed=0.6)
            saved["stations"][0]["label"] = "Inbound dock"
            client.put(f"/api/layouts/{TMP_NAME}", json=saved)
            back_again = client.get(f"/api/layouts/{TMP_NAME}").json()["layout"]
            robot = back_again["robots"][0]
            failures += check(robot["via"] == via and robot["loop"] is True and robot["max_speed"] == 0.6
                              and back_again["stations"][0]["label"] == "Inbound dock",
                              "via, loop, max speed and station label survive save/reload")
    finally:
        if tmp_path.exists():
            tmp_path.unlink()

    # ------------------------------------------------------------ simulation
    sim = GridSimulation(Layout(loop_layout(via=via2, max_speed=0.5)))
    visited = []
    top_speed = 0.0
    for _ in range(1200):
        sim.step()
        agent = sim.agents[0]
        cell = list(sim._cell(agent.state.position))
        if cell in via2 and cell not in visited:
            visited.append(cell)
        top_speed = max(top_speed, agent.robot.speed)
        if sim.status != "RUNNING":
            break
    failures += check(sim.status == "COMPLETED", f"robot completes its trip (t={sim.time:.1f}s)")
    failures += check(visited == via2, f"simulation drives through the via-points in order {visited}")
    failures += check(top_speed <= 0.5 + 1e-9, f"robot respects its max speed (top speed {top_speed:.2f} m/s)")

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
