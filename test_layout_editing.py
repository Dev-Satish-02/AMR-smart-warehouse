"""
Layout editor backend: saving, reloading and validating layouts.

Covers what the editor relies on:
    * PUT /api/layouts/<name> round-trips a layout exactly
    * file names are sanitised, malformed layouts are rejected
    * validation finds unreachable goals, missing return routes, dead-end
      one-way lanes, bad via-points, and returns cells to highlight
    * arbitrary warehouse sizes and multi-lane roads validate cleanly

Run:  python test_layout_editing.py
"""

from pathlib import Path

from fastapi.testclient import TestClient

from nexus.layout import Layout, blank_layout
from nexus_gui.server import LAYOUT_DIR, app

TMP_NAME = "_test_editor_tmp"


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def issues_of(layout_dict):
    return Layout(layout_dict).validate()


def two_lane_corridor():
    """12 x 5 warehouse, a 2-lane road (west-bound on y=3, east-bound on y=2)
    with junction columns at both ends so robots can U-turn."""
    rows = [
        "############",
        "#++<<<<<<++#",
        "#++>>>>>>++#",
        "#..........#",
        "############",
    ]
    return {
        "name": "Two-lane corridor",
        "width": 12,
        "height": 5,
        "rows": rows,
        "stations": [
            {"id": "A", "type": "loading", "x": 1, "y": 1},
            {"id": "B", "type": "unloading", "x": 10, "y": 1},
        ],
        "robots": [
            {"id": "R1", "start": "A", "goal": "B", "loop": True},
            {"id": "R2", "start": "B", "goal": "A", "loop": True, "via": [[5, 3]]},
        ],
    }


def main():
    print("=== NEXUS layout editing ===")
    failures = 0
    tmp_path = LAYOUT_DIR / f"{TMP_NAME}.json"

    try:
        with TestClient(app) as client:
            layout = two_lane_corridor()
            # stations sit below the road on floor cells: connect them
            layout["rows"][3] = "#+........+#"

            res = client.put(f"/api/layouts/{TMP_NAME}", json=layout)
            failures += check(res.status_code == 200, "save a new layout")
            failures += check(res.json()["issues"] == [], f"two-lane corridor has no issues {res.json()['issues']}")

            back = client.get(f"/api/layouts/{TMP_NAME}").json()["layout"]
            same = all(back[k] == layout[k] for k in ("name", "width", "height", "rows"))
            same = same and back["robots"][1]["via"] == [[5, 3]] and back["robots"][0]["loop"] is True
            failures += check(same, "reload returns the same rows, robots and via-points")
            listed = [item["name"] for item in client.get("/api/layouts").json()["layouts"]]
            failures += check(TMP_NAME in listed, "saved layout appears in the layout list")

            failures += check(client.put("/api/layouts/bad name!", json=layout).status_code == 400,
                              "file names with spaces/symbols are rejected")
            failures += check(client.put(f"/api/layouts/{TMP_NAME}", json={"width": 0, "height": 3}).status_code == 422,
                              "malformed layout is rejected")

        # ---------------------------------------------------------- validation
        big = blank_layout(80, 50, name="Big")
        failures += check(Layout(big).validate() == [], "an empty 80 x 50 warehouse validates")

        one_way = {
            "name": "One way", "width": 8, "height": 3,
            "rows": ["########", "#>>>>>>#", "########"],
            "stations": [{"id": "A", "type": "loading", "x": 1, "y": 1},
                         {"id": "B", "type": "unloading", "x": 6, "y": 1}],
            "robots": [{"id": "R1", "start": "B", "goal": "A"},
                       {"id": "R2", "start": "A", "goal": "B", "loop": True}],
        }
        messages = [i["message"] for i in issues_of(one_way)]
        failures += check(any("R1: no lane route" in m for m in messages), "goal against a one-way lane is unreachable")
        failures += check(any("R2: can reach its goal but has no lane route back" in m for m in messages),
                          "looping robot without a return route is flagged")
        unreachable = next(i for i in issues_of(one_way) if i["message"].startswith("R1"))
        failures += check(unreachable["cells"] == [[6, 1], [1, 1]], "issue carries start/goal cells to highlight")

        dead = {"name": "Dead end", "width": 6, "height": 3,
                "rows": ["######", "#>>>.#", "######"], "stations": [], "robots": []}
        dead_issue = [i for i in issues_of(dead) if "dead end" in i["message"]]
        failures += check(len(dead_issue) == 1 and dead_issue[0]["cells"] == [[3, 1]],
                          "one-way lane running into floor is a dead end")

        bad_via = two_lane_corridor()
        bad_via["robots"][0]["via"] = [[0, 0]]
        failures += check(any(i["severity"] == "error" and "via point" in i["message"] for i in issues_of(bad_via)),
                          "via-point on a wall is an error")

        shared = two_lane_corridor()
        shared["robots"][1]["start"] = "A"
        failures += check(any("shares its start" in i["message"] for i in issues_of(shared)),
                          "two robots on one start cell is an error")
    finally:
        if tmp_path.exists():
            tmp_path.unlink()

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
