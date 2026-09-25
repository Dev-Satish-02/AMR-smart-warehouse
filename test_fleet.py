"""
Fleet management: missions, dispatcher, batteries and charging
(nexus/fleet.py, driven through GridSimulation).

Run:  python test_fleet.py
"""

import json

from fastapi.testclient import TestClient

from nexus.grid_simulation import GridSimulation
from nexus.layout import Layout
from nexus_gui.server import app


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def ring(robots=None, flows=None, fleet=None, stations_extra=()):
    """
    22 x 8 two-lane ring. Stations: A (top-left), B (top-right), C (bottom),
    one charger (west), parking P1 (bottom-left) and P2 (bottom-right).
    """
    W = 22
    rows = [
        "." * W,
        ".++" + "<" * 16 + "++.",
        ".++" + ">" * 16 + "++.",
        ".v^" + "." * 16 + "v^.",
        ".v^" + "." * 16 + "v^.",
        ".++" + "<" * 16 + "++.",
        ".++" + ">" * 16 + "++.",
        "." * W,
    ]
    grid = [list(r) for r in reversed(rows)]  # grid[y][x]
    for x, y in [(5, 6), (15, 6), (10, 1), (4, 1), (17, 1), (1, 4), (8, 1)]:
        grid[y][x] = "+"  # junctions beside stations
    rows = ["".join(r) for r in reversed(grid)]
    stations = [
        {"id": "A", "type": "loading", "x": 5, "y": 7, "label": "Dock A"},
        {"id": "B", "type": "unloading", "x": 15, "y": 7, "label": "Ship B"},
        {"id": "C", "type": "workstation", "x": 10, "y": 0, "label": "Cell C"},
        {"id": "CH", "type": "charging", "x": 0, "y": 4, "label": "Charger"},
        {"id": "P1", "type": "parking", "x": 4, "y": 0, "label": "Parking 1"},
        {"id": "P2", "type": "parking", "x": 17, "y": 0, "label": "Parking 2"},
        {"id": "P3", "type": "parking", "x": 8, "y": 0, "label": "Parking 3"},
        *stations_extra,
    ]
    return {
        "name": "Ring", "width": W, "height": 8, "rows": rows, "stations": stations,
        "robots": robots if robots is not None else [
            {"id": "R1", "mode": "dispatch", "home": "P1"},
            {"id": "R2", "mode": "dispatch", "home": "P2"},
        ],
        "flows": flows or [],
        "fleet": {"generator": False, "charge_rate": 5.0, **(fleet or {})},
    }


def run_until(sim, predicate, seconds=300):
    for _ in range(int(seconds / sim.time_step)):
        sim.step()
        if predicate():
            return True
    return False


def main():
    print("=== NEXUS fleet: missions, dispatcher, batteries ===")
    failures = 0

    # ---------------------------------------------------------- validation
    failures += check(Layout(ring()).validate() == [], "ring test layout is clean")
    legacy = Layout({"name": "x", "width": 3, "height": 1, "rows": ["+++"],
                     "robots": [{"id": "R1", "start": [0, 0], "goal": [2, 0]}]})
    failures += check(legacy.robots[0]["mode"] == "fixed", "robots without a mode stay fixed-route (old layouts unchanged)")
    bad = ring(flows=[{"id": "F1", "name": "Bad", "from": ["A"], "to": ["NOPE"], "rate": 10}])
    failures += check(any("unknown station" in i["message"] for i in Layout(bad).validate()), "flow with an unknown station is an error")
    no_charger = ring()
    no_charger["stations"] = [s for s in no_charger["stations"] if s["type"] != "charging"]
    failures += check(any("no charging station" in i["message"] for i in Layout(no_charger).validate()),
                      "dispatched robots without a charger are flagged")
    fixed_only = ring(robots=[{"id": "R1", "start": "P1", "goal": "A"}],
                      flows=[{"id": "F1", "name": "A-B", "from": ["A"], "to": ["B"], "rate": 10}])
    failures += check(any("no robot is in Dispatched mode" in i["message"] for i in Layout(fixed_only).validate()),
                      "flows without dispatched robots are flagged")
    shared = ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1"}, {"id": "R2", "mode": "dispatch", "home": "P1"}])
    failures += check(any("shares its home" in i["message"] for i in Layout(shared).validate()), "two robots sharing a home is an error")

    # ---------------------------------------------------------- mission lifecycle
    sim = GridSimulation(Layout(ring()))
    fleet = sim.fleet
    mission = fleet.create("A", "B", "normal")
    seen = [mission.status]
    def track():
        if not seen or seen[-1] != mission.status:
            seen.append(mission.status)
        return mission.status == "completed"
    done = run_until(sim, track, 240)
    failures += check(done, f"manual order completes (t={sim.time:.0f}s)")
    failures += check(seen == ["queued", "assigned", "loading", "to_dropoff", "unloading", "completed"],
                      f"lifecycle: {' → '.join(seen)}")
    robot = sim.meta[mission.robot]
    parked = run_until(sim, lambda: robot["activity"] == "IDLE", 120)
    failures += check(parked and sim._cell(sim._agent(mission.robot).state.position) == robot["home_cell"],
                      "robot returns to its home parking after delivering")
    failures += check(fleet.metrics()["avg_lead_time"] is not None and fleet.metrics()["completed"] == 1,
                      f"lead time recorded ({fleet.metrics()['avg_lead_time']} s)")

    # ---------------------------------------------------------- auction: nearest robot wins
    sim = GridSimulation(Layout(ring()))
    m_right = sim.fleet.create("B", "C")            # B is top-right: R2 (P2, bottom-right) is closer
    sim.step()
    failures += check(m_right.robot == "R2", f"nearest robot wins the auction (B → {m_right.robot})")
    m_left = sim.fleet.create("A", "C")
    run_until(sim, lambda: m_left.robot is not None, 5)
    failures += check(m_left.robot == "R1", f"next order goes to the remaining robot ({m_left.robot})")

    # ---------------------------------------------------------- priority
    one = ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1"}])
    sim = GridSimulation(Layout(one))
    low = sim.fleet.create("A", "B", "low")
    high = sim.fleet.create("B", "C", "high")
    sim.step()
    failures += check(high.robot == "R1" and low.robot is None, "high-priority order is served before an older low one")

    # ---------------------------------------------------------- cancellation
    sim = GridSimulation(Layout(one))
    queued = sim.fleet.create("A", "B")
    sim.step()                                        # assigned to R1
    later = sim.fleet.create("B", "C")                # stays queued
    sim.fleet.cancel(later.id)
    failures += check(later.status == "cancelled" and later.id not in sim.fleet.queue, "queued order can be cancelled")
    sim.fleet.cancel(queued.id)
    failures += check(queued.status == "cancelled" and sim.meta["R1"]["activity"] == "RETURNING",
                      "cancelling an assigned (not loaded) order sends the robot home")
    loaded = sim.fleet.create("A", "B")
    run_until(sim, lambda: loaded.status == "to_dropoff", 120)
    try:
        sim.fleet.cancel(loaded.id)
        refused = False
    except ValueError:
        refused = True
    failures += check(refused, "an order that is already loaded cannot be cancelled")

    # ---------------------------------------------------------- battery + charging
    low_bat = ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1", "battery": 31}])
    sim = GridSimulation(Layout(low_bat))
    job = sim.fleet.create("A", "B")
    meta = sim.meta["R1"]
    run_until(sim, lambda: job.status == "completed", 240)
    went = run_until(sim, lambda: meta["activity"] == "CHARGING", 120)
    failures += check(job.status == "completed" and went, f"robot below the threshold finishes its order, then charges ({meta['battery']:.0f}%)")
    back = run_until(sim, lambda: meta["activity"] in ("RETURNING", "IDLE"), 120)
    failures += check(back and meta["battery"] >= 95, f"charges to the target and returns to service ({meta['battery']:.0f}%)")

    drained = ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1", "battery": 12}])
    sim = GridSimulation(Layout(drained))
    job = sim.fleet.create("A", "B")
    run_until(sim, lambda: sim.meta["R1"]["activity"] == "CHARGING", 60)
    failures += check(job.robot is None and sim.meta["R1"]["activity"] == "CHARGING",
                      "a robot without enough battery for the job doesn't bid; it charges")
    run_until(sim, lambda: job.status == "completed", 240)
    failures += check(job.status == "completed", "the order is served once the robot has charged")

    sim = GridSimulation(Layout(one))
    job = sim.fleet.create("A", "B")
    sim.step()
    sim.meta["R1"]["battery"] = 7.5                 # critical on the way to pickup
    sim.step()
    failures += check(job.status == "queued" and job.robot is None and sim.meta["R1"]["activity"] == "TO_CHARGER",
                      "critical battery before pickup: order goes back to the queue, robot charges")

    two_low = ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1", "battery": 20},
                           {"id": "R2", "mode": "dispatch", "home": "P2", "battery": 20}])
    sim = GridSimulation(Layout(two_low))
    peak = {"R1": 0.0, "R2": 0.0}
    def charged():
        for r in peak:
            peak[r] = max(peak[r], sim.meta[r]["battery"])
        return all(peak[r] >= 95 and sim.meta[r]["activity"] == "IDLE" for r in peak)
    both = run_until(sim, charged, 400)
    failures += check(both and sim.fleet.counters["charges"] == 2, "two low robots share one charger in turn, then park")

    # ---------------------------------------------------------- generator
    flows = [{"id": "F1", "name": "A to B", "from": ["A"], "to": ["B"], "rate": 300},
             {"id": "F2", "name": "B to C", "from": ["B"], "to": ["C"], "rate": 200, "priority": "high"}]
    def generated(seed):
        s = GridSimulation(Layout(ring(flows=flows, fleet={"generator": True, "seed": seed})))
        s.run(max_steps=3000)
        return [(m.id, m.pickup, m.dropoff, m.priority, round(m.created, 1)) for m in s.fleet.missions.values()], s
    a, sim_a = generated(3)
    b, _ = generated(3)
    failures += check(a == b and len(a) >= 5, f"order generator is repeatable for a seed ({len(a)} orders in 300 s)")
    failures += check(sim_a.fleet.counters["completed"] >= 5 and sim_a.metrics()["safety_violations"] == 0,
                      f"generated orders are delivered safely ({sim_a.fleet.counters['completed']} done)")

    # ---------------------------------------------------------- API
    with TestClient(app) as client:
        client.put("/api/layouts/_test_fleet_tmp", json=ring())
        try:
            with client.websocket_connect("/ws") as ws:
                ws.receive_text(); ws.receive_text()
                ws.send_text(json.dumps({"cmd": "load", "name": "_test_fleet_tmp"}))
                ws.receive_text(); ws.receive_text()
                created = client.post("/api/missions", json={"pickup": "A", "dropoff": "B", "priority": "high"})
                failures += check(created.status_code == 200 and created.json()["status"] == "queued", "POST /api/missions creates an order")
                failures += check(client.post("/api/missions", json={"pickup": "A", "dropoff": "A"}).status_code == 422,
                                  "invalid orders are rejected")
                ws.send_text(json.dumps({"cmd": "mission.create", "pickup": "B", "dropoff": "C", "priority": "low"}))
                state = None
                for _ in range(10):
                    msg = json.loads(ws.receive_text())
                    if msg.get("type") == "state" and msg["state"]["fleet"] and len(msg["state"]["fleet"]["queued"]) == 2:
                        state = msg["state"]
                        break
                failures += check(state is not None, "WebSocket mission.create adds an order to the live queue")
                mid = created.json()["id"]
                failures += check(client.delete(f"/api/missions/{mid}").json()["status"] == "cancelled", "DELETE /api/missions cancels an order")
                listing = client.get("/api/missions").json()
                failures += check(len(listing["queued"]) == 1 and listing["metrics"]["cancelled"] == 1, "GET /api/missions lists the queue")
                ws.send_text(json.dumps({"cmd": "fleet.generator", "value": True}))
                switched = False
                for _ in range(10):  # skip broadcasts queued by the REST calls above
                    msg = json.loads(ws.receive_text())
                    if msg.get("type") == "state" and msg["state"]["fleet"]["generator"] is True:
                        switched = True
                        break
                failures += check(switched, "generator can be switched on live")
        finally:
            import os
            path = "layouts/_test_fleet_tmp.json"
            if os.path.exists(path):
                os.remove(path)

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
