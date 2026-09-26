"""
Operations features: E-stop, operator hold, charge now, alerts, shift report,
CSV exports and their REST / WebSocket commands.

Run:  python test_operations.py
"""

import csv
import io
import json
import os

from fastapi.testclient import TestClient

from nexus.grid_simulation import GridSimulation, TIME_CATEGORIES
from nexus.layout import Layout, load_layout
from nexus.report import build_report, missions_csv, robots_csv, _percentile
from nexus_gui.server import app, controller
from test_fleet import ring, run_until


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def lane_layout():
    """One straight east-bound lane, 20 cells, two fixed-route robots in line."""
    rows = ["." * 22, "." + ">" * 20 + ".", "." * 22]
    return {
        "name": "Lane", "width": 22, "height": 3, "rows": rows,
        "robots": [
            {"id": "R1", "start": [5, 1], "goal": [19, 1]},
            {"id": "R2", "start": [1, 1], "goal": [18, 1]},
        ],
    }


def active_codes(sim):
    return {a["code"]: a for a in sim.alerts.snapshot()["active"]}


def main():
    print("=== NEXUS operations: E-stop, hold, alerts, reports ===")
    failures = 0

    # ---------------------------------------------------------- E-stop
    sim = GridSimulation(load_layout("layouts/bel_warehouse.json"))
    run_until(sim, lambda: any(sim.meta[r]["activity"] == "LOADING" for r in sim.meta), 300)
    loading = next(r for r in sim.meta if sim.meta[r]["activity"] == "LOADING")
    before = {a.robot_id: a.state.position.copy() for a in sim.agents}
    queue_before = len(sim.fleet.queue)
    assigned_before = sum(1 for m in sim.fleet.missions.values() if m.assigned_at is not None)
    sim.set_estop(True)
    for _ in range(600):
        sim.step()
    frozen = all((a.state.position == before[a.robot_id]).all() for a in sim.agents)
    failures += check(frozen and all(sim.meta[a.robot_id]["status"] == "E_STOP" for a in sim.agents),
                      "E-stop: every robot stops where it is (60 s)")
    failures += check(sum(1 for m in sim.fleet.missions.values() if m.assigned_at is not None) == assigned_before
                      and len(sim.fleet.queue) >= queue_before,
                      f"E-stop: nothing is dispatched, orders keep queueing ({queue_before} → {len(sim.fleet.queue)})")
    failures += check(sim.meta[loading]["activity"] == "LOADING", "E-stop: loading timers freeze")
    estop_alert = active_codes(sim).get("ESTOP")
    failures += check(estop_alert is not None and estop_alert["severity"] == "critical", "E-stop raises a critical alert")
    sim.set_estop(False)
    moved = run_until(sim, lambda: any((a.state.position != before[a.robot_id]).any() for a in sim.agents), 10)
    failures += check(moved and "ESTOP" not in active_codes(sim), "release: robots move again and the alert resolves")
    failures += check(all(abs(sum(sim.meta[a.robot_id]["time_in"].values()) - sim.time) < 0.15 for a in sim.agents)
                      and all(sim.meta[a.robot_id]["time_in"]["stopped"] >= 59.9 for a in sim.agents),
                      "time accounting: categories add up to the shift, 60 s booked as E-stop")

    # ---------------------------------------------------------- hold + blocked alert
    sim = GridSimulation(Layout(lane_layout()))
    sim.step()
    sim.set_hold("R1", True)
    held_at = sim._agent("R1").state.position.copy()
    run_until(sim, lambda: "BLOCKED" in active_codes(sim), 60)
    codes = active_codes(sim)
    failures += check((sim._agent("R1").state.position == held_at).all() and sim.meta["R1"]["status"] == "HELD",
                      "hold: the robot stays put with status HELD")
    failures += check("HOLD" in codes and codes["HOLD"]["severity"] == "info", "hold raises an info alert")
    blocked = codes.get("BLOCKED")
    waited = blocked["first_seen"] - sim.meta["R2"]["stall_since"] if blocked else 0
    failures += check(blocked is not None and blocked["robot"] == "R2" and "for R1" in blocked["message"]
                      and 30.0 <= waited <= 31.0,
                      f"robot waiting > 30 s behind it raises BLOCKED ({blocked and blocked['message']})")
    sim.set_hold("R1", False)
    cleared = run_until(sim, lambda: "BLOCKED" not in active_codes(sim) and "HOLD" not in active_codes(sim), 10)
    failures += check(cleared, "releasing the hold clears both alerts")

    # ---------------------------------------------------------- hold excludes dispatch
    sim = GridSimulation(Layout(ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1"}])))
    sim.set_hold("R1", True)
    job = sim.fleet.create("A", "B")
    run_until(sim, lambda: False, 5)
    failures += check(job.robot is None, "a held robot is not dispatched")
    sim.set_hold("R1", False)
    run_until(sim, lambda: job.robot == "R1", 5)
    failures += check(job.robot == "R1", "after release it takes the order")

    # ---------------------------------------------------------- charge now
    sim = GridSimulation(Layout(ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1"}])))
    result = sim.fleet.charge_now("R1")
    failures += check(result == "now" and sim.meta["R1"]["activity"] == "TO_CHARGER", "charge now: an idle robot goes to charge")
    sim = GridSimulation(Layout(ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1"}])))
    job = sim.fleet.create("A", "B")
    sim.step()
    sim.fleet.charge_now("R1")
    failures += check(job.status == "queued" and sim.meta["R1"]["activity"] == "TO_CHARGER",
                      "charge now before pickup: the order goes back to the queue")
    sim = GridSimulation(Layout(ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1"}])))
    job = sim.fleet.create("A", "B")
    run_until(sim, lambda: job.status == "to_dropoff", 120)
    sim.fleet.charge_now("R1")
    delivered_then_charged = run_until(sim, lambda: job.status == "completed" and sim.meta["R1"]["activity"] == "TO_CHARGER", 120)
    failures += check(delivered_then_charged, "charge now with a load: delivers first, then charges")

    # ---------------------------------------------------------- battery, backlog, charger wait, events
    sim = GridSimulation(Layout(ring()))
    job = sim.fleet.create("A", "B")
    run_until(sim, lambda: job.status == "to_dropoff", 120)
    sim.meta[job.robot]["battery"] = 5.0
    sim.alerts.next_eval = 0
    sim.alerts.evaluate(sim.time)
    failures += check(active_codes(sim).get("BATTERY_CRIT", {}).get("severity") == "critical", "battery below critical raises a critical alert")

    sim = GridSimulation(Layout(ring()))
    for r in ("R1", "R2"):
        sim.set_hold(r, True)
    for _ in range(16):
        sim.fleet.create("A", "B")
    run_until(sim, lambda: "BACKLOG" in active_codes(sim), 5)
    failures += check("BACKLOG" in active_codes(sim), "more than 15 queued orders raises BACKLOG")

    slow = ring(robots=[{"id": "R1", "mode": "dispatch", "home": "P1", "battery": 20},
                        {"id": "R2", "mode": "dispatch", "home": "P2", "battery": 20}],
                fleet={"charge_rate": 0.8})
    sim = GridSimulation(Layout(slow))
    waited = run_until(sim, lambda: "CHARGER_WAIT" in active_codes(sim), 200)
    failures += check(waited, "a robot waiting > 60 s for the only charger raises CHARGER_WAIT")

    sim = GridSimulation(Layout(ring()))
    sim._event("error", "R1: no lane route to Ship B", "R1")
    sim.step()
    failures += check(active_codes(sim).get("NO_ROUTE", {}).get("severity") == "critical", "a routing failure raises NO_ROUTE")
    run_until(sim, lambda: "NO_ROUTE" not in active_codes(sim), 70)
    failures += check("NO_ROUTE" not in active_codes(sim) and any(a["code"] == "NO_ROUTE" for a in sim.alerts.snapshot()["recent"]),
                      "event alerts resolve after 60 s and stay in the history")

    # ---------------------------------------------------------- acknowledge
    sim = GridSimulation(Layout(ring()))
    sim.set_estop(True)
    for r in ("R1", "R2"):
        sim.set_hold(r, True)
    for _ in range(16):
        sim.fleet.create("A", "B")
    run_until(sim, lambda: False, 2)
    snap = sim.alerts.snapshot()
    first = snap["active"][0]
    failures += check(first["code"] == "ESTOP" and snap["counts"]["unacknowledged"] == 2,
                      "active alerts sort critical first; info alerts don't need acknowledgement")
    sim.alerts.acknowledge(first["id"])
    failures += check(sim.alerts.snapshot()["counts"]["unacknowledged"] == 1, "acknowledging one alert")
    sim.alerts.acknowledge()
    failures += check(sim.alerts.snapshot()["counts"]["unacknowledged"] == 0, "acknowledge all")

    # ---------------------------------------------------------- report + CSV
    sim = GridSimulation(load_layout("layouts/bel_warehouse.json"))
    sim.run(max_steps=4000)
    report = build_report(sim)
    s = report["summary"]
    completed = sum(1 for m in sim.fleet.missions.values() if m.status == "completed")
    failures += check(s["delivered"] == completed == sum(report["timeline"]["counts"]),
                      f"report: delivered count matches missions and timeline ({completed})")
    failures += check(all(abs(sum(r["time_in"].values()) - report["shift_seconds"]) < 0.2 for r in report["robots"]),
                      "report: each robot's time breakdown adds up to the shift")
    failures += check(abs(sum(report["time_breakdown"].values()) - 1) < 1e-3 and 0 <= s["utilisation"] <= 1,
                      f"report: fleet time shares add to 100% (utilisation {s['utilisation']:.0%}, traffic {s['traffic_share']:.1%})")
    failures += check(s["lead_time_avg"] is not None and s["lead_time_p90"] >= s["lead_time_avg"] * 0.8,
                      f"report: lead time avg {s['lead_time_avg']} s, p90 {s['lead_time_p90']} s")
    failures += check(_percentile([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 0.9) == 9.1, "90th percentile uses linear interpolation")
    rows = list(csv.reader(io.StringIO(missions_csv(sim))))
    failures += check(rows[0][0] == "id" and len(rows) - 1 == len(sim.fleet.missions), f"missions CSV has every order ({len(rows) - 1})")
    rrows = list(csv.reader(io.StringIO(robots_csv(sim))))
    failures += check(len(rrows) - 1 == len(sim.agents) and rrows[0][-len(TIME_CATEGORIES):] == [f"{c}_s" for c in TIME_CATEGORIES],
                      "robots CSV has every robot with its time breakdown")

    # ---------------------------------------------------------- API + WebSocket
    with TestClient(app) as client:
        client.put("/api/layouts/_test_ops_tmp", json=ring())
        try:
            with client.websocket_connect("/ws") as ws:
                ws.receive_text(); ws.receive_text()
                ws.send_text(json.dumps({"cmd": "load", "name": "_test_ops_tmp"}))
                ws.receive_text(); ws.receive_text()

                def next_state(pred):
                    for _ in range(12):
                        msg = json.loads(ws.receive_text())
                        if msg.get("type") == "state" and pred(msg["state"]):
                            return msg["state"]
                    return None

                ws.send_text(json.dumps({"cmd": "estop", "value": True}))
                st = next_state(lambda st: st["estop"])
                failures += check(st is not None and any(a["code"] == "ESTOP" for a in st["alerts"]["active"]),
                                  "WebSocket estop engages and the state carries the alert")
                failures += check(client.post("/api/estop", json={"engaged": False}).json() == {"estop": False},
                                  "POST /api/estop releases")
                ws.send_text(json.dumps({"cmd": "robot.hold", "id": "R1", "value": True}))
                st = next_state(lambda st: any(r["id"] == "R1" and r["hold"] for r in st["robots"]))
                failures += check(st is not None and next(r for r in st["robots"] if r["id"] == "R1")["status"] == "HELD",
                                  "WebSocket robot.hold holds a robot")
                ws.send_text(json.dumps({"cmd": "robot.charge", "id": "R2"}))
                st = next_state(lambda st: next(r for r in st["robots"] if r["id"] == "R2")["activity"] == "TO_CHARGER")
                failures += check(st is not None, "WebSocket robot.charge sends a robot to charge")
                alerts = client.get("/api/alerts").json()
                failures += check(any(a["code"] == "HOLD" for a in alerts["active"]), "GET /api/alerts lists active alerts")
                ack = client.post("/api/alerts/ack", json={}).json()
                failures += check("acknowledged" in ack and client.post("/api/alerts/ack", json={"id": "A-9999"}).status_code == 404,
                                  "POST /api/alerts/ack acknowledges (unknown id → 404)")
                rep = client.get("/api/report")
                failures += check(rep.status_code == 200 and rep.json()["layout"] == "Ring", "GET /api/report")
                mc = client.get("/api/report/missions.csv")
                failures += check(mc.headers["content-type"].startswith("text/csv") and "attachment" in mc.headers["content-disposition"]
                                  and mc.text.startswith("id,"), "missions CSV downloads as an attachment")
                rc = client.get("/api/report/robots.csv")
                failures += check(rc.text.startswith("robot,") and rc.text.count("\n") == 3, "robots CSV downloads")
                allm = client.get("/api/missions/all").json()
                failures += check(set(allm) >= {"missions", "flows", "generator", "time"}, "GET /api/missions/all")

                # Event log keeps flowing after the simulation trims it (> 300 events).
                sim = controller.sim
                for i in range(400):
                    sim._event("system", f"filler {i}")
                controller.sent_events = sim.event_seq - 5
                fresh = controller.state_message()["events"]
                failures += check(len(fresh) == 5 and fresh[-1]["text"] == "filler 399",
                                  "live log still delivers new events after 300+ events")
        finally:
            path = "layouts/_test_ops_tmp.json"
            if os.path.exists(path):
                os.remove(path)

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
