"""
Shift report for the running simulation: what engineers ask about a fleet.

    build_report(sim)   -> JSON-ready dict (Reports page, /api/report)
    missions_csv(sim)   -> CSV of every transport order
    robots_csv(sim)     -> CSV of per-robot utilisation and time breakdown

Time categories per robot (see grid_simulation.TIME_CATEGORY):
    moving    driving, turning, rerouting, backing off
    handling  loading / unloading / docked at a station
    waiting   waiting in traffic or yielding to another robot
    charging  on a charger
    idle      parked / arrived with nothing to do
    held      held by an operator
    stopped   E-stop
Utilisation = (moving + handling) / shift time. "Time lost to traffic" is the
waiting share.
"""

from __future__ import annotations

import csv
import io
import math
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from nexus.grid_simulation import TIME_CATEGORIES


def _percentile(values: List[float], p: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    k = (len(ordered) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    value = ordered[lo] + (ordered[hi] - ordered[lo]) * (k - lo)
    return round(value, 1)


def _mean(values: List[float]) -> Optional[float]:
    return round(sum(values) / len(values), 1) if values else None


def bucket_seconds(duration: float) -> int:
    """Timeline bucket size: 1, 2, 5, 10, 15, 30 or 60 minutes, ~12-30 bars."""
    for minutes in (1, 2, 5, 10, 15, 30, 60):
        if duration / (minutes * 60) <= 30:
            return minutes * 60
    return 3600


def build_report(sim) -> Dict[str, Any]:
    now = float(sim.world.time)
    fleet = sim.fleet
    missions = fleet.all_missions() if fleet.active else []
    completed = [m for m in missions if m["status"] == "completed"]
    lead = [m["completed_at"] - m["created"] for m in completed]
    wait = [m["assigned_at"] - m["created"] for m in missions if m["assigned_at"] is not None]

    # Missions (dispatched) or trips (fixed routes) delivered over time.
    size = bucket_seconds(max(now, 1.0))
    buckets = [0] * max(1, math.ceil(now / size))
    for m in completed:
        buckets[min(int(m["completed_at"] // size), len(buckets) - 1)] += 1

    robots = []
    totals = {c: 0.0 for c in TIME_CATEGORIES}
    for agent in sim.agents:
        meta = sim.meta[agent.robot_id]
        time_in = {c: round(meta["time_in"].get(c, 0.0), 1) for c in TIME_CATEGORIES}
        for c in TIME_CATEGORIES:
            totals[c] += time_in[c]
        span = max(sum(time_in.values()), 1e-9)
        robots.append({
            "id": agent.robot_id,
            "mode": meta["mode"],
            "status": meta["status"],
            "completed": meta["trips"],
            "distance_m": round(agent.robot.distance_travelled, 1),
            "battery": None if meta["battery"] is None else round(meta["battery"], 1),
            "charges": meta["charges"],
            "time_in": time_in,
            "utilisation": round((time_in["moving"] + time_in["handling"]) / span, 3),
            "waiting_share": round(time_in["waiting"] / span, 3),
        })

    fleet_time = max(sum(totals.values()), 1e-9)
    counters = sim.counters
    alerts = sim.alerts.snapshot()
    delivered = len(completed) if fleet.active else counters["trips_completed"]
    return {
        "layout": sim.layout.name,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "shift_seconds": round(now, 1),
        "estop": sim.estop,
        "summary": {
            "delivered": delivered,
            "delivered_per_hour": round(delivered / max(now, 1e-9) * 3600.0, 1),
            "mode": "missions" if fleet.active else "trips",
            "created": len(missions),
            "open": sum(1 for m in missions if m["status"] not in ("completed", "cancelled")),
            "cancelled": sum(1 for m in missions if m["status"] == "cancelled"),
            "lead_time_avg": _mean(lead),
            "lead_time_p90": _percentile(lead, 0.9),
            "lead_time_max": round(max(lead), 1) if lead else None,
            "wait_avg": _mean(wait),
            "wait_p90": _percentile(wait, 0.9),
            "utilisation": round((totals["moving"] + totals["handling"]) / fleet_time, 3),
            "traffic_share": round(totals["waiting"] / fleet_time, 3),
            "distance_m": round(sum(r["distance_m"] for r in robots), 1),
            "robots": len(robots),
            "charges": sum(r["charges"] for r in robots),
        },
        "time_breakdown": {c: round(totals[c] / fleet_time, 4) for c in TIME_CATEGORIES},
        "timeline": {"bucket_seconds": size, "counts": buckets},
        "robots": robots,
        "flows": fleet.flow_stats() if fleet.active else [],
        "safety": {
            "violations": counters["safety_violations"],
            "min_separation": None if sim.min_separation == float("inf") else round(sim.min_separation, 2),
        },
        "coordination": {
            "negotiations": counters["negotiations"],
            "reroutes": counters["reroutes"],
            "deadlocks_resolved": counters["deadlocks_resolved"],
            "backoffs": counters["backoffs"],
        },
        "alerts": {"totals": alerts["totals"], "active": len(alerts["active"])},
    }


def _csv(rows: List[List[Any]]) -> str:
    out = io.StringIO()
    csv.writer(out).writerows(rows)
    return out.getvalue()


def missions_csv(sim) -> str:
    rows = [["id", "flow", "source", "priority", "status", "pickup", "dropoff", "robot",
             "created_s", "assigned_s", "picked_s", "completed_s", "wait_s", "lead_time_s"]]
    missions = sim.fleet.all_missions() if sim.fleet.active else []
    for m in missions:
        wait = None if m["assigned_at"] is None else round(m["assigned_at"] - m["created"], 1)
        lead = None if m["status"] != "completed" else round(m["completed_at"] - m["created"], 1)
        rows.append([m["id"], m["flow"] or "", m["source"], m["priority"], m["status"],
                     m["pickup_label"], m["dropoff_label"], m["robot"] or "",
                     round(m["created"], 1), "" if m["assigned_at"] is None else round(m["assigned_at"], 1),
                     "" if m["picked_at"] is None else round(m["picked_at"], 1),
                     "" if m["completed_at"] is None else round(m["completed_at"], 1),
                     "" if wait is None else wait, "" if lead is None else lead])
    return _csv(rows)


def robots_csv(sim) -> str:
    report = build_report(sim)
    rows = [["robot", "mode", "status", "completed", "distance_m", "battery_pct", "charges", "utilisation",
             *[f"{c}_s" for c in TIME_CATEGORIES]]]
    for r in report["robots"]:
        rows.append([r["id"], r["mode"], r["status"], r["completed"], r["distance_m"],
                     "" if r["battery"] is None else r["battery"], r["charges"], r["utilisation"],
                     *[r["time_in"][c] for c in TIME_CATEGORIES]])
    return _csv(rows)


__all__ = ["build_report", "missions_csv", "robots_csv", "bucket_seconds"]
