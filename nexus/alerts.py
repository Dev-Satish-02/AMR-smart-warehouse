"""
Operator alerts for the running fleet.

Condition alerts are raised while a condition holds and resolve on their own
when it ends:

    ESTOP           critical  fleet-wide E-stop engaged
    ROBOT_ERROR     critical  a robot cannot operate (e.g. no lane route)
    BATTERY_CRIT    critical  battery below the critical level
    BLOCKED         warning   robot waiting in traffic for more than 30 s
    BATTERY_LOW     warning   battery below the charge threshold and not charging
    CHARGER_WAIT    warning   robot waiting more than 60 s for a free charger
    BACKLOG         warning   more than 15 orders queued, or the oldest waits > 3 min
    HOLD            info      robot held by an operator

Event alerts come from the simulation log and resolve after 60 s:

    SAFETY          critical  two robots closer than the safety distance
    NO_ROUTE        critical  a robot could not be routed to its destination
    DEADLOCK        info      a deadlock was detected and resolved by back-off

Every alert keeps first/last seen times, occurrence count and whether an
operator acknowledged it; resolved alerts move to the history.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

BLOCKED_AFTER = 30.0
CHARGER_WAIT_AFTER = 60.0
BACKLOG_QUEUE = 15
BACKLOG_AGE = 180.0
EVENT_ALERT_SECONDS = 60.0
EVALUATE_EVERY = 0.5
HISTORY_LIMIT = 200

SEVERITY_RANK = {"critical": 0, "warning": 1, "info": 2}


class AlertManager:

    def __init__(self, sim):
        self.sim = sim
        self.active: Dict[str, Dict[str, Any]] = {}
        self.history: List[Dict[str, Any]] = []
        self.next_number = 1
        self.next_eval = 0.0
        self.seen_events = 0
        self.totals: Dict[str, int] = {}

    # ------------------------------------------------------------------

    def _raise(self, key: str, code: str, severity: str, message: str, robot: Optional[str], now: float,
               expires: Optional[float] = None):
        alert = self.active.get(key)
        if alert is None:
            alert = {
                "id": f"A-{self.next_number:04d}", "key": key, "code": code, "severity": severity,
                "robot": robot, "message": message, "first_seen": now, "last_seen": now,
                "resolved_at": None, "active": True, "acknowledged": False, "count": 1, "expires": expires,
            }
            self.next_number += 1
            self.active[key] = alert
            self.totals[code] = self.totals.get(code, 0) + 1
        else:
            alert["last_seen"] = now
            alert["message"] = message
            alert["severity"] = severity
            if expires is not None:
                alert["expires"] = expires
                alert["count"] += 1
        return alert

    def _resolve(self, key: str, now: float):
        alert = self.active.pop(key)
        alert["active"] = False
        alert["resolved_at"] = now
        self.history.append(alert)
        if len(self.history) > HISTORY_LIMIT:
            self.history = self.history[-HISTORY_LIMIT:]

    # ------------------------------------------------------------------

    def evaluate(self, now: float, force: bool = False):
        """Re-check conditions (every 0.5 s of sim time, or now if forced,
        e.g. right after an operator action while the simulation is paused)."""
        self._scan_events(now)
        if not force and now + 1e-9 < self.next_eval:
            return
        self.next_eval = now + EVALUATE_EVERY
        sim = self.sim
        cfg = sim.fleet.cfg
        present = set()

        def raise_(key, code, severity, message, robot=None):
            present.add(key)
            self._raise(key, code, severity, message, robot, now)

        if sim.estop:
            raise_("ESTOP", "ESTOP", "critical", f"E-stop engaged {now - sim.estop_since:.0f} s ago: all robots stopped")

        for agent in sim.agents:
            rid = agent.robot_id
            meta = sim.meta[rid]
            if meta["error"]:
                raise_(f"ERROR:{rid}", "ROBOT_ERROR", "critical", f"{rid}: {meta['error']}", rid)
            if meta["hold"]:
                raise_(f"HOLD:{rid}", "HOLD", "info", f"{rid} is held by an operator", rid)
            if meta["stall_since"] is not None and now - meta["stall_since"] > BLOCKED_AFTER and not sim.estop:
                who = meta["blocked_by"] or (sim.yielding.get(rid) or {}).get("winner")
                raise_(f"BLOCKED:{rid}", "BLOCKED", "warning",
                       f"{rid} waiting {now - meta['stall_since']:.0f} s" + (f" (for {who})" if who else ""), rid)
            battery = meta["battery"]
            if battery is not None:
                charging = meta["activity"] in ("CHARGING", "TO_CHARGER")
                if battery < cfg["battery_critical"]:
                    raise_(f"BATTERY:{rid}", "BATTERY_CRIT", "critical", f"{rid} battery critical: {battery:.0f}%", rid)
                elif battery < cfg["battery_low"] and not charging:
                    plan = " (will charge after its current order)" if meta["mission"] else ""
                    raise_(f"BATTERY:{rid}", "BATTERY_LOW", "warning", f"{rid} battery low: {battery:.0f}%{plan}", rid)
                if meta.get("needs_charge"):
                    since = meta.setdefault("charge_wait_since", now)
                    if now - since > CHARGER_WAIT_AFTER:
                        raise_(f"CHARGER:{rid}", "CHARGER_WAIT", "warning",
                               f"{rid} waiting {now - since:.0f} s for a free charger", rid)
                else:
                    meta.pop("charge_wait_since", None)

        fleet = sim.fleet
        if fleet.active and fleet.queue:
            oldest = min(fleet.missions[m].created for m in fleet.queue)
            if len(fleet.queue) > BACKLOG_QUEUE or now - oldest > BACKLOG_AGE:
                raise_("BACKLOG", "BACKLOG", "warning",
                       f"{len(fleet.queue)} orders queued, oldest waiting {now - oldest:.0f} s")

        for key in list(self.active):
            alert = self.active[key]
            if alert["expires"] is not None:
                if now >= alert["expires"]:
                    self._resolve(key, now)
            elif key not in present:
                self._resolve(key, now)

    def _scan_events(self, now: float):
        # Events carry an increasing sequence number (the log itself is trimmed).
        for event in self.sim.events:
            if event["seq"] <= self.seen_events:
                continue
            self.seen_events = event["seq"]
            text = event["text"]
            robot = event.get("robot")
            if event["kind"] == "error" and text.startswith("SAFETY"):
                self._raise(f"SAFETY:{robot}", "SAFETY", "critical", text, robot, now, now + EVENT_ALERT_SECONDS)
            elif event["kind"] == "error" and "no lane route" in text:
                self._raise(f"NO_ROUTE:{robot}", "NO_ROUTE", "critical", text, robot, now, now + EVENT_ALERT_SECONDS)
            elif event["kind"] == "deadlock":
                self._raise(f"DEADLOCK:{robot}", "DEADLOCK", "info", text, robot, now, now + EVENT_ALERT_SECONDS)

    # ------------------------------------------------------------------

    def acknowledge(self, alert_id: Optional[str] = None) -> int:
        """Acknowledge one alert (by id) or all active alerts. Returns the count."""
        count = 0
        for alert in list(self.active.values()) + self.history:
            if (alert_id is None and alert["active"]) or alert["id"] == alert_id:
                if not alert["acknowledged"]:
                    alert["acknowledged"] = True
                    count += 1
        if alert_id is not None and count == 0 and not any(a["id"] == alert_id for a in list(self.active.values()) + self.history):
            raise ValueError(f"No alert {alert_id}")
        return count

    def snapshot(self) -> Dict[str, Any]:
        active = sorted(self.active.values(), key=lambda a: (SEVERITY_RANK[a["severity"]], a["first_seen"]))
        clean = lambda a: {k: v for k, v in a.items() if k != "expires"}
        return {
            "active": [clean(a) for a in active],
            "recent": [clean(a) for a in reversed(self.history[-60:])],
            "counts": {
                "critical": sum(1 for a in active if a["severity"] == "critical"),
                "warning": sum(1 for a in active if a["severity"] == "warning"),
                "info": sum(1 for a in active if a["severity"] == "info"),
                "unacknowledged": sum(1 for a in active if not a["acknowledged"] and a["severity"] != "info"),
            },
            "totals": dict(self.totals),
        }


__all__ = ["AlertManager"]
