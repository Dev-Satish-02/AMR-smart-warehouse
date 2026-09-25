"""
Fleet management for dispatched robots: transport orders (missions), a
market-based dispatcher, a battery model with automatic charging, and an
order generator driven by the layout's mission flows.

Mission lifecycle
    queued -> assigned (robot drives to pickup) -> loading -> to_dropoff
    -> unloading -> completed          (or cancelled / requeued)

Dispatcher (runs every second)
    Orders are taken highest priority first, then oldest first. Every free
    robot bids its lane-network travel time to the pickup; the lowest bid
    wins. Robots only bid if their battery covers pickup + delivery + the
    trip to a charger with a safety margin.

Battery
    Drains per metre driven and slowly while parked. Below `battery_low` a
    robot finishes its current order, then goes to the nearest free charger
    and charges to `charge_target` before taking orders again. Below
    `battery_critical` an unloaded robot hands its order back to the queue
    and charges immediately.
"""

from __future__ import annotations

import heapq
import random
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from nexus.layout import Cell

PRIORITY_RANK = {"high": 0, "normal": 1, "low": 2}
DISPATCH_PERIOD = 1.0  # s between generator / dispatcher rounds
ENERGY_MARGIN = 5.0    # % kept in reserve when accepting an order


@dataclass
class Mission:
    id: str
    name: str
    pickup: str
    dropoff: str
    priority: str
    created: float
    flow: Optional[str] = None
    source: str = "generator"
    status: str = "queued"
    robot: Optional[str] = None
    assigned_at: Optional[float] = None
    picked_at: Optional[float] = None
    completed_at: Optional[float] = None
    bid: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class FleetManager:

    def __init__(self, sim):
        self.sim = sim
        self.layout = sim.layout
        self.cfg = dict(self.layout.fleet)
        self.rng = random.Random(int(self.cfg.get("seed", 7)))
        self.missions: Dict[str, Mission] = {}
        self.queue: List[str] = []
        self.next_number = 1
        self.next_round = 0.0
        self.counters = {"created": 0, "completed": 0, "cancelled": 0, "requeued": 0, "charges": 0}
        self.lead_times: List[float] = []
        self.wait_times: List[float] = []

        self.robots = [a for a in sim.agents if sim.meta[a.robot_id]["mode"] == "dispatch"]
        self.chargers: List[Cell] = [
            (int(s["x"]), int(s["y"])) for s in self.layout.data["stations"] if s["type"] == "charging"
        ]
        self.charger_claims: Dict[Cell, str] = {}
        self._station_dist: Dict[Cell, Dict[Cell, float]] = {}

        for agent in self.robots:
            meta = sim.meta[agent.robot_id]
            meta["last_distance"] = 0.0
            meta["busy_until"] = None
            meta["charger"] = None

        # A few orders straight away so a demo starts moving immediately.
        if self.cfg.get("generator") and self.robots:
            for flow in self.layout.flows:
                if flow["enabled"] and flow["rate"] > 0:
                    self._generate(flow, 0.0)

    @property
    def active(self) -> bool:
        return bool(self.robots)

    # ================================================================ distances

    def _dijkstra(self, source: Cell) -> Dict[Cell, float]:
        """Lane-network travel cost (metres, slow cells weighted) from source."""
        layout = self.layout
        dist = {source: 0.0}
        heap = [(0.0, source)]
        while heap:
            d, cell = heapq.heappop(heap)
            if d > dist.get(cell, float("inf")):
                continue
            if cell != source and layout.is_station(cell):
                continue  # stations are endpoints only
            x, y = cell
            for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if not layout.move_allowed(cell, n):
                    continue
                nd = d + layout.cost_factor(n) * layout.cell_size
                if nd < dist.get(n, float("inf")):
                    dist[n] = nd
                    heapq.heappush(heap, (nd, n))
        return dist

    def station_distances(self, cell: Cell) -> Dict[Cell, float]:
        if cell not in self._station_dist:
            self._station_dist[cell] = self._dijkstra(cell)
        return self._station_dist[cell]

    def _nearest_charger_cost(self, cell: Cell) -> float:
        dist = self.station_distances(cell)
        costs = [dist[c] for c in self.chargers if c in dist]
        return min(costs) if costs else 0.0

    # ================================================================ missions

    def create(self, pickup: str, dropoff: str, priority: str = "normal", name: Optional[str] = None,
               flow: Optional[str] = None, source: str = "manual", now: Optional[float] = None) -> Mission:
        stations = self.layout.stations
        if pickup not in stations or dropoff not in stations:
            raise ValueError("Unknown pickup or drop-off station")
        if pickup == dropoff:
            raise ValueError("Pickup and drop-off must differ")
        if priority not in PRIORITY_RANK:
            raise ValueError("Priority must be high, normal or low")
        a = self.layout.resolve_cell(pickup)
        b = self.layout.resolve_cell(dropoff)
        if b not in self.station_distances(a):
            raise ValueError(f"No lane route from {stations[pickup]['label']} to {stations[dropoff]['label']}")
        now = self.sim.world.time if now is None else now
        mission = Mission(
            id=f"M-{self.next_number:04d}",
            name=name or f"{stations[pickup]['label']} → {stations[dropoff]['label']}",
            pickup=pickup, dropoff=dropoff, priority=priority, created=now, flow=flow, source=source,
        )
        self.next_number += 1
        self.missions[mission.id] = mission
        self.queue.append(mission.id)
        self.counters["created"] += 1
        if source == "manual":
            self.sim._event("mission", f"{mission.id} created: {mission.name} ({priority})")
        return mission

    def cancel(self, mission_id: str) -> Mission:
        mission = self.missions.get(mission_id)
        if mission is None:
            raise ValueError(f"No mission {mission_id}")
        if mission.status in ("completed", "cancelled"):
            raise ValueError(f"{mission_id} is already {mission.status}")
        if mission.status in ("loading", "to_dropoff", "unloading"):
            raise ValueError(f"{mission_id} is already loaded on {mission.robot}")
        if mission_id in self.queue:
            self.queue.remove(mission_id)
        if mission.robot:
            agent = self.sim._agent(mission.robot)
            meta = self.sim.meta[mission.robot]
            meta["mission"] = None
            self._go_home(agent)
        mission.status = "cancelled"
        mission.completed_at = self.sim.world.time
        self.counters["cancelled"] += 1
        self.sim._event("mission", f"{mission_id} cancelled", mission.robot)
        return mission

    def _generate(self, flow: Dict[str, Any], now: float):
        if len(self.queue) >= int(self.cfg.get("max_queue", 40)):
            return
        pairs = [(a, b) for a in flow["from"] for b in flow["to"] if a != b]
        if not pairs:
            return
        pickup, dropoff = self.rng.choice(pairs)
        try:
            self.create(pickup, dropoff, flow["priority"], name=None, flow=flow["id"], source="generator", now=now)
        except ValueError:
            pass

    # ================================================================ robot commands

    def _arrived(self, agent) -> bool:
        sim = self.sim
        meta = sim.meta[agent.robot_id]
        if agent.robot_id in sim.yielding or agent.stopped:
            return False
        if agent.planned_path and agent.waypoint_index < len(agent.planned_path):
            return False
        return sim._at_center(agent) and sim._cell(agent.state.position) == meta["goal_cell"]

    def _send(self, agent, cell: Cell, activity: str) -> bool:
        meta = self.sim.meta[agent.robot_id]
        if not self.sim.command(agent, cell):
            self.sim._event("error", f"{agent.robot_id}: no lane route to {self.sim._station_name(cell)}", agent.robot_id)
            return False
        meta["activity"] = activity
        return True

    def _go_home(self, agent):
        meta = self.sim.meta[agent.robot_id]
        self._release_charger(agent)
        self._send(agent, meta["home_cell"], "RETURNING")

    def _release_charger(self, agent):
        meta = self.sim.meta[agent.robot_id]
        if meta.get("charger") is not None:
            self.charger_claims.pop(meta["charger"], None)
            meta["charger"] = None

    def _go_charge(self, agent, now: float) -> bool:
        meta = self.sim.meta[agent.robot_id]
        dist = self._dijkstra(self.sim._cell(agent.state.position))
        free = [c for c in self.chargers if c not in self.charger_claims and c in dist]
        if not free:
            meta["needs_charge"] = True
            if meta["activity"] not in ("RETURNING", "IDLE"):
                self._go_home(agent)
            return False
        charger = min(free, key=lambda c: dist[c])
        self.charger_claims[charger] = agent.robot_id
        meta["charger"] = charger
        meta["needs_charge"] = False
        self._send(agent, charger, "TO_CHARGER")
        self.sim._event("battery", f"{agent.robot_id} battery {meta['battery']:.0f}% → going to {self.sim._station_name(charger)}", agent.robot_id)
        return True

    # ================================================================ update

    def update(self, now: float):
        """Robot state machines every step; generator + dispatcher every second."""
        cfg = self.cfg
        for agent in self.robots:
            meta = self.sim.meta[agent.robot_id]
            activity = meta["activity"]
            mission = self.missions.get(meta["mission"]) if meta["mission"] else None

            if activity == "TO_PICKUP" and meta["battery"] < cfg["battery_critical"] and mission:
                # Not loaded yet: hand the order back and charge now.
                mission.status, mission.robot, mission.assigned_at = "queued", None, None
                self.queue.insert(0, mission.id)
                meta["mission"] = None
                self.counters["requeued"] += 1
                self.sim._event("battery", f"{agent.robot_id} battery critical: {mission.id} back to the queue", agent.robot_id)
                self._go_charge(agent, now)
                continue

            if activity == "TO_PICKUP" and self._arrived(agent):
                meta["activity"] = "LOADING"
                meta["busy_until"] = now + cfg["service_time"]
                mission.status = "loading"
            elif activity == "LOADING" and now >= meta["busy_until"]:
                mission.status = "to_dropoff"
                mission.picked_at = now
                self._send(agent, self.layout.resolve_cell(mission.dropoff), "TO_DROPOFF")
            elif activity == "TO_DROPOFF" and self._arrived(agent):
                meta["activity"] = "UNLOADING"
                meta["busy_until"] = now + cfg["service_time"]
                mission.status = "unloading"
            elif activity == "UNLOADING" and now >= meta["busy_until"]:
                mission.status = "completed"
                mission.completed_at = now
                self.counters["completed"] += 1
                self.lead_times.append(now - mission.created)
                meta["trips"] += 1
                meta["mission"] = None
                self.sim.counters["trips_completed"] += 1
                self.sim._event("arrival", f"{mission.id} delivered by {agent.robot_id}: {mission.name} "
                                f"({now - mission.created:.0f} s)", agent.robot_id)
                if meta["battery"] < cfg["battery_low"]:
                    self._go_charge(agent, now)
                else:
                    self._go_home(agent)
            elif activity == "TO_CHARGER" and self._arrived(agent):
                meta["activity"] = "CHARGING"
                self.counters["charges"] += 1
            elif activity == "CHARGING" and meta["battery"] >= cfg["charge_target"]:
                self.sim._event("battery", f"{agent.robot_id} charged to {meta['battery']:.0f}%, back in service", agent.robot_id)
                self._go_home(agent)
            elif activity == "RETURNING" and self._arrived(agent):
                meta["activity"] = "IDLE"

            if meta["activity"] in ("IDLE", "RETURNING") and (meta["battery"] < cfg["battery_low"] or meta.get("needs_charge")):
                if now >= self.next_round - 1e-9:  # retry at dispatcher pace
                    self._go_charge(agent, now)

        if now + 1e-9 >= self.next_round:
            self.next_round = now + DISPATCH_PERIOD
            if cfg.get("generator"):
                for flow in self.layout.flows:
                    if flow["enabled"] and flow["rate"] > 0 and self.rng.random() < flow["rate"] * DISPATCH_PERIOD / 3600.0:
                        self._generate(flow, now)
            self._dispatch(now)

    def _dispatch(self, now: float):
        cfg = self.cfg
        free = []
        for agent in self.robots:
            meta = self.sim.meta[agent.robot_id]
            if meta["activity"] not in ("IDLE", "RETURNING") or meta.get("needs_charge"):
                continue
            if meta["battery"] < cfg["battery_low"] or agent.robot_id in self.sim.yielding or agent.stopped:
                continue
            free.append(agent)
        if not free or not self.queue:
            return

        searches = {a.robot_id: self._dijkstra(self.sim._cell(a.state.position)) for a in free}
        order = sorted(self.queue, key=lambda mid: (PRIORITY_RANK[self.missions[mid].priority], self.missions[mid].created))
        for mission_id in order:
            if not free:
                break
            mission = self.missions[mission_id]
            pickup = self.layout.resolve_cell(mission.pickup)
            dropoff = self.layout.resolve_cell(mission.dropoff)
            delivery = self.station_distances(pickup).get(dropoff)
            if delivery is None:
                continue
            after = self._nearest_charger_cost(dropoff)
            best, best_bid = None, None
            for agent in free:
                to_pickup = searches[agent.robot_id].get(pickup)
                if to_pickup is None:
                    continue
                meta = self.sim.meta[agent.robot_id]
                energy = (to_pickup + delivery + after) * cfg["drain_per_m"] + ENERGY_MARGIN
                if meta["battery"] - energy < cfg["battery_critical"]:
                    continue
                bid = to_pickup / max(agent.max_speed, 1e-6)
                if best_bid is None or bid < best_bid:
                    best, best_bid = agent, bid
            if best is None:
                continue
            meta = self.sim.meta[best.robot_id]
            self.queue.remove(mission_id)
            mission.status = "assigned"
            mission.robot = best.robot_id
            mission.assigned_at = now
            mission.bid = round(best_bid, 1)
            self.wait_times.append(now - mission.created)
            meta["mission"] = mission_id
            free.remove(best)
            if self._send(best, pickup, "TO_PICKUP"):
                self.sim._event("mission", f"{mission.id} → {best.robot_id} (bid {best_bid:.0f} s): {mission.name}", best.robot_id)

    def after_motion(self, dt: float):
        cfg = self.cfg
        for agent in self.robots:
            meta = self.sim.meta[agent.robot_id]
            travelled = agent.robot.distance_travelled - meta["last_distance"]
            meta["last_distance"] = agent.robot.distance_travelled
            battery = meta["battery"] - travelled * cfg["drain_per_m"]
            if meta["activity"] == "CHARGING":
                battery += cfg["charge_rate"] * dt
            elif meta["activity"] == "IDLE":
                battery -= cfg["drain_idle"] * dt
            meta["battery"] = float(min(100.0, max(0.0, battery)))

    # ================================================================ reporting

    def _mission_view(self, mission: Mission) -> Dict[str, Any]:
        view = mission.to_dict()
        stations = self.layout.stations
        view["pickup_label"] = stations[mission.pickup]["label"]
        view["dropoff_label"] = stations[mission.dropoff]["label"]
        return view

    def snapshot(self) -> Dict[str, Any]:
        queued = sorted((self.missions[m] for m in self.queue),
                        key=lambda m: (PRIORITY_RANK[m.priority], m.created))
        active = [m for m in self.missions.values() if m.status in ("assigned", "loading", "to_dropoff", "unloading")]
        done = sorted((m for m in self.missions.values() if m.status in ("completed", "cancelled")),
                      key=lambda m: m.completed_at or 0.0, reverse=True)
        # Keep memory bounded on long runs.
        if len(self.missions) > 600:
            for m in done[200:]:
                self.missions.pop(m.id, None)
        return {
            "queued": [self._mission_view(m) for m in queued[:30]],
            "active": [self._mission_view(m) for m in sorted(active, key=lambda m: m.id)],
            "recent": [self._mission_view(m) for m in done[:12]],
            "generator": bool(self.cfg.get("generator")),
            "metrics": self.metrics(),
        }

    def metrics(self) -> Dict[str, Any]:
        t = max(self.sim.world.time, 1e-6)
        batteries = [self.sim.meta[a.robot_id]["battery"] for a in self.robots]
        mean = lambda xs: round(sum(xs) / len(xs), 1) if xs else None
        return {
            **self.counters,
            "queued": len(self.queue),
            "active": sum(1 for m in self.missions.values() if m.status in ("assigned", "loading", "to_dropoff", "unloading")),
            "per_hour": round(self.counters["completed"] / t * 3600.0, 1),
            "avg_lead_time": mean(self.lead_times[-200:]),
            "avg_wait": mean(self.wait_times[-200:]),
            "avg_battery": mean(batteries),
            "charging": sum(1 for a in self.robots if self.sim.meta[a.robot_id]["activity"] in ("CHARGING", "TO_CHARGER")),
            "dispatched_robots": len(self.robots),
        }


__all__ = ["FleetManager", "Mission", "PRIORITY_RANK"]
