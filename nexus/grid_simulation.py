"""
Grid (lane) simulation runtime for NEXUS.

Runs the existing NEXUS stack on a painted lane layout:

    RobotAgent          local state + planned path          (agents/robot_agent.py)
    P2PNetwork          decentralised state exchange        (communication/p2p.py)
    NEXUSPlanner        A*, here over a LaneGrid            (algorithms/planner.py, lane_grid.py)
    ConflictDetector    predictive trajectory conflicts     (algorithms/conflict_detector.py)
    NegotiationManager  ETA-based priority                  (algorithms/negotiation.py)
    RobotAgent.replan_around  dynamic rerouting

plus grid-specific coordination:

    CellReservationTable  robots reserve cells before entering them
    deadlock resolution   wait-for cycles -> negotiation loser backs off to a side cell
    via-point routes      per-robot custom paths honoured through every replan

and a kinematic GridWorld instead of IR-SIM physics.
"""

from __future__ import annotations

from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from agents.robot_agent import RobotAgent
from algorithms.cell_reservation import CellReservationTable
from algorithms.conflict_detector import PredictedConflict
from algorithms.fast_conflict_detector import FastConflictDetector
from algorithms.lane_grid import LaneGrid
from algorithms.negotiation import NegotiationManager
from algorithms.planner import NEXUSPlanner
from communication.p2p import P2PNetwork
from nexus.fleet import FleetManager
from nexus.grid_world import GridWorld
from nexus.layout import Cell, Layout

AT_CENTER = 1e-3          # metres from a cell centre that count as "on" it (motion lands exactly)
PARK_REROUTE_AFTER = 2.0  # s blocked by a parked/idle robot before rerouting
YIELD_TIMEOUT = 25.0      # s a backed-off robot waits before retrying anyway


class RoutePlanner:
    """
    Per-robot planner that threads A* legs through the robot's remaining
    via-points. Drop-in for NEXUSPlanner (same plan() signature), so
    RobotAgent.plan_path / replan_around honour custom routes automatically.
    """

    def __init__(self, base: NEXUSPlanner, via_points: List[np.ndarray]):
        self.base = base
        self.via = [np.asarray(p, dtype=float) for p in via_points]
        self.next_via = 0

    def remaining_via(self) -> List[np.ndarray]:
        return self.via[self.next_via:]

    def plan(self, start, goal, dynamic_obstacles=None):
        stops = [np.asarray(start, dtype=float)] + self.remaining_via() + [np.asarray(goal, dtype=float)]
        path: List[np.ndarray] = []
        for a, b in zip(stops, stops[1:]):
            leg = self.base.plan(a, b, dynamic_obstacles=dynamic_obstacles)
            if not leg:
                return []
            path.extend(leg if not path else leg[1:])
        return path


class GridSimulation:

    def __init__(self, layout: Layout):
        self.layout = layout
        self.reset()

    # ==================================================================
    # BUILD
    # ==================================================================

    def reset(self):
        layout = self.layout
        sim = layout.simulation
        cs = layout.cell_size

        self.time_step = float(sim["time_step"])
        self.max_time = float(sim["max_time"])
        self.lookahead = int(sim["reservation_lookahead"])

        self.grid = LaneGrid(layout, turn_penalty=float(sim["turn_penalty"]))
        self.planner = NEXUSPlanner(grid=self.grid)
        # Same results as NEXUS's ConflictDetector, computed faster (see
        # algorithms/fast_conflict_detector.py).
        self.detector = FastConflictDetector(
            prediction_horizon=3.0,
            prediction_dt=0.2,
            # Adjacent lanes are one cell apart; only same-cell encounters conflict.
            safety_distance=0.9 * cs,
        )
        self.negotiation = NegotiationManager(eta_margin=0.25)
        self.network = P2PNetwork()
        self.reservations = CellReservationTable()
        self.world = GridWorld(
            max_speed=float(sim["max_speed"]),
            acceleration=float(sim["acceleration"]),
            angular_speed=float(sim["angular_speed"]),
            time_step=self.time_step,
        )
        # Reach a zone's speed limit by its edge, one step's travel early so
        # no 0.1 s step crosses the edge above the limit.
        self.world.cap_lead = 0.5 * cs + self.world.max_speed * self.time_step

        self.agents: List[RobotAgent] = []
        self.meta: Dict[str, Dict[str, Any]] = {}
        self.active_conflicts: Dict[Tuple[str, str], Dict[str, Any]] = {}
        self.yielding: Dict[str, Dict[str, Any]] = {}
        self.predicted: List[PredictedConflict] = []
        self.stuck_since: Dict[Tuple[str, str], float] = {}
        self.stuck_reported = set()
        self.events: List[Dict[str, Any]] = []
        self.counters = {
            "conflicts_predicted": 0,
            "negotiations": 0,
            "reroutes": 0,
            "backoffs": 0,
            "deadlocks_resolved": 0,
            "safety_violations": 0,
            "trips_completed": 0,
        }
        self.min_separation = float("inf")
        self.status = "READY"
        self.errors: List[str] = [i["message"] for i in layout.validate() if i["severity"] == "error"]

        if not self.errors:
            self._create_agents()
        self.fleet = FleetManager(self)

        dispatched = len(self.fleet.robots)
        self._event("system", f"Layout '{layout.name}' loaded: {len(self.agents)} robots"
                    + (f" ({dispatched} dispatched)" if dispatched else ""))
        for message in self.errors:
            self._event("error", message)
        if self.errors:
            self.status = "INVALID"

    def _create_agents(self):
        layout = self.layout
        for index, config in enumerate(layout.robots):
            robot_id = str(config["id"])
            dispatched = config["mode"] == "dispatch"
            start_cell = layout.resolve_cell(config["start"])
            # Dispatched robots wait at home until the fleet manager sends them.
            goal_cell = start_cell if dispatched else layout.resolve_cell(config["goal"])
            start = np.array(layout.cell_center(start_cell))
            goal = np.array(layout.cell_center(goal_cell))

            via = [np.array(layout.cell_center((int(v[0]), int(v[1])))) for v in config.get("via", [])]
            route_planner = RoutePlanner(self.planner, via)

            robot = self.world.add_robot(start, 0.0)
            agent = RobotAgent(robot_id=robot_id, robot=robot, planner=route_planner)
            agent.max_speed = min(float(config.get("max_speed", self.world.max_speed)), self.world.max_speed)
            # Progress is reported exactly by GridWorld; never skip a corner early.
            agent.waypoint_tolerance = 1e-9
            agent.start_position = start.copy()
            agent.update(0.0)
            agent.set_task(task_id=config.get("task_id") or f"T-{robot_id}", goal=goal)

            self.meta[robot_id] = {
                "index": index,
                "color": layout.robot_color(index, config),
                "start_cell": start_cell,
                "goal_cell": goal_cell,
                "home_cell": start_cell,
                "loop": bool(config.get("loop", False)),
                "outbound": True,
                "dwell_until": None,
                "blocked_by": None,
                "blocked_since": None,
                "status": "IDLE",
                "trips": 0,
                "arrival_logged": False,
                "reroute_flash": 0.0,
                "error": None,
                "mode": config["mode"],
                "activity": "IDLE" if dispatched else "ROUTE",
                "mission": None,
                "battery": float(config.get("battery", 100.0)) if dispatched else None,
                "needs_charge": False,
            }

            if dispatched:
                agent.set_path([start])
                agent.advance_waypoint()
                self.agents.append(agent)
                self.network.register(agent)
                continue

            try:
                agent.plan_path(start=start, goal=goal)
            except RuntimeError:
                agent.set_path([])
                agent.intent = "IDLE"
                self.meta[robot_id]["error"] = "No lane path to goal"
                self._event("error", f"{robot_id}: no lane path from start to goal", robot_id)

            # Face along the first move so the robot does not spin at t=0.
            if len(agent.planned_path) >= 2:
                d = agent.planned_path[1] - agent.planned_path[0]
                robot.state[2] = float(np.arctan2(d[1], d[0]))

            self.agents.append(agent)
            self.network.register(agent)

        for agent in self.agents:
            agent.update(0.0)
            self.reservations.reserve(agent.robot_id, self._cell(agent.state.position))

    # ==================================================================
    # HELPERS
    # ==================================================================

    @property
    def time(self) -> float:
        return float(self.world.time)

    def _event(self, kind: str, text: str, robot: Optional[str] = None, **extra):
        self.events.append({"t": round(self.world.time if hasattr(self, "world") else 0.0, 1),
                            "kind": kind, "text": text, "robot": robot, **extra})
        if len(self.events) > 300:
            self.events = self.events[-300:]

    def _cell(self, point) -> Cell:
        return self.layout.world_to_cell(point)

    def _agent(self, robot_id: str) -> Optional[RobotAgent]:
        return next((a for a in self.agents if a.robot_id == robot_id), None)

    def _at_center(self, agent: RobotAgent) -> bool:
        centre = np.array(self.layout.cell_center(self._cell(agent.state.position)))
        return float(np.linalg.norm(centre - agent.state.position)) < AT_CENTER

    def _occupied(self, agent: RobotAgent) -> List[Cell]:
        """Cells the robot body overlaps (one at a centre, two between centres)."""
        position = agent.state.position
        here = self._cell(position)
        cells = [here]
        if not self._at_center(agent):
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (here[0] + dx, here[1] + dy)
                if self.layout.in_bounds(n):
                    c = np.array(self.layout.cell_center(n))
                    if np.linalg.norm(c - position) < self.layout.cell_size - 1e-6:
                        cells.append(n)
        return cells

    def _upcoming(self, agent: RobotAgent) -> List[Tuple[int, Cell]]:
        """(path index, cell) for the rest of the planned path."""
        return [(j, self._cell(agent.planned_path[j])) for j in range(agent.waypoint_index, len(agent.planned_path))]

    def _remaining_cells(self, agent: RobotAgent) -> List[Cell]:
        return [cell for _, cell in self._upcoming(agent)]

    def _is_intersection(self, cell: Cell) -> bool:
        neighbours = [(cell[0] + dx, cell[1] + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
        links = sum(1 for n in neighbours if self.layout.move_allowed(cell, n) or self.layout.move_allowed(n, cell))
        return links > 2

    def _parked(self, agent: RobotAgent) -> bool:
        return agent.intent in {"ARRIVED", "IDLE"} and agent.robot_id not in self.yielding

    def command(self, agent: RobotAgent, cell: Cell) -> bool:
        """Send a robot to a cell (used by the fleet manager). A robot that is
        backing off picks the new goal up when it resumes."""
        meta = self.meta[agent.robot_id]
        goal = np.array(self.layout.cell_center(cell))
        meta["goal_cell"] = cell
        meta["dwell_until"] = None
        agent.set_task(task_id=agent.state.task_id, goal=goal)
        if agent.robot_id in self.yielding:
            return True
        path = agent.planner.plan(agent.state.position, goal)
        if not path:
            return False
        self._install_path(agent, path)
        if len(path) <= 1 or agent.waypoint_index >= len(agent.planned_path):
            agent.intent = "ARRIVED"
        return True

    def _install_path(self, agent: RobotAgent, path: List[np.ndarray]):
        self.meta[agent.robot_id]["arrival_logged"] = False
        agent.set_path(path)
        agent.advance_waypoint()
        if agent.waypoint_index < len(agent.planned_path):
            agent.intent = "MOVING"

    # ==================================================================
    # STEP
    # ==================================================================

    def step(self) -> Dict[str, Any]:
        if self.status in {"INVALID", "COMPLETED"}:
            return self.snapshot()
        if self.status == "READY":
            self.status = "RUNNING"

        now = self.world.time

        # 1. Local state + P2P exchange -----------------------------------
        for agent in self.agents:
            agent.update(now)
        self.network.broadcast_all()

        # 2. Path progress, via-points, arrivals, loops -------------------
        for agent in self.agents:
            self._progress(agent, now)
        self.fleet.update(now)

        # 3. Predictive conflicts -> negotiation -> reroute / stop --------
        self._release_conflicts(now)
        self.predicted = self.detector.detect_all(self.agents)
        self._negotiate(self.predicted, now)

        # 4. Cell reservations -------------------------------------------
        routes, wait_for = self._reserve(now)

        # 5. Deadlocks and long blocks ------------------------------------
        self._resolve_blocks(wait_for, now)
        self._resume_yielders(now)
        # Paths may have changed above: routes must match the paths the
        # world is about to drive (waypoint progress is counted per route).
        routes, _ = self._reserve(now)

        # 6. Motion --------------------------------------------------------
        speeds, caps = self._zone_limits(routes)
        reached = self.world.step(routes, speeds, caps)
        for agent, count in zip(self.agents, reached):
            agent.update(self.world.time)
            # The world drives straight through several cell centres per step;
            # route i always starts at planned_path[waypoint_index].
            agent.waypoint_index += count
            if agent.planned_path:
                agent.advance_waypoint()

        self.fleet.after_motion(self.time_step)
        self._safety_check()
        for agent in self.agents:
            self._log_arrival(agent, self.world.time)
        self._update_status()

        if self.agents and not self.fleet.active \
                and all(self.meta[a.robot_id]["status"] == "ARRIVED" for a in self.agents) \
                and not any(self.meta[a.robot_id]["loop"] for a in self.agents):
            self.status = "COMPLETED"
            self._event("system", f"All robots arrived in {self.world.time:.1f}s")
        elif self.world.time >= self.max_time:
            self.status = "COMPLETED"
            self._event("system", "Time limit reached")

        return self.snapshot()

    # ------------------------------------------------------------------

    def _progress(self, agent: RobotAgent, now: float):
        meta = self.meta[agent.robot_id]
        planner: RoutePlanner = agent.planner

        if planner.next_via < len(planner.via) and self._at_center(agent):
            if self._cell(agent.state.position) == self._cell(planner.via[planner.next_via]):
                planner.next_via += 1
                self._event("route", f"{agent.robot_id} passed via-point {planner.next_via}/{len(planner.via)}", agent.robot_id)

        if agent.robot_id in self.yielding or agent.stopped or meta["mode"] == "dispatch":
            return

        if agent.planned_path and agent.waypoint_index >= len(agent.planned_path) and agent.intent != "ARRIVED":
            agent.intent = "ARRIVED"

        self._log_arrival(agent, now)

        if meta["loop"] and meta["dwell_until"] is not None and now >= meta["dwell_until"]:
            meta["dwell_until"] = None
            meta["start_cell"], meta["goal_cell"] = meta["goal_cell"], meta["start_cell"]
            meta["outbound"] = not meta["outbound"]
            planner.next_via = 0 if meta["outbound"] else len(planner.via)
            goal = np.array(self.layout.cell_center(meta["goal_cell"]))
            agent.set_task(task_id=agent.state.task_id, goal=goal)
            path = planner.plan(agent.state.position, goal)
            if path:
                self._install_path(agent, path)
            else:
                agent.intent = "ARRIVED"
                meta["dwell_until"] = now + 2.0

    def _log_arrival(self, agent: RobotAgent, now: float):
        meta = self.meta[agent.robot_id]
        if agent.robot_id in self.yielding or meta["arrival_logged"] or meta["mode"] == "dispatch":
            return
        if agent.intent == "ARRIVED" and self._cell(agent.state.position) == meta["goal_cell"]:
            meta["arrival_logged"] = True
            meta["trips"] += 1
            self.counters["trips_completed"] += 1
            self._event("arrival", f"{agent.robot_id} arrived at {self._station_name(meta['goal_cell'])}", agent.robot_id)
            if meta["loop"]:
                meta["dwell_until"] = now + 2.0

    def _station_name(self, cell: Cell) -> str:
        station = self.layout.station_cells.get(cell)
        return station["label"] if station else f"cell {list(cell)}"

    def _zone_limits(self, routes: List[List[np.ndarray]]):
        """Per robot: speed limit now (its max and any safety zone it is in),
        and the zone cap of every cell on its route this step."""
        speeds, caps = [], []
        for agent, route in zip(self.agents, routes):
            limit = agent.max_speed
            # The cell under the robot's centre: the look-ahead caps below
            # already brake it to the limit by the zone's edge.
            cap = self.layout.speed_limit(self._cell(agent.state.position))
            if cap is not None:
                limit = min(limit, cap)
            speeds.append(limit)
            caps.append([self.layout.speed_limit(self._cell(point)) for point in route])
        return speeds, caps

    # ------------------------------------------------------------------
    # Negotiation (existing NEXUS conflict flow)
    # ------------------------------------------------------------------

    def _release_conflicts(self, now: float):
        for pair, info in list(self.active_conflicts.items()):
            winner, loser = info["winner"], info["loser"]
            point = info["position"]
            cs = self.layout.cell_size
            far = np.linalg.norm(winner.state.position - point) >= 1.5 * cs
            # The conflict is detected seconds ahead, so the winner starts out
            # "far" from it: only release once the point is behind the winner.
            ahead = any(
                np.linalg.norm(p - point) < 0.9 * cs
                for p in winner.planned_path[winner.waypoint_index:]
            )
            gone = winner.intent == "ARRIVED" or not self._remaining_cells(winner)
            if (far and not ahead and not winner.stopped) or gone or now - info["since"] > 20.0:
                self._event("resume", f"{winner.robot_id} cleared -> {loser.robot_id} resumes", loser.robot_id)
                self._drop_conflict(pair)

        # Safety net: a negotiation stop must always belong to a live conflict.
        losers = {info["loser"].robot_id for info in self.active_conflicts.values()}
        for agent in self.agents:
            if agent.stopped and agent.robot_id not in losers and agent.robot_id not in self.yielding:
                agent.resume()

    def _drop_conflict(self, pair):
        info = self.active_conflicts.pop(pair, None)
        if info is not None and info["loser"].stopped and info["loser"].robot_id not in self.yielding:
            info["loser"].resume()

    def _negotiate(self, conflicts: List[PredictedConflict], now: float):
        for conflict in conflicts:
            pair = tuple(sorted([conflict.robot_a, conflict.robot_b]))
            if pair in self.active_conflicts:
                continue
            a, b = self._agent(conflict.robot_a), self._agent(conflict.robot_b)
            if a is None or b is None or a.stopped or b.stopped:
                continue
            if a.robot_id in self.yielding or b.robot_id in self.yielding:
                continue
            if not self._needs_negotiation(a, b):
                continue

            self.counters["conflicts_predicted"] += 1
            self.counters["negotiations"] += 1
            win_d, lose_d = self.negotiation.negotiate(a, b, conflict)
            winner, loser = self._agent(win_d.robot_id), self._agent(lose_d.robot_id)

            loser.stop(reason=f"WAIT_FOR_{winner.robot_id}")

            action = "PROCEED"
            loser_cells = set(self._occupied(loser))
            if loser_cells & set(self._remaining_cells(winner)):
                if winner.replan_around(loser.state.position, obstacle_radius=0.6 * self.layout.cell_size):
                    winner.advance_waypoint()
                    self.counters["reroutes"] += 1
                    self.meta[winner.robot_id]["reroute_flash"] = now + 1.5
                    action = "REROUTE"

            self.active_conflicts[pair] = {
                "winner": winner,
                "loser": loser,
                "position": np.asarray(conflict.conflict_position, dtype=float).copy(),
                "since": now,
            }
            self._event(
                "conflict",
                f"Conflict {a.robot_id}↔{b.robot_id} in {conflict.time_to_conflict:.1f}s: "
                f"{winner.robot_id} {action.lower()}s, {loser.robot_id} yields ({win_d.reason})",
                winner.robot_id,
                position=[float(v) for v in conflict.conflict_position],
                pair=[a.robot_id, b.robot_id],
            )

    def _needs_negotiation(self, a: RobotAgent, b: RobotAgent) -> bool:
        """
        Crossing and head-on encounters are negotiated. Queuing behind a
        waiting/parked robot, or following one in the same direction, is
        already handled by cell reservations.
        """
        settled = {"WAITING", "DOCKED", "ARRIVED", "IDLE", "ERROR"}
        if self.meta[a.robot_id]["status"] in settled or self.meta[b.robot_id]["status"] in settled:
            return False
        if a.intent in {"STOPPED", "ARRIVED", "IDLE"} or b.intent in {"STOPPED", "ARRIVED", "IDLE"}:
            return False
        ha, hb = a.robot.heading, b.robot.heading
        return float(np.cos(ha - hb)) < 0.9

    # ------------------------------------------------------------------
    # Cell reservations
    # ------------------------------------------------------------------

    def _priority(self, agent: RobotAgent):
        losing = any(info["loser"] is agent for info in self.active_conflicts.values())
        eta = agent.state.eta if agent.state.eta is not None else 0.0
        return (agent.stopped or losing,) + self.reservations.priority_key(eta, agent.robot_id)

    def _reserve(self, now: float):
        routes: List[List[np.ndarray]] = [[] for _ in self.agents]
        wait_for: Dict[str, str] = {}
        occupied = {a.robot_id: self._occupied(a) for a in self.agents}

        # Bodies first: every robot always holds the cells it overlaps.
        for agent in self.agents:
            upcoming = set(self._remaining_cells(agent)[: self.lookahead + 1])
            keep = set(occupied[agent.robot_id]) | (set(self.reservations.held_by(agent.robot_id)) & upcoming)
            self.reservations.retain_only(agent.robot_id, keep)
            for cell in occupied[agent.robot_id]:
                self.reservations.reserve(agent.robot_id, cell)

        order = sorted(range(len(self.agents)), key=lambda i: self._priority(self.agents[i]))
        for i in order:
            agent = self.agents[i]
            meta = self.meta[agent.robot_id]
            upcoming = self._upcoming(agent)
            body = set(occupied[agent.robot_id])
            chain: List[int] = []
            blocked_by = None

            for j, cell in upcoming:
                if cell in body:
                    chain.append(j)
                    continue
                if agent.stopped or meta["dwell_until"] is not None:
                    break
                if len(chain) >= self.lookahead:
                    break
                if not self.reservations.reserve(agent.robot_id, cell):
                    blocked_by = self.reservations.holder(cell)
                    break
                chain.append(j)

            # Don't stop inside an intersection: back the chain off to the
            # last non-intersection cell unless we are already inside it.
            while chain and blocked_by is not None:
                last_cell = self._cell(agent.planned_path[chain[-1]])
                if last_cell in body or not self._is_intersection(last_cell):
                    break
                self.reservations.retain_only(agent.robot_id, set(self.reservations.held_by(agent.robot_id)) - {last_cell})
                chain.pop()

            routes[i] = [agent.planned_path[j] for j in chain]

            if blocked_by is not None:
                wait_for[agent.robot_id] = blocked_by
                if meta["blocked_by"] != blocked_by:
                    meta["blocked_by"] = blocked_by
                    meta["blocked_since"] = now
            else:
                meta["blocked_by"] = None
                meta["blocked_since"] = None

        # A negotiation loser implicitly waits for its winner.
        for info in self.active_conflicts.values():
            if info["loser"].stopped:
                wait_for.setdefault(info["loser"].robot_id, info["winner"].robot_id)
        # A backed-off robot waits for the robot it yielded to.
        for robot_id, info in self.yielding.items():
            wait_for.setdefault(robot_id, info["winner"])

        return routes, wait_for

    # ------------------------------------------------------------------
    # Deadlocks
    # ------------------------------------------------------------------

    def _resolve_blocks(self, wait_for: Dict[str, str], now: float):
        # Only real blocking edges (robot has somewhere to go and is stuck).
        blocking = {r: s for r, s in wait_for.items() if r not in self.yielding}
        cycle = self.reservations.find_cycle(blocking)
        if cycle:
            self._break_cycle(cycle, now)
            return

        for robot_id, blocker_id in wait_for.items():
            if robot_id in self.yielding:
                continue
            meta = self.meta[robot_id]
            if meta["blocked_since"] is None or now - meta["blocked_since"] < PARK_REROUTE_AFTER:
                continue
            agent, blocker = self._agent(robot_id), self._agent(blocker_id)
            if agent is None or blocker is None or agent.stopped:
                continue
            if not (self._parked(blocker) or blocker.robot_id in self.yielding
                    or self.meta[blocker_id]["blocked_since"] is not None):
                continue

            if agent.replan_around(blocker.state.position, obstacle_radius=0.6 * self.layout.cell_size):
                agent.advance_waypoint()
                self.counters["reroutes"] += 1
                meta["reroute_flash"] = now + 1.5
                meta["blocked_since"] = now
                self._event("reroute", f"{robot_id} reroutes around {blocker_id}", robot_id)
            elif self._parked(blocker) and self._back_off(blocker, agent, now):
                self._event("backoff", f"{blocker_id} makes way for {robot_id}", blocker_id)
            else:
                meta["blocked_since"] = now  # try again later

    def _break_cycle(self, cycle: List[str], now: float):
        a = self._agent(cycle[0])
        b = self._agent(cycle[1] if len(cycle) > 1 else cycle[0])
        pair = tuple(sorted([a.robot_id, b.robot_id]))
        if pair in self.active_conflicts:
            winner, loser = self.active_conflicts[pair]["winner"], self.active_conflicts[pair]["loser"]
        else:
            midpoint = (a.state.position + b.state.position) / 2.0
            conflict = PredictedConflict(
                robot_a=a.robot_id, robot_b=b.robot_id, time_to_conflict=0.0,
                position_a=a.state.position.copy(), position_b=b.state.position.copy(),
                conflict_position=midpoint, minimum_distance=float(np.linalg.norm(a.state.position - b.state.position)),
                conflict_type="DEADLOCK",
            )
            win_d, lose_d = self.negotiation.negotiate(a, b, conflict)
            winner, loser = self._agent(win_d.robot_id), self._agent(lose_d.robot_id)
            self.counters["negotiations"] += 1

        for yielder, other in ((loser, winner), (winner, loser)):
            if self._back_off(yielder, other, now):
                self.stuck_since.pop(pair, None)
                self._drop_conflict(pair)
                self.counters["deadlocks_resolved"] += 1
                self._event(
                    "deadlock",
                    f"Deadlock {' → '.join(cycle)} → {cycle[0]}: {yielder.robot_id} backs off for {other.robot_id}",
                    yielder.robot_id,
                    position=[float(v) for v in yielder.state.position],
                )
                return

        # No side cell right now; usually traffic clears one within seconds.
        first = self.stuck_since.setdefault(pair, now)
        if now - first > 10.0 and pair not in self.stuck_reported:
            self.stuck_reported.add(pair)
            self._event("error", f"Deadlock {a.robot_id}↔{b.robot_id}: no side cell to back off into for 10 s", a.robot_id)

    def _back_off(self, yielder: RobotAgent, winner: RobotAgent, now: float) -> bool:
        """Move yielder to the nearest free cell off the winner's path."""
        start = self._cell(yielder.state.position)
        avoid = set(self._remaining_cells(winner)) | set(self._occupied(winner))
        parents: Dict[Cell, Optional[Cell]] = {start: None}
        queue = deque([(start, 0)])
        target = None
        while queue:
            cell, depth = queue.popleft()
            if cell != start and cell not in avoid:
                target = cell
                break
            if depth >= 20:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (cell[0] + dx, cell[1] + dy)
                if n in parents or not self.layout.move_allowed(cell, n):
                    continue
                if self.reservations.holder(n) not in (None, yielder.robot_id):
                    continue
                # A free station is a fine bay, but never driven through.
                if self.layout.is_station(n) and n in avoid:
                    continue
                parents[n] = cell
                queue.append((n, depth + 1))

        if target is None:
            return False

        cells = [target]
        while parents[cells[-1]] is not None:
            cells.append(parents[cells[-1]])
        cells.reverse()
        path = [np.array(self.layout.cell_center(c)) for c in cells]
        if yielder.stopped:
            yielder.resume()
        yielder.set_path(path)
        yielder.advance_waypoint()
        yielder.intent = "REROUTING"
        self.yielding[yielder.robot_id] = {"winner": winner.robot_id, "bay": target, "since": now}
        self.meta[yielder.robot_id]["dwell_until"] = None
        self.counters["backoffs"] += 1
        return True

    def _resume_yielders(self, now: float):
        for robot_id, info in list(self.yielding.items()):
            agent = self._agent(robot_id)
            winner = self._agent(info["winner"])
            meta = self.meta[robot_id]
            if agent.waypoint_index < len(agent.planned_path):
                continue  # still driving to the bay
            goal = np.array(self.layout.cell_center(meta["goal_cell"]))
            path = agent.planner.plan(agent.state.position, goal)
            if not path:
                continue
            winner_cells = set(self._remaining_cells(winner)) | set(self._occupied(winner))
            ahead = {self._cell(p) for p in path[: 2 * self.lookahead + 2]}
            clear = not (ahead & winner_cells) or winner.intent == "ARRIVED" and not (ahead & set(self._occupied(winner)))
            if clear or now - info["since"] > YIELD_TIMEOUT:
                del self.yielding[robot_id]
                agent.set_task(task_id=agent.state.task_id, goal=goal)
                self._install_path(agent, path)
                if len(path) <= 1:
                    agent.intent = "ARRIVED"
                self._event("resume", f"{robot_id} resumes toward {self._station_name(meta['goal_cell'])}", robot_id)

    # ------------------------------------------------------------------
    # Safety + status
    # ------------------------------------------------------------------

    def _safety_check(self):
        n = len(self.agents)
        for i in range(n):
            for j in range(i + 1, n):
                a, b = self.agents[i], self.agents[j]
                distance = float(np.linalg.norm(a.state.position - b.state.position))
                self.min_separation = min(self.min_separation, distance)
                if distance < 0.7 * self.layout.cell_size:
                    self.counters["safety_violations"] += 1
                    self._event("error", f"SAFETY: {a.robot_id} and {b.robot_id} {distance:.2f} m apart", a.robot_id)

    def _update_status(self):
        now = self.world.time
        for agent in self.agents:
            meta = self.meta[agent.robot_id]
            robot = agent.robot
            if meta["error"]:
                status = "ERROR"
            elif agent.robot_id in self.yielding:
                status = "BACKING_OFF" if agent.waypoint_index < len(agent.planned_path) else "YIELDING"
            elif agent.stopped:
                status = "YIELDING"
            elif meta["dwell_until"] is not None or meta["activity"] in ("LOADING", "UNLOADING"):
                status = "DOCKED"
            elif meta["activity"] == "CHARGING":
                status = "CHARGING"
            elif meta["mode"] == "dispatch" and meta["activity"] == "IDLE":
                status = "PARKED"
            elif agent.intent == "ARRIVED":
                status = "ARRIVED"
            elif meta["blocked_by"] is not None and robot.speed < 1e-3 and not robot.turning:
                status = "WAITING"
            elif now < meta["reroute_flash"]:
                status = "REROUTING"
            elif robot.turning:
                status = "TURNING"
            elif agent.planned_path:
                status = "MOVING"
            else:
                status = "IDLE"
            meta["status"] = status
            # Keep agent intent in NEXUS vocabulary for the detector / peers.
            if status == "WAITING":
                agent.intent = "STOPPED"
            elif status in {"MOVING", "TURNING"} and agent.intent in {"STOPPED", "REROUTING"} and not agent.stopped:
                agent.intent = "MOVING"

    # ==================================================================
    # SNAPSHOT
    # ==================================================================

    def metrics(self) -> Dict[str, Any]:
        statuses = [self.meta[a.robot_id]["status"] for a in self.agents]
        moving = sum(s in {"MOVING", "TURNING", "REROUTING", "BACKING_OFF"} for s in statuses)
        distance = sum(a.robot.distance_travelled for a in self.agents)
        t = max(self.world.time, 1e-6)
        return {
            "time": round(self.world.time, 2),
            "robots": len(self.agents),
            "moving": moving,
            "waiting": sum(s in {"WAITING", "YIELDING"} for s in statuses),
            "arrived": sum(s in {"ARRIVED", "DOCKED"} for s in statuses),
            "trips_completed": self.counters["trips_completed"],
            "throughput_per_min": round(self.counters["trips_completed"] / t * 60.0, 2),
            "distance_m": round(distance, 1),
            "utilisation": round(moving / max(len(self.agents), 1), 3),
            "min_separation": None if self.min_separation == float("inf") else round(self.min_separation, 2),
            **self.counters,
        }

    def snapshot(self) -> Dict[str, Any]:
        robots = []
        for agent in self.agents:
            meta = self.meta[agent.robot_id]
            planner: RoutePlanner = agent.planner
            remaining = [agent.state.position.tolist()] + [p.tolist() for p in agent.planned_path[agent.waypoint_index:]]
            robots.append({
                "id": agent.robot_id,
                "color": meta["color"],
                "x": round(float(agent.state.position[0]), 4),
                "y": round(float(agent.state.position[1]), 4),
                "heading": round(float(agent.robot.heading), 4),
                "speed": round(float(agent.robot.speed), 3),
                "speed_limit": self.layout.speed_limit(self._cell(agent.state.position)),
                "status": meta["status"],
                "intent": agent.intent,
                "stop_reason": agent.stop_reason,
                "blocked_by": meta["blocked_by"] or (self.yielding.get(agent.robot_id) or {}).get("winner"),
                "path": [[round(v, 3) for v in p] for p in remaining],
                "reserved": [list(c) for c in self.reservations.held_by(agent.robot_id)],
                "via": [[round(v, 3) for v in p.tolist()] for p in planner.remaining_via()],
                "start": list(meta["start_cell"]),
                "goal": list(meta["goal_cell"]),
                "eta": None if agent.state.eta is None else round(float(agent.state.eta), 1),
                "trips": meta["trips"],
                "distance": round(agent.robot.distance_travelled, 1),
                "peers": len(agent.get_peer_states()),
                "error": meta["error"],
                "mode": meta["mode"],
                "activity": meta["activity"],
                "mission": meta["mission"],
                "battery": None if meta["battery"] is None else round(meta["battery"], 1),
                "home": list(meta["home_cell"]),
            })
        return {
            "time": round(self.world.time, 2),
            "status": self.status,
            "robots": robots,
            "predicted": [
                {"pair": [c.robot_a, c.robot_b],
                 "position": [round(float(v), 3) for v in c.conflict_position],
                 "ttc": round(float(c.time_to_conflict), 2)}
                for c in self.predicted
            ],
            "negotiations": [
                {"pair": list(pair), "winner": info["winner"].robot_id, "loser": info["loser"].robot_id,
                 "position": [round(float(v), 3) for v in info["position"]]}
                for pair, info in self.active_conflicts.items()
            ],
            "metrics": self.metrics(),
            "fleet": self.fleet.snapshot() if self.fleet.active else None,
        }

    def run(self, max_steps: Optional[int] = None) -> Dict[str, Any]:
        steps = max_steps if max_steps is not None else int(self.max_time / self.time_step)
        for _ in range(steps):
            self.step()
            if self.status in {"COMPLETED", "INVALID"}:
                break
        return self.snapshot()


__all__ = ["GridSimulation", "RoutePlanner"]
