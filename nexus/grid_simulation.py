"""
Grid (lane) simulation runtime for NEXUS.

Runs the existing NEXUS stack on a painted lane layout:

    RobotAgent          local state + planned path          (agents/robot_agent.py)
    P2PNetwork          decentralised state exchange        (communication/p2p.py)
    NEXUSPlanner        A*, here over a LaneGrid            (algorithms/planner.py, lane_grid.py)
    ConflictDetector    predictive trajectory conflicts     (algorithms/conflict_detector.py)
    NegotiationManager  ETA-based priority                  (algorithms/negotiation.py)

plus grid-specific coordination:

    CellReservationTable  robots reserve cells before entering them
    flow-through crossing the give-way robot is kept off the winner's cells
                          and slows to arrive just after it clears (no stop);
                          the order never closes a wait loop
    cost-aware rerouting  change lanes / go around only if quicker than waiting
    deadlock resolution   wait-for cycles -> a robot in the loop backs off to a side cell
    via-point routes      per-robot custom paths honoured through every replan

simulation.strategy = "stop_and_wait" swaps the NEXUS negotiation for the
classical baseline (fixed priority, stop at the shared stretch, no rerouting);
everything else is shared.

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
from algorithms.spacetime import SpaceTimePlanner
from communication.p2p import P2PNetwork
from nexus.alerts import AlertManager
from nexus.fleet import FleetManager
from nexus.grid_world import GridWorld
from nexus.layout import Cell, Layout

# Status -> time category (Fleet page and shift report).
TIME_CATEGORY = {
    "MOVING": "moving", "TURNING": "moving", "REROUTING": "moving", "BACKING_OFF": "moving",
    "DOCKED": "handling",
    "WAITING": "waiting", "YIELDING": "waiting",
    "CHARGING": "charging",
    "PARKED": "idle", "ARRIVED": "idle", "IDLE": "idle", "ERROR": "idle",
    "HELD": "held",
    "E_STOP": "stopped",
}
TIME_CATEGORIES = ("moving", "handling", "waiting", "charging", "idle", "held", "stopped")

AT_CENTER = 1e-3          # metres from a cell centre that count as "on" it (motion lands exactly)
PARK_REROUTE_AFTER = 2.0  # s blocked by a parked/idle robot before rerouting
YIELD_TIMEOUT = 25.0      # s a backed-off robot waits before retrying anyway
BAY_STUCK_AFTER = 4.0     # s a robot driving to its back-off bay may be blocked before it counts in deadlocks
FLOW_WINDOW = 12          # cells of a give-way robot's path checked against the other's path
FLOW_MIN_SPEED = 0.2      # m/s: a give-way robot slows to no less than this while it can still roll
FLOW_MARGIN = 0.4         # s arrival margin after the other robot has cleared the crossing
FLOW_TIMEOUT = 30.0       # s before a crossing order is dropped (the deadlock layer takes over)
ST_ENABLED = True
ST_REPLAN_EVERY = 3.0     # s between a NEXUS robot's space-time route re-checks while driving
ST_SWITCH_GAIN = 1.5      # s a new space-time route must save to be taken
ZONE_SIZE = 3             # zone-lock baseline: blocks of ZONE_SIZE x ZONE_SIZE cells
ZONE_HANDSHAKE = 1.0      # s at a standstill at a zone control point to request the zone
ST_HOLD = 60.0            # s a standing robot is assumed to keep its cell in peers' plans


class _TimedPath(list):
    """A path (list of points) that also carries its space-time plan."""
    times: List[float]


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
        self.strategy = sim.get("strategy", "nexus")

        self.grid = LaneGrid(layout, turn_penalty=float(sim["turn_penalty"]))
        self.planner = NEXUSPlanner(grid=self.grid)
        self.st_planner = SpaceTimePlanner(layout, speed=float(sim["max_speed"]),
                                           angular_speed=float(sim["angular_speed"]))
        # Broadcast timed plans: robot -> {"path": id of its path, "times": [t per path point]}
        self.st_plans: Dict[str, Dict[str, Any]] = {}
        # Zone-lock baseline: block -> robot it has been granted to.
        self.zone_grants: Dict[Any, str] = {}
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
        self.event_seq = 0
        self.counters = {
            "conflicts_predicted": 0,
            "negotiations": 0,
            "reroutes": 0,
            "backoffs": 0,
            "deadlocks_resolved": 0,
            "safety_violations": 0,
            "trips_completed": 0,
            "stops": 0,
        }
        self.min_separation = float("inf")
        self.estop = False
        self.estop_since: Optional[float] = None
        self.status = "READY"
        self.errors: List[str] = [i["message"] for i in layout.validate() if i["severity"] == "error"]

        if not self.errors:
            self._create_agents()
        self.fleet = FleetManager(self)
        self.alerts = AlertManager(self)

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
                "hold": False,
                "stall_since": None,
                "charges": 0,
                "time_in": {c: 0.0 for c in TIME_CATEGORIES},
                "stops": 0,
                "last_speed": 0.0,
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
        if self.strategy == "nexus":
            # Prioritised space-time planning: each robot plans around the
            # ones that planned before it.
            for agent in self.agents:
                meta = self.meta[agent.robot_id]
                if meta["mode"] == "fixed" and not meta["error"] and not agent.planner.remaining_via():
                    path = self._plan(agent, np.array(self.layout.cell_center(meta["goal_cell"])))
                    if path:
                        self._install_path(agent, path)

    # ==================================================================
    # HELPERS
    # ==================================================================

    @property
    def time(self) -> float:
        return float(self.world.time)

    def _event(self, kind: str, text: str, robot: Optional[str] = None, **extra):
        self.event_seq = getattr(self, "event_seq", 0) + 1
        self.events.append({"seq": self.event_seq, "t": round(self.world.time if hasattr(self, "world") else 0.0, 1),
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

    # ------------------------------------------------------------------
    # Operator controls
    # ------------------------------------------------------------------

    def set_estop(self, engaged: bool):
        """Fleet-wide emergency stop: every robot halts where it is and the
        dispatcher stops assigning until the E-stop is released."""
        engaged = bool(engaged)
        if engaged == self.estop:
            return
        self.estop = engaged
        if engaged:
            self.estop_since = self.world.time
            for agent in self.agents:
                agent.robot.speed = 0.0
            self._event("error", "E-STOP engaged: all robots stopped")
        else:
            self._event("system", f"E-stop released after {self.world.time - self.estop_since:.0f} s")
            self.estop_since = None
        self._update_status(account=False)
        self.alerts.evaluate(self.world.time, force=True)

    def set_hold(self, robot_id: str, hold: bool):
        """Operator hold of a single robot (it stops where it is)."""
        agent = self._agent(robot_id)
        if agent is None:
            raise ValueError(f"No robot {robot_id}")
        meta = self.meta[robot_id]
        if meta["hold"] == bool(hold):
            return
        meta["hold"] = bool(hold)
        if hold:
            agent.robot.speed = 0.0
        self._event("system", f"{robot_id} {'held by operator' if hold else 'released by operator'}", robot_id)
        self._update_status(account=False)
        self.alerts.evaluate(self.world.time, force=True)

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
        path = self._plan(agent, goal)
        if not path:
            return False
        self._install_path(agent, path)
        if len(path) <= 1 or agent.waypoint_index >= len(agent.planned_path):
            agent.intent = "ARRIVED"
        return True

    def _install_path(self, agent: RobotAgent, path: List[np.ndarray]):
        self.meta[agent.robot_id]["arrival_logged"] = False
        timed = getattr(path, "times", None)
        agent.set_path(path)
        if timed is not None:
            self.st_plans[agent.robot_id] = {"path": id(agent.planned_path), "times": list(timed)}
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

        if self.strategy == "nexus" and ST_ENABLED and ST_REPLAN_EVERY > 0:
            self._st_replan(now)

        # 3. Predictive conflicts -> negotiation -> reroute / stop --------
        self._release_conflicts(now)
        self.predicted = self.detector.detect_all(self.agents)
        self._negotiate(self.predicted, now)

        # 4. Cell reservations -------------------------------------------
        routes, wait_for = self._reserve(now)

        # 5. Deadlocks and long blocks (not while everything is frozen) ---
        if not self.estop:
            self._resolve_blocks(wait_for, now)
            self._resume_yielders(now)
        # Paths may have changed above: routes must match the paths the
        # world is about to drive (waypoint progress is counted per route).
        routes, _ = self._reserve(now)

        # 6. Motion (E-stop / operator hold: robots stay where they are) ---
        routes = [
            [] if self.estop or self.meta[agent.robot_id]["hold"] else route
            for agent, route in zip(self.agents, routes)
        ]
        speeds, caps = self._zone_limits(routes)
        if self.strategy == "nexus":
            speeds = self._flow_speeds(speeds)
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
        self.alerts.evaluate(self.world.time)

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
            path = self._plan(agent, goal)
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
        if self.strategy in ("stop_and_wait", "zone_lock"):
            self._release_stop_and_wait(now)
            return
        # Flow-through: the crossing order ends the moment the winner's body
        # and remaining path no longer touch the give-way robot's next cells.
        for pair, info in list(self.active_conflicts.items()):
            winner, loser = info["winner"], info["loser"]
            gone = winner.intent == "ARRIVED" or not self._remaining_cells(winner)
            if self._first_gate(loser, winner) is None or gone or now - info["since"] > FLOW_TIMEOUT:
                self._event("resume", f"{winner.robot_id} cleared -> {loser.robot_id} crosses", loser.robot_id)
                self._drop_conflict(pair)

        # Safety net: NEXUS never parks a robot with stop(); nothing stays stopped.
        for agent in self.agents:
            if agent.stopped and agent.robot_id not in self.yielding:
                agent.resume()

    def _drop_conflict(self, pair):
        info = self.active_conflicts.pop(pair, None)
        if info is not None and info["loser"].stopped and info["loser"].robot_id not in self.yielding:
            info["loser"].resume()

    def _negotiate(self, conflicts: List[PredictedConflict], now: float):
        if self.strategy == "stop_and_wait":
            self._negotiate_stop_and_wait(conflicts, now)
            return
        if self.strategy == "zone_lock":
            return  # block control only: no negotiation at all
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

            win_d, lose_d = self.negotiation.negotiate(a, b, conflict)
            winner, loser = self._agent(win_d.robot_id), self._agent(lose_d.robot_id)
            reason = win_d.reason
            # A robot already standing on the other's path has to clear it
            # first, whatever the ETAs say.
            if set(self._occupied(loser)) & set(self._remaining_cells(winner)) \
                    and not set(self._occupied(winner)) & set(self._remaining_cells(loser)):
                winner, loser = loser, winner
                reason = f"{winner.robot_id} is already on {loser.robot_id}'s path"
            ordered = self._acyclic_order(winner, loser)
            if ordered[0] is None:
                continue
            if ordered[0] is not winner:
                reason = f"{loser.robot_id} giving way would close a wait loop"
            winner, loser = ordered
            gate = self._first_gate(loser, winner)
            if gate is None:
                continue  # the paths do not actually share a cell nearby

            # Cost-aware: change lanes when the detour is quicker than waiting
            # for the winner to clear (head-on on a two-way stretch, a long
            # shared stretch); a plain crossing is cheaper to flow through.
            wait = self._expected_wait(loser, winner, gate)
            avoid = set(self._remaining_cells(winner)[: 2 * FLOW_WINDOW]) | set(self._occupied(winner))
            if self._detour(loser, avoid, wait, now, f"changes lane to pass {winner.robot_id}"):
                self.counters["conflicts_predicted"] += 1
                self.counters["negotiations"] += 1
                continue

            self.counters["conflicts_predicted"] += 1
            self.counters["negotiations"] += 1
            self.active_conflicts[pair] = {
                "winner": winner,
                "loser": loser,
                "position": np.asarray(conflict.conflict_position, dtype=float).copy(),
                "gate": gate[1],
                "since": now,
            }
            self._event(
                "conflict",
                f"Conflict {a.robot_id}↔{b.robot_id} in {conflict.time_to_conflict:.1f}s: "
                f"{winner.robot_id} crosses first, {loser.robot_id} slows to cross behind it ({reason})",
                winner.robot_id,
                position=[float(v) for v in conflict.conflict_position],
                pair=[a.robot_id, b.robot_id],
            )

    # ------------------------------------------------------------------
    # Space-time planning (NEXUS)
    # ------------------------------------------------------------------

    def _heading_index(self, agent: RobotAgent) -> int:
        h = agent.robot.heading
        return int(round(h / (np.pi / 2.0))) % 4

    def _timelines(self, now: float, exclude: str) -> Dict[Cell, List[Tuple[float, float]]]:
        """What every other robot has broadcast: per cell, when it will be
        there. Moving robots: their timed plan (shifted by how late they
        are) or their path at nominal speed; standing robots hold their cell."""
        c = self.st_planner.clearance
        table: Dict[Cell, List[Tuple[float, float]]] = {}
        speed = self.world.max_speed
        for other in self.agents:
            if other.robot_id == exclude:
                continue
            body = self._occupied(other)
            upcoming = self._upcoming(other)
            if not upcoming or other.robot.speed < 1e-3 and self.meta[other.robot_id]["status"] in (
                    "ARRIVED", "DOCKED", "PARKED", "IDLE", "CHARGING", "HELD", "ERROR"):
                for cell in body:
                    table.setdefault(cell, []).append((now - c, now + ST_HOLD))
                continue
            for cell in body:
                table.setdefault(cell, []).append((now - c, now + c))
            plan = self.st_plans.get(other.robot_id)
            times = None
            if plan and plan["path"] == id(other.planned_path) and len(plan["times"]) == len(other.planned_path):
                k = upcoming[0][0]
                eta_k = now + self._distance_to(other, k) / speed
                shift = eta_k - plan["times"][k]
                times = [plan["times"][j] + shift for j, _ in upcoming]
            if times is None:
                times, t, prev = [], now, other.state.position
                for j, cell in upcoming:
                    t += float(np.linalg.norm(other.planned_path[j] - prev)) / speed
                    prev = other.planned_path[j]
                    times.append(t)
            for (j, cell), t in zip(upcoming, times):
                table.setdefault(cell, []).append((t - c, t + c))
            # Where it ends up (a lane cell, not a station): held afterwards.
            last = upcoming[-1][1]
            if not self.layout.is_station(last):
                table.setdefault(last, []).append((times[-1], times[-1] + ST_HOLD))
        from algorithms.spacetime import merge
        return {cell: merge(iv) for cell, iv in table.items()}

    def _plan(self, agent: RobotAgent, goal: np.ndarray):
        """Route to goal. NEXUS plans in space-time around its peers'
        broadcast plans; the baselines (and via-point routes) use plain A*."""
        if self.strategy != "nexus" or agent.planner.remaining_via() or not ST_ENABLED:
            return agent.planner.plan(agent.state.position, goal)
        now = self.world.time
        start, target = self._cell(agent.state.position), self._cell(goal)
        reserved = self._timelines(now, agent.robot_id)
        # Our own body is never an obstacle to ourselves.
        timed = self.st_planner.plan(start, target, now, reserved, self._heading_index(agent))
        if not timed:
            return agent.planner.plan(agent.state.position, goal)
        path = _TimedPath(np.array(self.layout.cell_center(cell)) for cell, _ in timed)
        path.times = [t for _, t in timed]
        return path

    def _st_replan(self, now: float):
        """Every ST_REPLAN_EVERY s a driving robot re-checks its route
        against the latest broadcast plans and switches if a route that
        avoids the traffic ahead arrives clearly earlier."""
        for agent in self.agents:
            meta = self.meta[agent.robot_id]
            if agent.robot_id in self.yielding or agent.stopped or meta["hold"] or self.estop:
                continue
            if meta["status"] not in ("MOVING", "TURNING", "WAITING", "REROUTING") or agent.state.goal is None:
                continue
            if agent.planner.remaining_via() or not self._at_center(agent):
                continue
            if now - meta.get("st_checked", -1e9) < ST_REPLAN_EVERY:
                continue
            meta["st_checked"] = now
            upcoming = self._upcoming(agent)
            if len(upcoming) < 3:
                continue
            start = self._cell(agent.state.position)
            reserved = self._timelines(now, agent.robot_id)
            current = [start] + [c for _, c in upcoming if c != start]
            arrive_now = self.st_planner.time_path(current, now, reserved, self._heading_index(agent))[-1][1]
            timed = self.st_planner.plan(start, current[-1], now, reserved, self._heading_index(agent))
            if not timed or timed[-1][1] > arrive_now - ST_SWITCH_GAIN:
                continue
            if [c for c, _ in timed] == current:
                continue
            path = _TimedPath(np.array(self.layout.cell_center(cell)) for cell, _ in timed)
            path.times = [t for _, t in timed]
            self._install_path(agent, path)
            agent.intent = "REROUTING"
            self.counters["reroutes"] += 1
            meta["reroute_flash"] = now + 1.5
            self._event("reroute", f"{agent.robot_id} takes a less busy route "
                        f"(arrives {arrive_now - timed[-1][1]:.1f}s sooner)", agent.robot_id)

    # ------------------------------------------------------------------
    # Flow-through crossing (NEXUS)
    # ------------------------------------------------------------------

    def _first_gate(self, loser: RobotAgent, winner: RobotAgent) -> Optional[Tuple[int, Cell]]:
        """(path index, cell) of the first cell in the give-way robot's next
        FLOW_WINDOW cells that the winner still occupies or has to drive
        through; None once the way is clear."""
        blocked = set(self._remaining_cells(winner)) | set(self._occupied(winner))
        body = set(self._occupied(loser))
        for j, cell in self._upcoming(loser)[:FLOW_WINDOW]:
            if cell in body:
                continue
            if cell in blocked:
                return j, cell
        return None

    def _waits_for_graph(self) -> Dict[str, set]:
        """Who waits (or will wait) for whom: crossing orders, current
        reservation blocks, and robots queued behind another on its path."""
        graph: Dict[str, set] = {a.robot_id: set() for a in self.agents}
        for info in self.active_conflicts.values():
            graph[info["loser"].robot_id].add(info["winner"].robot_id)
        for agent in self.agents:
            blocker = self.meta[agent.robot_id]["blocked_by"]
            if blocker:
                graph[agent.robot_id].add(blocker)
        # Queued behind: a robot standing, or driving the same way, on the path ahead
        # (one driving towards it is the encounter being negotiated, not a queue).
        for agent in self.agents:
            ahead = set(self._remaining_cells(agent)[:FLOW_WINDOW])
            for other in self.agents:
                if other is agent or not set(self._occupied(other)) & ahead:
                    continue
                if other.robot.speed < 1e-3 or float(np.cos(other.robot.heading - agent.robot.heading)) > 0.7:
                    graph[agent.robot_id].add(other.robot_id)
        return graph

    def _acyclic_order(self, winner: RobotAgent, loser: RobotAgent):
        """(winner, loser), flipped if loser -> winner would close a wait
        loop; (None, None) if both orders would."""
        graph = self._waits_for_graph()

        def reaches(src: str, dst: str) -> bool:
            seen, stack = {src}, [src]
            while stack:
                node = stack.pop()
                if node == dst:
                    return True
                for nxt in graph.get(node, ()):
                    if nxt not in seen:
                        seen.add(nxt)
                        stack.append(nxt)
            return False

        if not reaches(winner.robot_id, loser.robot_id):
            return winner, loser
        if not reaches(loser.robot_id, winner.robot_id):
            return loser, winner
        return None, None

    def _blocker_wait(self, blocker: RobotAgent, now: float) -> float:
        """Rough seconds until a stationary blocker is out of the way."""
        meta = self.meta[blocker.robot_id]
        if meta["hold"]:
            return 60.0
        if meta.get("busy_until") is not None and meta["activity"] in ("LOADING", "UNLOADING"):
            return max(0.0, meta["busy_until"] - now) + 3.0
        if meta["dwell_until"] is not None:
            return max(0.0, meta["dwell_until"] - now) + 3.0
        if self._parked(blocker):
            return 4.0  # it will be asked to make way
        return 6.0

    def _expected_wait(self, loser: RobotAgent, winner: RobotAgent, gate: Tuple[int, Cell]) -> float:
        """Seconds the give-way robot would lose waiting at the gate."""
        seconds = self._clear_time(winner, gate[1])
        if seconds is None:
            return 8.0  # the winner is itself blocked or parks there: unknown, assume long
        reach = max(0.0, self._distance_to(loser, gate[0]) - self.layout.cell_size) / max(loser.max_speed, 1e-6)
        return max(0.0, seconds - reach)

    def _path_time(self, start: np.ndarray, points: List[np.ndarray], speed: float) -> float:
        """Driving time along points: distance at speed plus quarter turns."""
        seconds, previous, heading = 0.0, start, None
        for point in points:
            d = point - previous
            length = float(np.linalg.norm(d))
            if length < 1e-9:
                continue
            h = float(np.arctan2(d[1], d[0]))
            if heading is not None:
                seconds += abs((h - heading + np.pi) % (2 * np.pi) - np.pi) / self.world.angular_speed
            seconds += length / max(speed, 1e-6)
            heading, previous = h, point
        return seconds

    def _detour(self, agent: RobotAgent, avoid, wait: float, now: float, what: str) -> bool:
        """Replan around avoid (cells) and take the new path only if it is
        quicker than the current one plus the expected wait."""
        if self.strategy != "nexus" or agent.state.goal is None or wait < 1.0:
            return False
        keep = set(self._occupied(agent)) | {self.meta[agent.robot_id]["goal_cell"]}
        obstacles = [(np.array(self.layout.cell_center(c)), 0.5 * self.layout.cell_size) for c in avoid if c not in keep]
        if not obstacles:
            return False
        path = agent.planner.plan(agent.state.position, agent.state.goal, dynamic_obstacles=obstacles)
        if not path:
            return False
        current = self._path_time(agent.state.position, agent.planned_path[agent.waypoint_index:], agent.max_speed)
        detour = self._path_time(agent.state.position, path, agent.max_speed)
        if detour + 0.5 >= current + wait:
            return False
        agent.set_path(path)
        agent.advance_waypoint()
        agent.intent = "REROUTING"
        self.counters["reroutes"] += 1
        self.meta[agent.robot_id]["reroute_flash"] = now + 1.5
        self._event("reroute", f"{agent.robot_id} {what} (+{max(0.0, detour - current):.1f}s detour vs {wait:.1f}s wait)",
                    agent.robot_id)
        return True

    def _gates(self) -> Dict[str, Dict[Cell, str]]:
        """Per give-way robot: cells it may not reserve yet -> the robot it gives way to."""
        gates: Dict[str, Dict[Cell, str]] = {}
        for info in self.active_conflicts.values():
            winner, loser = info["winner"], info["loser"]
            cells = gates.setdefault(loser.robot_id, {})
            still = set(self._remaining_cells(winner)) | set(self._occupied(winner))
            if self.strategy == "stop_and_wait":
                # The whole shared stretch stays closed until all of it is clear.
                blocked = info["shared"] if info["shared"] & still else set()
            else:
                # Flow-through: only the cells the winner still has to pass.
                blocked = still
            for cell in blocked:
                cells.setdefault(cell, winner.robot_id)
        return gates

    def _distance_to(self, agent: RobotAgent, index: int) -> float:
        """Distance along the agent's path from its position to path point index."""
        total, previous = 0.0, agent.state.position
        for j in range(agent.waypoint_index, min(index, len(agent.planned_path) - 1) + 1):
            total += float(np.linalg.norm(agent.planned_path[j] - previous))
            previous = agent.planned_path[j]
        return total

    def _clear_time(self, winner: RobotAgent, cell: Cell) -> Optional[float]:
        """Seconds until the winner's body has left cell (None: unknown, e.g.
        it is blocked itself or parks there)."""
        meta = self.meta[winner.robot_id]
        if meta["blocked_by"] is not None or winner.stopped or winner.robot_id in self.yielding:
            return None
        upcoming = self._upcoming(winner)
        hit = next((k for k, (_, c) in enumerate(upcoming) if c == cell), None)
        if hit is None:
            return 0.0 if cell not in self._occupied(winner) else None
        if hit + 1 >= len(upcoming):
            return None  # it stops in that cell
        exit_index = upcoming[hit + 1][0]  # its centre on the next cell: body clear
        distance = self._distance_to(winner, exit_index)
        vmax = max(winner.max_speed, 1e-6)
        seconds = distance / vmax + max(0.0, vmax - winner.robot.speed) / (2.0 * self.world.acceleration)
        # Turns on the way (rotate in place).
        points = [winner.state.position] + [winner.planned_path[j] for j, _ in upcoming[: hit + 2]]
        headings = [np.arctan2(*(b - a)[::-1]) for a, b in zip(points, points[1:]) if np.linalg.norm(b - a) > 1e-9]
        for h0, h1 in zip(headings, headings[1:]):
            seconds += abs((h1 - h0 + np.pi) % (2 * np.pi) - np.pi) / self.world.angular_speed
        return seconds

    def _flow_speeds(self, speeds: List[float]) -> List[float]:
        """Give-way robots slow down so they reach the crossing just as the
        other robot has cleared it, instead of braking to a stop there."""
        cs = self.layout.cell_size
        out = list(speeds)
        for info in self.active_conflicts.values():
            loser, winner = info["loser"], info["winner"]
            i = self.meta[loser.robot_id]["index"]
            gate = self._first_gate(loser, winner)
            if gate is None:
                continue
            seconds = self._clear_time(winner, gate[1])
            if seconds is None:
                continue  # unknown: drive up to the gate and wait there
            # Its body touches the gate cell one cell before the gate's centre.
            distance = self._distance_to(loser, gate[0]) - cs
            if distance <= 0:
                continue
            target = distance / (seconds + FLOW_MARGIN)
            if target >= out[i]:
                continue
            # Brake at most at the normal deceleration.
            floor = loser.robot.speed - self.world.acceleration * self.time_step
            out[i] = min(out[i], max(target, FLOW_MIN_SPEED, floor))
        return out

    # ------------------------------------------------------------------
    # Classical stop-and-wait (benchmark baseline)
    # ------------------------------------------------------------------

    def _negotiate_stop_and_wait(self, conflicts: List[PredictedConflict], now: float):
        """Traditional traffic rule for overlapping paths: fixed priority by
        robot ID; the lower-priority robot drives up to the shared stretch,
        stops at its entry and waits until the other has passed every cell
        their remaining paths share. No rerouting, no speed adaptation."""
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
            winner, loser = (a, b) if a.robot_id < b.robot_id else (b, a)
            shared = set(self._remaining_cells(winner)) & (set(self._remaining_cells(loser)) | set(self._occupied(loser)))
            self.counters["conflicts_predicted"] += 1
            self.counters["negotiations"] += 1
            self.active_conflicts[pair] = {
                "winner": winner, "loser": loser, "since": now, "shared": shared,
                "position": np.asarray(conflict.conflict_position, dtype=float).copy(),
            }
            self._event("conflict", f"Conflict {a.robot_id}↔{b.robot_id}: {loser.robot_id} stops at the shared path and waits for "
                        f"{winner.robot_id} to clear {len(shared)} shared cell(s) (fixed priority)",
                        winner.robot_id, position=[float(v) for v in conflict.conflict_position],
                        pair=[a.robot_id, b.robot_id])

    def _release_stop_and_wait(self, now: float):
        for pair, info in list(self.active_conflicts.items()):
            winner, loser = info["winner"], info["loser"]
            still = set(self._remaining_cells(winner)) | set(self._occupied(winner))
            gone = winner.intent == "ARRIVED" or not self._remaining_cells(winner)
            if not (info["shared"] & still) or gone or now - info["since"] > 60.0:
                self._event("resume", f"{winner.robot_id} cleared the shared path -> {loser.robot_id} resumes", loser.robot_id)
                self._drop_conflict(pair)
        losers = {info["loser"].robot_id for info in self.active_conflicts.values()}
        for agent in self.agents:
            if agent.stopped and agent.robot_id not in losers and agent.robot_id not in self.yielding:
                agent.resume()

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

        gates = self._gates()
        zones = self._zone_holders() if self.strategy == "zone_lock" else None
        if zones is not None:
            # Classical block control: fixed priority by robot number.
            order = sorted(range(len(self.agents)), key=lambda i: self.reservations.priority_key(0.0, self.agents[i].robot_id))
        else:
            order = sorted(range(len(self.agents)), key=lambda i: self._priority(self.agents[i]))
        for i in order:
            agent = self.agents[i]
            meta = self.meta[agent.robot_id]
            upcoming = self._upcoming(agent)
            body = set(occupied[agent.robot_id])
            chain: List[int] = []
            blocked_by = None
            gated = gates.get(agent.robot_id, {})
            meta["zone_stop"] = False

            for j, cell in upcoming:
                if cell in body:
                    chain.append(j)
                    continue
                if agent.stopped or meta["dwell_until"] is not None:
                    break
                if len(chain) >= self.lookahead:
                    break
                if cell in gated:
                    # Crossing order: wait for the winner to clear this cell.
                    blocked_by = gated[cell]
                    break
                if zones is not None and agent.robot_id not in self.yielding \
                        and self._zone(cell) is not None and zones.get(self._zone(cell)) != agent.robot_id:
                    # Entering a block system: claim every block up to the
                    # next exit at once, or stop dead at the boundary.
                    # Zone control point: come to a standstill at the boundary
                    # and request the zone (ZONE_HANDSHAKE s), free or not.
                    at_boundary = all(self._cell(agent.planned_path[k]) in body for k in chain) \
                        and agent.robot.speed < 1e-3 and self._at_center(agent)
                    if not at_boundary:
                        break
                    meta["zone_stop"] = True
                    run = []
                    for _, c in upcoming[upcoming.index((j, cell)):]:
                        zone = self._zone(c)
                        if zone is None:
                            break
                        if zone not in run:
                            run.append(zone)
                    holder = next((zones[z] for z in run if zones.get(z) not in (None, agent.robot_id)), None)
                    if holder is not None:
                        meta["zone_request"] = None
                        blocked_by = holder
                        break
                    if meta.get("zone_request") is None:
                        meta["zone_request"] = now
                    if now - meta["zone_request"] < ZONE_HANDSHAKE - 1e-9:
                        break
                    meta["zone_request"] = None
                    meta["zone_stop"] = False
                    for zone in run:
                        zones[zone] = agent.robot_id
                        self.zone_grants[zone] = agent.robot_id
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
        # Only real blocking edges (robot has somewhere to go and is stuck). A
        # robot still driving to its back-off bay counts: its bay can be
        # taken, and the cycle must be broken again.
        blocking = {r: s for r, s in wait_for.items() if r not in self.yielding or self._to_bay(r, now)}
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
                    or self.meta[blocker_id]["hold"]
                    or self.meta[blocker_id]["blocked_since"] is not None):
                continue

            # NEXUS reroutes when the detour beats the expected wait; the
            # classical baseline never reroutes (it waits, or the parked
            # blocker makes way).
            if self._detour(agent, set(self._occupied(blocker)), self._blocker_wait(blocker, now), now,
                            f"reroutes around {blocker_id}"):
                meta["blocked_since"] = now
            elif self._parked(blocker) and self._back_off(blocker, agent, now):
                self._event("backoff", f"{blocker_id} makes way for {robot_id}", blocker_id)
            else:
                meta["blocked_since"] = now  # try again later

    def _zone(self, cell: Cell):
        """Zone-lock baseline: the ZONE_SIZE x ZONE_SIZE block a two-way /
        junction cell belongs to. One-way lanes and stations are outside the
        block system (robots queue there on cell reservations)."""
        if self.layout.is_station(cell) or self.layout.direction(cell) is not None or not self.layout.is_drivable(cell):
            return None
        return (cell[0] // ZONE_SIZE, cell[1] // ZONE_SIZE)

    def _zone_holders(self) -> Dict[Any, str]:
        """Blocks held now: granted runs (until the robot has passed them)
        and the blocks robots stand in. One robot per block. A robot backing
        off out of a deadlock is outside the block system."""
        for zone, robot_id in list(self.zone_grants.items()):
            agent = self._agent(robot_id)
            # Released once the robot is out of the block and not about to
            # use it again (a route that comes back later claims it again).
            still = {self._zone(c) for c in list(self._occupied(agent)) + self._remaining_cells(agent)[:2 * ZONE_SIZE + 2]}
            if robot_id in self.yielding or zone not in still:
                del self.zone_grants[zone]
        holders: Dict[Any, str] = dict(self.zone_grants)
        for agent in self.agents:
            if agent.robot_id in self.yielding:
                continue
            for cell in self._occupied(agent):
                zone = self._zone(cell)
                if zone is not None:
                    holders.setdefault(zone, agent.robot_id)
        return holders

    def _to_bay(self, robot_id: str, now: float) -> bool:
        """Backing off, but stuck on the way to its bay for a while."""
        agent = self._agent(robot_id)
        since = self.meta[robot_id]["blocked_since"]
        return agent.waypoint_index < len(agent.planned_path) and since is not None and now - since >= BAY_STUCK_AFTER

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

        candidates = [(loser, winner), (winner, loser)]
        # Nobody in the pair can move aside: any robot in the loop may make
        # way for the robot it waits for.
        for k, robot_id in enumerate(cycle):
            member, ahead = self._agent(robot_id), self._agent(cycle[(k + 1) % len(cycle)])
            if member is not None and ahead is not None and member is not ahead \
                    and (member, ahead) not in candidates:
                candidates.append((member, ahead))
        for yielder, other in candidates:
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
        if self.meta[yielder.robot_id]["hold"]:
            return False  # an operator hold is never overridden
        start = self._cell(yielder.state.position)
        avoid = set(self._remaining_cells(winner)) | set(self._occupied(winner))
        bodies = {c for a in self.agents if a is not yielder for c in self._occupied(a)}

        def free(cell) -> bool:
            # Robot bodies never; a cell only reserved ahead by a robot that
            # is itself standing still can be taken (it is re-reserved later).
            holder = self.reservations.holder(cell)
            if cell in bodies:
                return False
            if holder in (None, yielder.robot_id):
                return True
            other = self._agent(holder)
            return other is not None and other.robot.speed < 1e-3 and self.meta[holder]["blocked_by"] is not None
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
                if not free(n):
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
        for cell in cells:
            holder = self.reservations.holder(cell)
            if holder not in (None, yielder.robot_id):
                self.reservations.retain_only(holder, set(self.reservations.held_by(holder)) - {cell})
            self.reservations.reserve(yielder.robot_id, cell)
        path = [np.array(self.layout.cell_center(c)) for c in cells]
        if yielder.stopped:
            yielder.resume()
        yielder.set_path(path)
        yielder.advance_waypoint()
        yielder.intent = "REROUTING"
        self.yielding[yielder.robot_id] = {"winner": winner.robot_id, "bay": target, "since": now}
        self.meta[yielder.robot_id]["dwell_until"] = None
        self.meta[yielder.robot_id]["blocked_since"] = now  # grace before it can count as stuck again
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
            path = self._plan(agent, goal)
            if not path:
                continue
            winner_cells = set(self._remaining_cells(winner)) | set(self._occupied(winner))
            ahead = {self._cell(p) for p in path[: 2 * self.lookahead + 2]}
            if self.strategy == "zone_lock":
                # Blocks, not cells: wait until the winner needs none of ours.
                winner_cells = {self._zone(c) for c in winner_cells} - {None}
                ahead = {self._zone(c) for c in ahead} - {None}
            clear = not (ahead & winner_cells) or winner.intent == "ARRIVED" and not ({self._cell(p) for p in path[:3]} & set(self._occupied(winner)))
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

    def _update_status(self, account: bool = True):
        """Derive each robot's status; account=True also books this step's
        time to the status's category (False for out-of-step refreshes)."""
        now = self.world.time
        for agent in self.agents:
            meta = self.meta[agent.robot_id]
            robot = agent.robot
            if meta["error"]:
                status = "ERROR"
            elif self.estop:
                status = "E_STOP"
            elif meta["hold"]:
                status = "HELD"
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
            elif (meta["blocked_by"] is not None or meta.get("zone_stop")) and robot.speed < 1e-3 and not robot.turning:
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
            if account:
                meta["time_in"][TIME_CATEGORY.get(status, "idle")] += self.time_step
                # A full stop: braking to a standstill because of traffic
                # (not at a station, a turn, a hold or an E-stop).
                if robot.speed < 1e-3 and meta["last_speed"] > 0.05:
                    meta["halted_at"] = now
                halted = meta.get("halted_at")
                if halted is not None and status in ("WAITING", "YIELDING") and now - halted <= 0.3 + 1e-9:
                    meta["stops"] += 1
                    self.counters["stops"] += 1
                    meta["halted_at"] = None
                elif halted is not None and now - halted > 0.3 + 1e-9:
                    meta["halted_at"] = None
                meta["last_speed"] = robot.speed
            if status in ("WAITING", "YIELDING"):
                if meta["stall_since"] is None:
                    meta["stall_since"] = now
            else:
                meta["stall_since"] = None
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
            "held": sum(1 for a in self.agents if self.meta[a.robot_id]["hold"]),
            "strategy": self.strategy,
            "estop": self.estop,
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
                "hold": meta["hold"],
                "charges": meta["charges"],
                "stops": meta["stops"],
                "time_in": {k: round(v, 1) for k, v in meta["time_in"].items()},
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
            "estop": self.estop,
            "estop_since": self.estop_since,
            "alerts": self.alerts.snapshot(),
        }

    def run(self, max_steps: Optional[int] = None) -> Dict[str, Any]:
        steps = max_steps if max_steps is not None else int(self.max_time / self.time_step)
        for _ in range(steps):
            self.step()
            if self.status in {"COMPLETED", "INVALID"}:
                break
        return self.snapshot()


__all__ = ["GridSimulation", "RoutePlanner"]
