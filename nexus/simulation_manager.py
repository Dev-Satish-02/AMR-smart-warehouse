import json
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import numpy as np
import yaml

import irsim

from algorithms.conflict_detector import ConflictDetector
from algorithms.negotiation import NegotiationManager
from algorithms.planner import NEXUSPlanner
from agents.robot_agent import RobotAgent
from communication.p2p import P2PNetwork


class SimulationManager:
    """
    Reusable simulation shell around the existing NEXUS engine.

    The dashboard uses this layer to keep the original conflict logic intact while
    exposing a simpler config-driven runtime and per-robot event stream.
    """

    def __init__(
        self,
        config: Optional[Dict[str, Any]] = None,
        warehouse_path: str = "configs/warehouse.yaml",
    ):
        self.warehouse_path = warehouse_path
        self.config = self._normalize_config(config)
        self.env = None
        self.planner = None
        self.detector = None
        self.negotiation = None
        self.network = None
        self.agents: List[RobotAgent] = []
        self.active_conflicts: Dict[tuple, Dict[str, Any]] = {}
        self.simulation_time = 0.0
        self.status = "IDLE"
        self.conflict_count = 0
        self.reroute_count = 0
        self.stop_count = 0
        self.safety_violations = 0
        self.event_log: List[str] = []
        self.robot_event_log: Dict[str, List[str]] = {}
        self._build()

    @staticmethod
    def create_default_config() -> Dict[str, Any]:
        return {
            "warehouse": {
                "name": "Warehouse A",
                "width": 30.0,
                "height": 20.0,
                "resolution": 0.5,
                "path": "configs/demo_warehouse.yaml",
                "obstacles": [],
                "stations": [],
                "charging_stations": [],
            },
            "robots": [
                {
                    "id": "R1",
                    "start": [2.0, 10.0],
                    "goal": [28.0, 10.0],
                    "heading": 0.0,
                    "max_speed": 0.8,
                    "battery": 100.0,
                    "task_id": "T1",
                },
                {
                    "id": "R4",
                    "start": [28.0, 10.0],
                    "goal": [2.0, 10.0],
                    "heading": 3.14159,
                    "max_speed": 0.8,
                    "battery": 100.0,
                    "task_id": "T4",
                },
            ],
            "tasks": [
                {"id": "T1", "pickup": [2.0, 10.0], "dropoff": [28.0, 10.0], "priority": 1, "assigned_robot": "R1", "status": "PENDING"},
                {"id": "T4", "pickup": [28.0, 10.0], "dropoff": [2.0, 10.0], "priority": 1, "assigned_robot": "R4", "status": "PENDING"},
            ],
            "simulation": {
                "time_step": 0.1,
                "max_steps": 900,
                "clearance_distance": 1.5,
                "dynamic_obstacle_radius": 1.0,
                "hard_safety_distance": 0.70,
                "strategy": "nexus",
            },
        }

    def _normalize_config(self, config: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        if config is None:
            config = self.create_default_config()

        normalized = deepcopy(config)
        normalized.setdefault("warehouse", {})
        normalized["warehouse"].setdefault("path", self.warehouse_path)
        normalized.setdefault("robots", [])
        normalized.setdefault("tasks", [])
        normalized.setdefault("simulation", {})
        normalized["simulation"].setdefault("time_step", 0.1)
        normalized["simulation"].setdefault("max_steps", 900)
        return normalized

    def load_config(self, path: str) -> Dict[str, Any]:
        file_path = Path(path)

        if file_path.suffix.lower() in {".json"}:
            with file_path.open("r", encoding="utf-8") as handle:
                self.config = json.load(handle)
        else:
            with file_path.open("r", encoding="utf-8") as handle:
                self.config = yaml.safe_load(handle) or {}

        self.config = self._normalize_config(self.config)
        self.warehouse_path = self.config["warehouse"].get("path", self.warehouse_path)
        self.reset()
        return self.config

    def save_config(self, path: str) -> str:
        file_path = Path(path)
        file_path.parent.mkdir(parents=True, exist_ok=True)

        if file_path.suffix.lower() in {".json"}:
            with file_path.open("w", encoding="utf-8") as handle:
                json.dump(self.config, handle, indent=2)
        else:
            with file_path.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(self.config, handle, sort_keys=False)

        return str(file_path)

    def _record_event(self, message: str):
        timestamp = f"[T={self.simulation_time:.1f}s]"
        event = f"{timestamp} {message}"
        self.event_log.append(event)
        if len(self.event_log) > 200:
            self.event_log = self.event_log[-200:]

    def _record_robot_event(self, robot_id: str, message: str):
        records = self.robot_event_log.setdefault(robot_id, [])
        records.append(f"[T={self.simulation_time:.1f}s] {message}")
        if len(records) > 25:
            records[:] = records[-25:]

    def _build(self):
        self.planner = NEXUSPlanner()
        self.detector = ConflictDetector(
            prediction_horizon=3.0,
            prediction_dt=0.2,
            safety_distance=1.40,
        )
        self.negotiation = NegotiationManager(eta_margin=0.25)
        self.network = P2PNetwork()

        self._create_environment()
        self._create_agents()
        self._sync_task_statuses()
        self.status = "READY"
        if not self.event_log:
            self._record_event("SIMULATION READY")

    def _create_environment(self):
        warehouse_path = self.config["warehouse"].get("path", self.warehouse_path)
        self.env = irsim.make(warehouse_path)
        self.warehouse_path = warehouse_path

    def _create_agents(self):
        if self.env is None:
            raise RuntimeError("Simulation environment not initialized.")

        robot_configs = self.config.get("robots", [])
        self.agents = []

        for index, robot_config in enumerate(robot_configs):
            if index >= len(self.env.robot_list):
                break

            robot_id = str(robot_config.get("id", f"R{index + 1}"))
            robot = self.env.robot_list[index]
            agent = RobotAgent(robot_id=robot_id, robot=robot, planner=self.planner)

            start = np.asarray(robot_config.get("start", robot_config.get("position", [0.0, 0.0])), dtype=float)
            goal = np.asarray(robot_config.get("goal", robot_config.get("target", [0.0, 0.0])), dtype=float)
            agent.start_position = np.asarray(start, dtype=float)

            task_id = robot_config.get("task_id")
            if task_id is None:
                task_id = f"T-{robot_id}"

            agent.set_task(task_id=task_id, goal=goal)
            agent.plan_path(start=start, goal=goal)

            if "heading" in robot_config:
                try:
                    agent.state.heading = float(robot_config["heading"])
                except (TypeError, ValueError):
                    pass

            if "max_speed" in robot_config:
                try:
                    agent.max_speed = float(robot_config["max_speed"])
                except (TypeError, ValueError):
                    pass

            self.agents.append(agent)
            self.network.register(agent)

    def _sync_task_statuses(self):
        for task in self.config.get("tasks", []):
            task_id = str(task.get("id", ""))
            if not task_id:
                continue

            assigned_robot = task.get("assigned_robot")
            agent = next((candidate for candidate in self.agents if candidate.robot_id == assigned_robot), None)

            if agent is None:
                task["status"] = "PENDING"
                continue

            if agent.intent == "ARRIVED":
                task["status"] = "COMPLETED"
                continue

            if task.get("dropoff") is not None:
                dropoff = np.asarray(task["dropoff"], dtype=float)
            elif agent.state.goal is not None:
                dropoff = np.asarray(agent.state.goal, dtype=float)
            else:
                dropoff = np.asarray([0.0, 0.0], dtype=float)

            if agent.state.goal is not None and np.linalg.norm(agent.state.position - dropoff) < 0.6:
                task["status"] = "COMPLETED"
            else:
                task["status"] = "PENDING"

    def _release_active_conflicts(self):
        for pair in list(self.active_conflicts.keys()):
            info = self.active_conflicts[pair]
            winner = info["winner"]
            loser = info["loser"]
            conflict_position = info["position"]

            distance_from_conflict = np.linalg.norm(winner.state.position - conflict_position)
            clearance_distance = float(self.config.get("simulation", {}).get("clearance_distance", 1.5))

            if distance_from_conflict >= clearance_distance and not winner.stopped:
                loser.resume()
                self._record_event(f"{winner.robot_id} cleared conflict -> {loser.robot_id} RESUMES")
                self._record_robot_event(winner.robot_id, f"cleared conflict -> {loser.robot_id} RESUMES")
                self._record_robot_event(loser.robot_id, f"resumed after {winner.robot_id} cleared conflict")
                del self.active_conflicts[pair]

    def _resolve_conflicts(self, conflicts: Iterable[Any]):
        for conflict in conflicts:
            pair = tuple(sorted([conflict.robot_a, conflict.robot_b]))

            if pair in self.active_conflicts:
                continue

            agent_a = next((agent for agent in self.agents if agent.robot_id == conflict.robot_a), None)
            agent_b = next((agent for agent in self.agents if agent.robot_id == conflict.robot_b), None)

            if agent_a is None or agent_b is None:
                continue

            if agent_a.stopped or agent_b.stopped:
                continue

            winner_decision, loser_decision = self.negotiation.negotiate(agent_a, agent_b, conflict)
            winner_agent = next((agent for agent in self.agents if agent.robot_id == winner_decision.robot_id), None)
            loser_agent = next((agent for agent in self.agents if agent.robot_id == loser_decision.robot_id), None)

            if winner_agent is None or loser_agent is None:
                continue

            loser_agent.stop(reason=f"WAIT_FOR_{winner_agent.robot_id}")

            rerouted = winner_agent.replan_around(
                obstacle_position=loser_agent.state.position,
                obstacle_radius=float(self.config.get("simulation", {}).get("dynamic_obstacle_radius", 1.0)),
            )

            if not rerouted:
                winner_agent.stop(reason="NO_SAFE_REROUTE")
                continue

            self.active_conflicts[pair] = {
                "winner": winner_agent,
                "loser": loser_agent,
                "position": conflict.conflict_position.copy(),
            }
            self.conflict_count += 1
            self.reroute_count += 1
            self.stop_count += 1

            self._record_event(
                "CONFLICT\n"
                f"   Robots: {agent_a.robot_id} <-> {agent_b.robot_id}\n"
                f"   Conflict point: ({conflict.conflict_position[0]:.2f}, {conflict.conflict_position[1]:.2f})\n"
                f"   Predicted distance: {conflict.minimum_distance:.2f} m\n"
                f"   TTC: {conflict.time_to_conflict:.2f} s\n"
                f"   PROCEED + REROUTE: {winner_agent.robot_id}\n"
                f"   STOP: {loser_agent.robot_id}\n"
                f"   New path: {len(winner_agent.planned_path)} waypoints"
            )
            self._record_robot_event(winner_agent.robot_id, f"conflict with {loser_agent.robot_id} -> proceed + reroute")
            self._record_robot_event(loser_agent.robot_id, f"conflict with {winner_agent.robot_id} -> stop for priority")

    def _apply_hard_safety(self):
        for i in range(len(self.agents)):
            for j in range(i + 1, len(self.agents)):
                agent_a = self.agents[i]
                agent_b = self.agents[j]
                distance = np.linalg.norm(agent_a.state.position - agent_b.state.position)
                threshold = float(self.config.get("simulation", {}).get("hard_safety_distance", 0.70))

                if distance < threshold:
                    self.safety_violations += 1
                    agent_a.stop(reason="HARD_SAFETY")
                    agent_b.stop(reason="HARD_SAFETY")
                    self._record_event(f"HARD SAFETY: {agent_a.robot_id} and {agent_b.robot_id} too close")

    def step(self, dt: Optional[float] = None) -> Dict[str, Any]:
        if self.env is None:
            raise RuntimeError("Simulation environment not initialized.")

        dt = dt if dt is not None else float(self.config.get("simulation", {}).get("time_step", 0.1))

        for agent in self.agents:
            agent.update(self.env.time)

        self.network.broadcast_all()
        self._release_active_conflicts()

        conflicts = self.detector.detect_all(self.agents)
        self._resolve_conflicts(conflicts)
        self._apply_hard_safety()

        actions = [agent.desired_velocity() for agent in self.agents]
        self.env.step(actions)
        self.simulation_time = float(self.env.time)
        self._sync_task_statuses()

        if self.is_complete():
            self.status = "COMPLETED"
        elif self.status == "IDLE":
            self.status = "RUNNING"

        return self.snapshot()

    def run(self, max_steps: Optional[int] = None, render: bool = False) -> Dict[str, Any]:
        max_steps = int(max_steps if max_steps is not None else self.config.get("simulation", {}).get("max_steps", 900))
        self.status = "RUNNING"

        for _ in range(max_steps):
            if self.env.done():
                break

            self.step()

            if render:
                self.env.render(0.01)

            if self.is_complete():
                self.status = "COMPLETED"
                break

        if self.status != "COMPLETED":
            self.status = "STOPPED"

        return self.snapshot()

    def reset(self):
        self.active_conflicts = {}
        self.simulation_time = 0.0
        self.conflict_count = 0
        self.reroute_count = 0
        self.stop_count = 0
        self.safety_violations = 0
        self.event_log = []
        self.robot_event_log = {}

        if self.env is not None:
            try:
                self.env.end()
            except Exception:
                pass

        self._build()
        self.status = "READY"
        self._record_event("SIMULATION READY")

    def is_complete(self) -> bool:
        if not self.agents:
            return False
        return all(agent.intent == "ARRIVED" for agent in self.agents)

    def compute_metrics(self) -> Dict[str, Any]:
        robots = self.agents
        moving = sum(1 for agent in robots if agent.intent == "MOVING")
        waiting = sum(1 for agent in robots if agent.intent in {"STOPPED", "REROUTING", "WAIT_FOR"})
        completed = sum(1 for agent in robots if agent.intent == "ARRIVED")

        return {
            "robots": len(robots),
            "active_tasks": len(self.config.get("tasks", [])),
            "completed_tasks": completed,
            "robots_moving": moving,
            "robots_waiting": waiting,
            "robots_rerouting": sum(1 for agent in robots if agent.intent == "REROUTING"),
            "conflicts_detected": self.conflict_count,
            "safety_violations": self.safety_violations,
            "simulation_time": self.simulation_time,
            "status": self.status,
            "throughput": (completed / max(self.simulation_time, 1e-6)),
        }

    def snapshot(self) -> Dict[str, Any]:
        robots = []
        for agent in self.agents:
            peer_states = {}
            for peer_id, peer_state in agent.get_peer_states().items():
                peer_states[peer_id] = {
                    "position": [float(v) for v in peer_state.position],
                    "intent": getattr(peer_state, "intent", "UNKNOWN"),
                    "stopped": bool(getattr(peer_state, "stopped", False)),
                    "heading": float(getattr(peer_state, "heading", 0.0)),
                    "task_id": getattr(peer_state, "task_id", None),
                }

            robots.append(
                {
                    "id": agent.robot_id,
                    "position": [float(v) for v in agent.state.position],
                    "start": [float(v) for v in getattr(agent, "start_position", agent.state.position)],
                    "heading": float(agent.state.heading),
                    "velocity": [float(v) for v in agent.state.velocity],
                    "battery": float(agent.state.battery),
                    "task_id": agent.state.task_id,
                    "goal": [float(v) for v in agent.state.goal] if agent.state.goal is not None else None,
                    "intent": agent.intent,
                    "eta": float(agent.state.eta) if agent.state.eta is not None else None,
                    "path_length": float(agent.path_length()),
                    "planned_path": [[float(v) for v in point] for point in agent.planned_path],
                    "stopped": bool(agent.stopped),
                    "stop_reason": agent.stop_reason,
                    "network": {
                        "known_robots": list(peer_states.keys()),
                        "peers": peer_states,
                    },
                }
            )

        tasks = []
        for task in self.config.get("tasks", []):
            tasks.append(
                {
                    "id": task.get("id"),
                    "pickup": task.get("pickup"),
                    "dropoff": task.get("dropoff"),
                    "priority": task.get("priority", 1),
                    "assigned_robot": task.get("assigned_robot"),
                    "status": task.get("status", "PENDING"),
                }
            )

        conflicts = []
        for pair, info in self.active_conflicts.items():
            winner = info["winner"].robot_id
            loser = info["loser"].robot_id
            conflicts.append({
                "pair": [pair[0], pair[1]],
                "winner": winner,
                "loser": loser,
                "position": [float(v) for v in info["position"]],
            })

        payload = {
            "simulation_time": float(self.simulation_time),
            "status": self.status,
            "warehouse": self.config.get("warehouse", {}),
            "simulation": self.config.get("simulation", {}),
            "robots": robots,
            "tasks": tasks,
            "conflicts": conflicts,
            "events": self.event_log[-30:],
            "robot_events": {robot_id: logs[-8:] for robot_id, logs in self.robot_event_log.items()},
            "metrics": self.compute_metrics(),
        }
        return payload


__all__ = ["SimulationManager"]
