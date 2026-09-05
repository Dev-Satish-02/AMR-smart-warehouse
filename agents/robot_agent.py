from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from algorithms.planner import NEXUSPlanner


@dataclass
class RobotState:
    """
    State maintained locally by each NEXUS robot agent.
    """

    robot_id: int

    position: np.ndarray
    velocity: np.ndarray
    heading: float

    battery: float = 100.0

    task_id: Optional[str] = None
    goal: Optional[np.ndarray] = None

    planned_path: list = field(default_factory=list)

    intent: Optional[str] = None

    eta: Optional[float] = None

    timestamp: float = 0.0

    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "position": self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "heading": self.heading,
            "battery": self.battery,
            "task_id": self.task_id,
            "goal": (
                self.goal.tolist()
                if self.goal is not None
                else None
            ),
            "planned_path": self.planned_path,
            "intent": self.intent,
            "eta": self.eta,
            "timestamp": self.timestamp,
        }


class RobotAgent:
    """
    NEXUS software agent representing one AMR.

    The simulator robot is treated as the physical plant.
    RobotAgent contains the robot's local intelligence.
    """

    def __init__(
        self,
        robot_id: int,
        robot,
        planner: Optional[NEXUSPlanner] = None,
    ):

        self.robot_id = robot_id
        self.robot = robot

        # Shared planning interface.
        self.planner = planner or NEXUSPlanner()

        # Local world state.
        self.state = self._read_simulation_state()

        # Distributed world model.
        self.peer_states: dict[int, RobotState] = {}

        # Task state.
        self.current_task: Optional[str] = None

        # Planning state.
        self.planned_path: list[tuple[float, float]] = []

        # Current intent.
        self.intent = "IDLE"

    # =========================================================
    # STATE
    # =========================================================

    def _read_simulation_state(self) -> RobotState:

        state = np.asarray(
            self.robot.state
        ).flatten()

        goal = np.asarray(
            self.robot.goal
        ).flatten()

        position = state[:2].copy()

        heading = float(
            state[2]
        )

        return RobotState(
            robot_id=self.robot_id,
            position=position,
            velocity=np.zeros(2),
            heading=heading,
            goal=goal[:2].copy(),
        )

    def update(
        self,
        simulation_time: float,
    ):

        self.state = self._read_simulation_state()

        self.state.planned_path = self.planned_path

        self.state.intent = self.intent

        self.state.timestamp = simulation_time

    # =========================================================
    # P2P
    # =========================================================

    def receive_peer_state(
        self,
        peer_state: RobotState,
    ):

        if peer_state.robot_id == self.robot_id:
            return

        self.peer_states[
            peer_state.robot_id
        ] = peer_state

    def get_peer_states(self):

        return self.peer_states

    # =========================================================
    # TASK
    # =========================================================

    def set_task(
        self,
        task_id: str,
    ):

        self.current_task = task_id

        self.state.task_id = task_id

    # =========================================================
    # INTENT
    # =========================================================

    def set_intent(
        self,
        intent: str,
    ):

        self.intent = intent

        self.state.intent = intent

    # =========================================================
    # PLANNING
    # =========================================================

    def plan_path(self):

        start = tuple(
            self.state.position
        )

        goal = tuple(
            self.state.goal
        )

        self.planned_path = (
            self.planner.plan(
                start=start,
                goal=goal,
            )
        )

        self.state.planned_path = (
            self.planned_path
        )

        self.intent = "PATH_PLANNED"

        self.state.intent = self.intent

        return self.planned_path

    # =========================================================
    # PATH INFORMATION
    # =========================================================

    def path_length(self) -> float:

        if len(self.planned_path) < 2:
            return 0.0

        distance = 0.0

        for i in range(
            1,
            len(self.planned_path),
        ):

            x1, y1 = (
                self.planned_path[i - 1]
            )

            x2, y2 = (
                self.planned_path[i]
            )

            distance += np.hypot(
                x2 - x1,
                y2 - y1,
            )

        return float(distance)

    def set_path(self, path):

        self.planned_path = list(path)

        self.state.planned_path = (
            self.planned_path
        )

    # =========================================================
    # DEBUG
    # =========================================================

    def summary(self):

        return {
            "robot_id": self.robot_id,
            "position": self.state.position.tolist(),
            "goal": (
                self.state.goal.tolist()
                if self.state.goal is not None
                else None
            ),
            "path_waypoints": len(
                self.planned_path
            ),
            "path_length": self.path_length(),
            "intent": self.intent,
            "peer_count": len(
                self.peer_states
            ),
        }
