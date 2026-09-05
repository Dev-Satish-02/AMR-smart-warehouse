from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class RobotState:
    """
    State advertised by an AMR to its peers.
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

    def __init__(self, robot_id: int, robot):
        self.robot_id = robot_id
        self.robot = robot

        self.state = self._read_simulation_state()

        # Local copy of peer states.
        # This is the basis of the distributed world model.
        self.peer_states: dict[int, RobotState] = {}

        # Local task information
        self.current_task = None

        # Local trajectory
        self.planned_path = []

        # Current intent
        self.intent = "IDLE"

    def _read_simulation_state(self) -> RobotState:
        """
        Extract the current state from the IR-SIM robot.
        """

        state = np.asarray(
            self.robot.state
        ).flatten()

        goal = np.asarray(
            self.robot.goal
        ).flatten()

        # IR-SIM diff-drive state:
        # [x, y, theta]
        position = state[:2]
        heading = float(state[2])

        return RobotState(
            robot_id=self.robot_id,
            position=position.copy(),
            velocity=np.zeros(2),
            heading=heading,
            goal=goal[:2].copy(),
        )

    def update(self, simulation_time: float):
        """
        Synchronize NEXUS state with the simulator.
        """

        self.state = self._read_simulation_state()

        self.state.planned_path = self.planned_path
        self.state.intent = self.intent
        self.state.timestamp = simulation_time

    def receive_peer_state(self, peer_state: RobotState):
        """
        Store the most recently received state from another AMR.
        """

        if peer_state.robot_id == self.robot_id:
            return

        self.peer_states[peer_state.robot_id] = peer_state

    def get_peer_states(self):
        return self.peer_states

    def set_intent(self, intent: str):
        self.intent = intent
        self.state.intent = intent

    def set_task(self, task_id: str):
        self.current_task = task_id
        self.state.task_id = task_id

    def set_path(self, path):
        self.planned_path = path
        self.state.planned_path = path
