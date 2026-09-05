from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class RobotState:
    robot_id: str

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


class RobotAgent:
    """
    NEXUS agent representing one differential-drive AMR.

    Responsibilities:
    - Read robot state from IR-SIM
    - Maintain local peer state
    - Store an A* path
    - Track the active waypoint
    - Generate differential-drive [v, omega] commands
    """

    def __init__(self, robot_id, robot, planner=None):
        self.robot_id = robot_id
        self.robot = robot
        self.planner = planner

        self.state = RobotState(
            robot_id=robot_id,
            position=np.zeros(2),
            velocity=np.zeros(2),
            heading=0.0,
        )

        # --------------------------------------------------------------
        # Distributed world model
        # --------------------------------------------------------------

        self.peer_states = {}

        # --------------------------------------------------------------
        # Task
        # --------------------------------------------------------------

        self.current_task = None

        # --------------------------------------------------------------
        # Path
        # --------------------------------------------------------------

        self.planned_path = []
        self.waypoint_index = 0

        # --------------------------------------------------------------
        # Motion parameters
        # --------------------------------------------------------------

        self.max_speed = 0.8
        self.max_angular_speed = 1.5

        self.heading_gain = 2.5

        self.waypoint_tolerance = 0.25
        self.heading_tolerance = np.deg2rad(8.0)

        self.intent = "IDLE"

    # ==================================================================
    # STATE
    # ==================================================================

    def _read_simulation_state(self):
        """
        Read the current state from IR-SIM.

        IR-SIM differential-drive state:
            [x, y, theta]
        """

        state = np.asarray(self.robot.state).reshape(-1)

        self.state.position = np.array(
            [
                float(state[0]),
                float(state[1]),
            ]
        )

        if len(state) >= 3:
            self.state.heading = float(state[2])

        # IR-SIM exposes velocity in the robot's command frame.
        if hasattr(self.robot, "velocity"):
            try:
                velocity = np.asarray(
                    self.robot.velocity
                ).reshape(-1)

                if len(velocity) >= 2:
                    self.state.velocity = np.array(
                        [
                            float(velocity[0]),
                            float(velocity[1]),
                        ]
                    )
            except Exception:
                pass

        self.state.intent = self.intent

        # ETA is based on remaining path length.
        remaining_distance = self.remaining_path_length()

        if self.max_speed > 0:
            self.state.eta = (
                remaining_distance / self.max_speed
            )

    def update(self, simulation_time):
        """
        Update local robot state.
        """

        self._read_simulation_state()

        self.state.timestamp = simulation_time

    # ==================================================================
    # P2P COMMUNICATION
    # ==================================================================

    def receive_peer_state(self, peer_state):
        """
        Receive another robot's state.

        This represents NEXUS's logical peer-to-peer communication layer.
        """

        self.peer_states[peer_state.robot_id] = peer_state

    def get_peer_states(self):
        return self.peer_states

    # ==================================================================
    # TASK MANAGEMENT
    # ==================================================================

    def set_task(self, task_id, goal):
        """
        Assign a task and destination.
        """

        self.current_task = task_id

        self.state.task_id = task_id

        self.state.goal = np.asarray(
            goal,
            dtype=float,
        )

        self.intent = "MOVING"

    # ==================================================================
    # PATH PLANNING
    # ==================================================================

    def plan_path(self, start=None, goal=None):
        """
        Generate an A* path.
        """

        if self.planner is None:
            raise RuntimeError(
                f"{self.robot_id}: No planner configured."
            )

        if start is None:
            start = self.state.position

        if goal is None:
            if self.state.goal is None:
                raise ValueError(
                    f"{self.robot_id}: No goal specified."
                )

            goal = self.state.goal

        path = self.planner.plan(
            start,
            goal,
        )

        self.set_path(path)

        return path

    def set_path(self, path):
        """
        Store an A* path.

        Path format:
            [(x1, y1), (x2, y2), ...]
        """

        self.planned_path = [
            np.asarray(point, dtype=float)
            for point in path
        ]

        self.state.planned_path = self.planned_path

        self.waypoint_index = 0

    # ==================================================================
    # PATH METRICS
    # ==================================================================

    def path_length(self):
        """
        Total length of the planned path.
        """

        if len(self.planned_path) < 2:
            return 0.0

        total = 0.0

        for i in range(len(self.planned_path) - 1):
            total += np.linalg.norm(
                self.planned_path[i + 1]
                - self.planned_path[i]
            )

        return float(total)

    def remaining_path_length(self):
        """
        Distance remaining from the current position
        through the remaining waypoints.
        """

        if not self.planned_path:
            return 0.0

        if self.waypoint_index >= len(self.planned_path):
            return 0.0

        total = np.linalg.norm(
            self.planned_path[self.waypoint_index]
            - self.state.position
        )

        for i in range(
            self.waypoint_index,
            len(self.planned_path) - 1,
        ):
            total += np.linalg.norm(
                self.planned_path[i + 1]
                - self.planned_path[i]
            )

        return float(total)

    # ==================================================================
    # WAYPOINT MANAGEMENT
    # ==================================================================

    def current_waypoint(self):
        """
        Return the active waypoint.
        """

        if not self.planned_path:
            return None

        if self.waypoint_index >= len(self.planned_path):
            return self.planned_path[-1]

        return self.planned_path[self.waypoint_index]

    def waypoint_reached(self):
        """
        Check whether the active waypoint has been reached.
        """

        waypoint = self.current_waypoint()

        if waypoint is None:
            return True

        distance = np.linalg.norm(
            waypoint - self.state.position
        )

        return distance <= self.waypoint_tolerance

    def advance_waypoint(self):
        """
        Advance through any waypoints that have been reached.
        """

        advanced = False

        while (
            self.waypoint_index < len(self.planned_path)
            and self.waypoint_reached()
        ):
            self.waypoint_index += 1
            advanced = True

        if self.waypoint_index >= len(self.planned_path):
            self.intent = "ARRIVED"

            return True

        return advanced

    # ==================================================================
    # DIFFERENTIAL DRIVE CONTROLLER
    # ==================================================================

    @staticmethod
    def wrap_angle(angle):
        """
        Wrap angle to [-pi, pi].
        """

        return (
            angle + np.pi
        ) % (
            2.0 * np.pi
        ) - np.pi

    def desired_world_velocity(self):
        """
        Generate a desired velocity in WORLD coordinates.

        This function does NOT directly control IR-SIM.

        It produces:
            [vx, vy]

        which can later be passed through:
            robot.vel_world2body()

        This abstraction becomes important when ORCA is added.
        """

        if not self.planned_path:
            self.intent = "IDLE"

            return np.zeros(2)

        self.advance_waypoint()

        if self.waypoint_index >= len(self.planned_path):
            self.intent = "ARRIVED"

            return np.zeros(2)

        target = self.current_waypoint()

        delta = (
            target
            - self.state.position
        )

        distance = np.linalg.norm(delta)

        if distance < 1e-8:
            return np.zeros(2)

        direction = delta / distance

        return direction * self.max_speed

    def desired_velocity(self):
        """
        Generate a DIFFERENTIAL DRIVE command:

            [linear_velocity, angular_velocity]

        The waypoint direction is first expressed in world
        coordinates and then converted into the robot's
        differential-drive control space.
        """

        if not self.planned_path:
            self.intent = "IDLE"

            return np.array(
                [0.0, 0.0],
                dtype=float,
            )

        self.advance_waypoint()

        if self.waypoint_index >= len(self.planned_path):
            self.intent = "ARRIVED"

            return np.array(
                [0.0, 0.0],
                dtype=float,
            )

        target = self.current_waypoint()

        delta = (
            target
            - self.state.position
        )

        distance = np.linalg.norm(delta)

        if distance < 1e-8:
            return np.array(
                [0.0, 0.0],
                dtype=float,
            )

        # --------------------------------------------------------------
        # Desired heading toward waypoint
        # --------------------------------------------------------------

        desired_heading = np.arctan2(
            delta[1],
            delta[0],
        )

        heading_error = self.wrap_angle(
            desired_heading
            - self.state.heading
        )

        # --------------------------------------------------------------
        # Angular velocity
        # --------------------------------------------------------------

        angular_velocity = (
            self.heading_gain
            * heading_error
        )

        angular_velocity = np.clip(
            angular_velocity,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        # --------------------------------------------------------------
        # Linear velocity
        # --------------------------------------------------------------

        # When the heading error is large, slow down.
        heading_factor = max(
            0.0,
            np.cos(heading_error),
        )

        linear_velocity = (
            self.max_speed
            * heading_factor
        )

        # Slow down near the waypoint.
        if distance < 1.0:
            linear_velocity *= min(
                1.0,
                distance / 0.5,
            )

        # --------------------------------------------------------------
        # If badly misaligned, rotate first.
        # --------------------------------------------------------------

        if abs(heading_error) > np.deg2rad(70.0):
            linear_velocity = 0.0

        self.intent = "MOVING"

        return np.array(
            [
                linear_velocity,
                angular_velocity,
            ],
            dtype=float,
        )

    # ==================================================================
    # STATUS
    # ==================================================================

    def summary(self):
        position = self.state.position

        return (
            f"{self.robot_id}: "
            f"pos=({position[0]:.2f}, "
            f"{position[1]:.2f}) "
            f"heading={np.rad2deg(self.state.heading):.1f}deg "
            f"wp={self.waypoint_index}/"
            f"{len(self.planned_path)} "
            f"intent={self.intent}"
        )
