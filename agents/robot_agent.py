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
    NEXUS AMR agent.

    Layers currently implemented:

        A* path
          ↓
        waypoint following
          ↓
        distributed coordination
          ↓
        differential-drive command

    ORCA will later sit between coordination and
    the final differential-drive command.
    """

    def __init__(self, robot_id, robot, planner=None):

        self.robot_id = robot_id
        self.robot = robot
        self.planner = planner

        # ----------------------------------------------------------
        # State
        # ----------------------------------------------------------

        self.state = RobotState(
            robot_id=robot_id,
            position=np.zeros(2),
            velocity=np.zeros(2),
            heading=0.0,
        )

        # ----------------------------------------------------------
        # Distributed world model
        # ----------------------------------------------------------

        self.peer_states = {}

        # ----------------------------------------------------------
        # Task
        # ----------------------------------------------------------

        self.current_task = None

        # ----------------------------------------------------------
        # Path
        # ----------------------------------------------------------

        self.planned_path = []
        self.waypoint_index = 0

        # ----------------------------------------------------------
        # Motion
        # ----------------------------------------------------------

        self.max_speed = 0.8
        self.max_angular_speed = 1.5

        self.heading_gain = 2.5

        self.waypoint_tolerance = 0.25

        # ----------------------------------------------------------
        # Coordination
        # ----------------------------------------------------------

        self.yielding = False
        self.yielding_to = None
        self.yield_position = None
        self.yield_until = 0.0

        self.intent = "IDLE"

    # ==================================================================
    # STATE
    # ==================================================================

    def _read_simulation_state(self):

        state = np.asarray(
            self.robot.state
        ).reshape(-1)

        self.state.position = np.array(
            [
                float(state[0]),
                float(state[1]),
            ]
        )

        if len(state) >= 3:
            self.state.heading = float(
                state[2]
            )

        # Try to read velocity if exposed by IR-SIM.
        for attribute in [
            "velocity",
            "vel",
        ]:

            if hasattr(self.robot, attribute):

                try:

                    velocity = np.asarray(
                        getattr(
                            self.robot,
                            attribute,
                        )
                    ).reshape(-1)

                    if len(velocity) >= 2:

                        self.state.velocity = np.array(
                            [
                                float(velocity[0]),
                                float(velocity[1]),
                            ]
                        )

                        break

                except Exception:
                    pass

        self.state.intent = self.intent

        remaining_distance = (
            self.remaining_path_length()
        )

        if self.max_speed > 0:

            self.state.eta = (
                remaining_distance
                / self.max_speed
            )

    def update(self, simulation_time):

        self._read_simulation_state()

        self.state.timestamp = (
            simulation_time
        )

        # Automatically release an expired yield.
        if (
            self.yielding
            and simulation_time >= self.yield_until
        ):

            self.clear_yield()

    # ==================================================================
    # P2P
    # ==================================================================

    def receive_peer_state(self, peer_state):

        self.peer_states[
            peer_state.robot_id
        ] = peer_state

    def get_peer_states(self):

        return self.peer_states

    # ==================================================================
    # TASK
    # ==================================================================

    def set_task(
        self,
        task_id,
        goal,
    ):

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

    def plan_path(
        self,
        start=None,
        goal=None,
    ):

        if self.planner is None:

            raise RuntimeError(
                f"{self.robot_id}: "
                "No planner configured."
            )

        if start is None:
            start = self.state.position

        if goal is None:

            if self.state.goal is None:

                raise ValueError(
                    f"{self.robot_id}: "
                    "No goal specified."
                )

            goal = self.state.goal

        path = self.planner.plan(
            start,
            goal,
        )

        self.set_path(path)

        return path

    def set_path(self, path):

        self.planned_path = [
            np.asarray(
                point,
                dtype=float,
            )
            for point in path
        ]

        self.state.planned_path = (
            self.planned_path
        )

        self.waypoint_index = 0

    # ==================================================================
    # PATH METRICS
    # ==================================================================

    def path_length(self):

        if len(self.planned_path) < 2:
            return 0.0

        total = 0.0

        for i in range(
            len(self.planned_path) - 1
        ):

            total += np.linalg.norm(
                self.planned_path[i + 1]
                - self.planned_path[i]
            )

        return float(total)

    def remaining_path_length(self):

        if not self.planned_path:
            return 0.0

        if (
            self.waypoint_index
            >= len(self.planned_path)
        ):
            return 0.0

        total = np.linalg.norm(
            self.planned_path[
                self.waypoint_index
            ]
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
    # WAYPOINTS
    # ==================================================================

    def current_waypoint(self):

        if not self.planned_path:
            return None

        if (
            self.waypoint_index
            >= len(self.planned_path)
        ):
            return self.planned_path[-1]

        return self.planned_path[
            self.waypoint_index
        ]

    def waypoint_reached(self):

        waypoint = self.current_waypoint()

        if waypoint is None:
            return True

        distance = np.linalg.norm(
            waypoint
            - self.state.position
        )

        return (
            distance
            <= self.waypoint_tolerance
        )

    def advance_waypoint(self):

        while (
            self.waypoint_index
            < len(self.planned_path)
            and self.waypoint_reached()
        ):

            self.waypoint_index += 1

        if (
            self.waypoint_index
            >= len(self.planned_path)
        ):

            self.intent = "ARRIVED"

            return True

        return False

    # ==================================================================
    # COORDINATION
    # ==================================================================

    def set_yield(
        self,
        yielding_to,
        yield_until,
        yield_position=None,
    ):
        """
        Put the robot into YIELD state.

        The robot will stop instead of entering the
        contested region.
        """

        self.yielding = True

        self.yielding_to = yielding_to

        self.yield_until = float(
            yield_until
        )

        if yield_position is not None:

            self.yield_position = np.asarray(
                yield_position,
                dtype=float,
            )

        self.intent = "YIELDING"

    def clear_yield(self):
        """
        Resume normal path execution.
        """

        self.yielding = False

        self.yielding_to = None

        self.yield_position = None

        self.yield_until = 0.0

        if (
            self.waypoint_index
            < len(self.planned_path)
        ):

            self.intent = "MOVING"

    # ==================================================================
    # DIFFERENTIAL DRIVE
    # ==================================================================

    @staticmethod
    def wrap_angle(angle):

        return (
            angle + np.pi
        ) % (
            2.0 * np.pi
        ) - np.pi

    def desired_velocity(self):

        # ----------------------------------------------------------
        # YIELD
        # ----------------------------------------------------------

        if self.yielding:

            self.intent = "YIELDING"

            return np.array(
                [
                    0.0,
                    0.0,
                ],
                dtype=float,
            )

        # ----------------------------------------------------------
        # No path
        # ----------------------------------------------------------

        if not self.planned_path:

            self.intent = "IDLE"

            return np.array(
                [
                    0.0,
                    0.0,
                ],
                dtype=float,
            )

        # ----------------------------------------------------------
        # Advance waypoint
        # ----------------------------------------------------------

        self.advance_waypoint()

        if (
            self.waypoint_index
            >= len(self.planned_path)
        ):

            self.intent = "ARRIVED"

            return np.array(
                [
                    0.0,
                    0.0,
                ],
                dtype=float,
            )

        target = self.current_waypoint()

        delta = (
            target
            - self.state.position
        )

        distance = np.linalg.norm(
            delta
        )

        if distance < 1e-8:

            return np.array(
                [
                    0.0,
                    0.0,
                ],
                dtype=float,
            )

        desired_heading = np.arctan2(
            delta[1],
            delta[0],
        )

        heading_error = self.wrap_angle(
            desired_heading
            - self.state.heading
        )

        # ----------------------------------------------------------
        # Angular velocity
        # ----------------------------------------------------------

        angular_velocity = (
            self.heading_gain
            * heading_error
        )

        angular_velocity = np.clip(
            angular_velocity,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        # ----------------------------------------------------------
        # Linear velocity
        # ----------------------------------------------------------

        heading_factor = max(
            0.0,
            np.cos(heading_error),
        )

        linear_velocity = (
            self.max_speed
            * heading_factor
        )

        # Slow down near waypoints.
        if distance < 1.0:

            linear_velocity *= min(
                1.0,
                distance / 0.5,
            )

        # If facing strongly away from target,
        # rotate before driving.
        if abs(heading_error) > np.deg2rad(
            70.0
        ):

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
            f"pos=("
            f"{position[0]:.2f}, "
            f"{position[1]:.2f}) "
            f"heading="
            f"{np.rad2deg(self.state.heading):.1f}deg "
            f"wp="
            f"{self.waypoint_index}/"
            f"{len(self.planned_path)} "
            f"intent="
            f"{self.intent}"
        )
