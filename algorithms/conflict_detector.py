from dataclasses import dataclass
from typing import Optional

import numpy as np


# ==========================================================================
# CONFLICT
# ==========================================================================

@dataclass
class PredictedConflict:

    robot_a: str
    robot_b: str

    time_to_conflict: float

    position_a: np.ndarray
    position_b: np.ndarray

    conflict_position: np.ndarray

    minimum_distance: float

    conflict_type: str = "TRAJECTORY"


# ==========================================================================
# CONFLICT DETECTOR
# ==========================================================================

class ConflictDetector:

    def __init__(
        self,
        prediction_horizon=3.0,
        prediction_dt=0.2,
        safety_distance=1.40,
    ):

        self.prediction_horizon = (
            prediction_horizon
        )

        self.prediction_dt = (
            prediction_dt
        )

        self.safety_distance = (
            safety_distance
        )

    # ======================================================================
    # PREDICTION
    # ======================================================================

    def predict_trajectory(
        self,
        agent,
    ):

        position = (
            agent.state.position.copy()
        )

        # A stopped robot does not participate in a moving-vs-moving
        # prediction.

        if getattr(
            agent,
            "stopped",
            False,
        ):

            return [
                (
                    0.0,
                    position.copy(),
                )
            ]

        if agent.intent in {
            "STOPPED",
            "IDLE",
            "ARRIVED",
        }:

            return [
                (
                    0.0,
                    position.copy(),
                )
            ]

        if (
            not agent.planned_path
            or agent.waypoint_index
            >= len(
                agent.planned_path
            )
        ):

            return [
                (
                    0.0,
                    position.copy(),
                )
            ]

        speed = agent.max_speed

        if speed <= 0:

            return [
                (
                    0.0,
                    position.copy(),
                )
            ]

        # ------------------------------------------------------------------
        # Remaining path.
        # ------------------------------------------------------------------

        points = [
            position.copy()
        ]

        points.extend(
            [
                np.asarray(
                    point,
                    dtype=float,
                )
                for point in agent.planned_path[
                    agent.waypoint_index:
                ]
            ]
        )

        # Remove duplicate points.

        path = [
            points[0]
        ]

        for point in points[1:]:

            if np.linalg.norm(
                point
                - path[-1]
            ) > 1e-8:

                path.append(
                    point
                )

        if len(path) < 2:

            return [
                (
                    0.0,
                    position.copy(),
                )
            ]

        # ------------------------------------------------------------------
        # Segment lengths.
        # ------------------------------------------------------------------

        lengths = []

        for i in range(
            len(path) - 1
        ):

            lengths.append(
                np.linalg.norm(
                    path[i + 1]
                    - path[i]
                )
            )

        cumulative = [0.0]

        for length in lengths:

            cumulative.append(
                cumulative[-1]
                + length
            )

        total_length = (
            cumulative[-1]
        )

        # ------------------------------------------------------------------
        # Sample prediction.
        # ------------------------------------------------------------------

        predictions = []

        times = np.arange(
            0.0,
            self.prediction_horizon
            + self.prediction_dt,
            self.prediction_dt,
        )

        for future_time in times:

            travelled = (
                speed
                * future_time
            )

            if travelled >= total_length:

                predictions.append(
                    (
                        float(
                            future_time
                        ),
                        path[-1].copy(),
                    )
                )

                continue

            for i in range(
                len(lengths)
            ):

                if (
                    cumulative[i]
                    <= travelled
                    <= cumulative[i + 1]
                ):

                    segment_length = (
                        lengths[i]
                    )

                    if segment_length < 1e-8:

                        ratio = 0.0

                    else:

                        ratio = (
                            travelled
                            - cumulative[i]
                        ) / segment_length

                    predicted = (
                        path[i]
                        + ratio
                        * (
                            path[i + 1]
                            - path[i]
                        )
                    )

                    predictions.append(
                        (
                            float(
                                future_time
                            ),
                            predicted,
                        )
                    )

                    break

        return predictions

    # ======================================================================
    # PAIR
    # ======================================================================

    def detect_pair(
        self,
        agent_a,
        agent_b,
    ) -> Optional[
        PredictedConflict
    ]:

        # Do not negotiate with a robot that is already stopped.

        if getattr(
            agent_a,
            "stopped",
            False,
        ):

            return None

        if getattr(
            agent_b,
            "stopped",
            False,
        ):

            return None

        trajectory_a = (
            self.predict_trajectory(
                agent_a
            )
        )

        trajectory_b = (
            self.predict_trajectory(
                agent_b
            )
        )

        count = min(
            len(trajectory_a),
            len(trajectory_b),
        )

        for i in range(count):

            time_a, position_a = (
                trajectory_a[i]
            )

            time_b, position_b = (
                trajectory_b[i]
            )

            distance = np.linalg.norm(
                position_a
                - position_b
            )

            if (
                distance
                < self.safety_distance
            ):

                conflict_position = (
                    position_a
                    + position_b
                ) / 2.0

                return PredictedConflict(
                    robot_a=agent_a.robot_id,
                    robot_b=agent_b.robot_id,

                    time_to_conflict=float(
                        min(
                            time_a,
                            time_b,
                        )
                    ),

                    position_a=(
                        position_a.copy()
                    ),

                    position_b=(
                        position_b.copy()
                    ),

                    conflict_position=(
                        conflict_position.copy()
                    ),

                    minimum_distance=float(
                        distance
                    ),
                )

        return None

    # ======================================================================
    # ALL
    # ======================================================================

    def detect_all(
        self,
        agents,
    ):

        conflicts = []

        for i in range(
            len(agents)
        ):

            for j in range(
                i + 1,
                len(agents),
            ):

                conflict = (
                    self.detect_pair(
                        agents[i],
                        agents[j],
                    )
                )

                if conflict is not None:

                    conflicts.append(
                        conflict
                    )

        return conflicts
