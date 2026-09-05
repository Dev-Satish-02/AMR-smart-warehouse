from dataclasses import dataclass
from typing import Optional

import numpy as np


# ==========================================================================
# PREDICTED CONFLICT
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
    """
    Lightweight predictive conflict detector.

    Each robot's remaining A* path is projected forward at approximately
    max_speed.

    A conflict is raised when the predicted distance between two moving
    robots falls below safety_distance.
    """

    def __init__(
        self,
        prediction_horizon=3.0,
        prediction_dt=0.2,
        safety_distance=0.95,
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
    # TRAJECTORY PREDICTION
    # ======================================================================

    def predict_trajectory(
        self,
        agent,
    ):

        position = (
            agent.state.position.copy()
        )

        # A robot that is already stopped should not generate a new
        # moving-vs-moving conflict.

        if getattr(
            agent,
            "stopped",
            False,
        ):

            times = np.arange(
                0.0,
                self.prediction_horizon
                + self.prediction_dt,
                self.prediction_dt,
            )

            return [
                (
                    float(t),
                    position.copy(),
                )
                for t in times
            ]

        if agent.intent in {
            "STOPPED",
            "IDLE",
            "ARRIVED",
        }:

            times = np.arange(
                0.0,
                self.prediction_horizon
                + self.prediction_dt,
                self.prediction_dt,
            )

            return [
                (
                    float(t),
                    position.copy(),
                )
                for t in times
            ]

        if (
            not agent.planned_path
            or agent.waypoint_index
            >= len(agent.planned_path)
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
        # Remaining path
        # ------------------------------------------------------------------

        path = [
            position.copy()
        ]

        path.extend(
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

        cleaned_path = [
            path[0]
        ]

        for point in path[1:]:

            if np.linalg.norm(
                point
                - cleaned_path[-1]
            ) > 1e-8:

                cleaned_path.append(
                    point
                )

        path = cleaned_path

        if len(path) < 2:

            return [
                (
                    0.0,
                    position.copy(),
                )
            ]

        # ------------------------------------------------------------------
        # Segment lengths
        # ------------------------------------------------------------------

        segment_lengths = []

        for i in range(
            len(path) - 1
        ):

            segment_lengths.append(
                np.linalg.norm(
                    path[i + 1]
                    - path[i]
                )
            )

        cumulative = [0.0]

        for length in segment_lengths:

            cumulative.append(
                cumulative[-1]
                + length
            )

        total_length = cumulative[-1]

        # ------------------------------------------------------------------
        # Predict
        # ------------------------------------------------------------------

        predictions = []

        times = np.arange(
            0.0,
            self.prediction_horizon
            + self.prediction_dt,
            self.prediction_dt,
        )

        for future_time in times:

            distance = (
                speed
                * future_time
            )

            if distance >= total_length:

                predictions.append(
                    (
                        float(future_time),
                        path[-1].copy(),
                    )
                )

                continue

            for segment_index in range(
                len(segment_lengths)
            ):

                start_distance = (
                    cumulative[
                        segment_index
                    ]
                )

                end_distance = (
                    cumulative[
                        segment_index + 1
                    ]
                )

                if (
                    start_distance
                    <= distance
                    <= end_distance
                ):

                    segment_length = (
                        segment_lengths[
                            segment_index
                        ]
                    )

                    if segment_length < 1e-8:

                        ratio = 0.0

                    else:

                        ratio = (
                            distance
                            - start_distance
                        ) / segment_length

                    start = path[
                        segment_index
                    ]

                    end = path[
                        segment_index + 1
                    ]

                    predicted = (
                        start
                        + ratio
                        * (end - start)
                    )

                    predictions.append(
                        (
                            float(future_time),
                            predicted,
                        )
                    )

                    break

        return predictions

    # ======================================================================
    # PAIRWISE CONFLICT
    # ======================================================================

    def detect_pair(
        self,
        agent_a,
        agent_b,
    ) -> Optional[PredictedConflict]:

        # If either robot is already stopped, do not create another
        # moving-vs-moving negotiation.

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
                    position_a=position_a.copy(),
                    position_b=position_b.copy(),
                    conflict_position=(
                        conflict_position.copy()
                    ),
                    minimum_distance=float(
                        distance
                    ),
                )

        return None

    # ======================================================================
    # ALL CONFLICTS
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
