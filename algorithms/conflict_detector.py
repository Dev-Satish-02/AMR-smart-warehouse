from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class PredictedConflict:
    """
    Predicted future conflict between two AMRs.
    """

    robot_a: str
    robot_b: str

    time_to_conflict: float

    position_a: np.ndarray
    position_b: np.ndarray

    conflict_position: np.ndarray

    minimum_distance: float

    conflict_type: str = "TRAJECTORY"


class ConflictDetector:
    """
    NEXUS predictive multi-robot conflict detector.

    Predicts each robot's future motion and checks for
    spatiotemporal safety-distance violations.
    """

    def __init__(
        self,
        prediction_horizon=4.0,
        prediction_dt=0.2,
        safety_distance=0.90,
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

    # ==================================================================
    # TRAJECTORY PREDICTION
    # ==================================================================

    def predict_trajectory(
        self,
        agent,
    ):
        """
        Predict future positions.

        Normal robot:
            move along remaining A* path.

        Yielding robot:
            move toward its assigned yield point,
            then remain stationary.

        This prevents the detector from repeatedly assuming
        that a yielding robot is still travelling through
        the conflict zone.
        """

        position = (
            agent.state.position.copy()
        )

        # ----------------------------------------------------------
        # NO PATH
        # ----------------------------------------------------------

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

        speed = max(
            agent.max_speed,
            1e-6,
        )

        # ----------------------------------------------------------
        # YIELDING ROBOT
        # ----------------------------------------------------------

        if agent.yielding:

            if agent.yield_position is None:

                return [
                    (
                        0.0,
                        position.copy(),
                    )
                ]

            yield_position = np.asarray(
                agent.yield_position,
                dtype=float,
            )

            distance_to_yield = np.linalg.norm(
                yield_position
                - position
            )

            times = np.arange(
                0.0,
                self.prediction_horizon
                + self.prediction_dt,
                self.prediction_dt,
            )

            predictions = []

            for future_time in times:

                travelled = (
                    speed
                    * future_time
                )

                if travelled >= distance_to_yield:

                    predicted_position = (
                        yield_position.copy()
                    )

                else:

                    if distance_to_yield < 1e-8:

                        predicted_position = (
                            position.copy()
                        )

                    else:

                        ratio = (
                            travelled
                            / distance_to_yield
                        )

                        predicted_position = (
                            position
                            + ratio
                            * (
                                yield_position
                                - position
                            )
                        )

                predictions.append(
                    (
                        float(future_time),
                        predicted_position,
                    )
                )

            return predictions

        # ----------------------------------------------------------
        # NORMAL A* TRAJECTORY
        # ----------------------------------------------------------

        remaining_points = [
            position.copy()
        ]

        remaining_points.extend(
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

        # ----------------------------------------------------------
        # Remove duplicate points
        # ----------------------------------------------------------

        path = [
            remaining_points[0]
        ]

        for point in remaining_points[1:]:

            if np.linalg.norm(
                point
                - path[-1]
            ) > 1e-8:

                path.append(point)

        if len(path) < 2:

            return [
                (
                    0.0,
                    position.copy(),
                )
            ]

        # ----------------------------------------------------------
        # Segment lengths
        # ----------------------------------------------------------

        segment_lengths = []

        for i in range(
            len(path) - 1
        ):

            length = np.linalg.norm(
                path[i + 1]
                - path[i]
            )

            segment_lengths.append(
                length
            )

        cumulative = [0.0]

        for length in segment_lengths:

            cumulative.append(
                cumulative[-1]
                + length
            )

        total_length = cumulative[-1]

        # ----------------------------------------------------------
        # Prediction
        # ----------------------------------------------------------

        predictions = []

        times = np.arange(
            0.0,
            self.prediction_horizon
            + self.prediction_dt,
            self.prediction_dt,
        )

        for future_time in times:

            travelled_distance = (
                speed
                * future_time
            )

            if (
                travelled_distance
                >= total_length
            ):

                predictions.append(
                    (
                        float(
                            future_time
                        ),
                        path[-1].copy(),
                    )
                )

                continue

            # Find containing segment.
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
                    <= travelled_distance
                    <= end_distance
                ):

                    segment_length = (
                        segment_lengths[
                            segment_index
                        ]
                    )

                    if (
                        segment_length
                        < 1e-8
                    ):

                        ratio = 0.0

                    else:

                        ratio = (
                            travelled_distance
                            - start_distance
                        ) / segment_length

                    start = path[
                        segment_index
                    ]

                    end = path[
                        segment_index + 1
                    ]

                    predicted_position = (
                        start
                        + ratio
                        * (
                            end
                            - start
                        )
                    )

                    predictions.append(
                        (
                            float(
                                future_time
                            ),
                            predicted_position,
                        )
                    )

                    break

        return predictions

    # ==================================================================
    # PAIRWISE CONFLICT
    # ==================================================================

    def detect_pair(
        self,
        agent_a,
        agent_b,
    ) -> Optional[PredictedConflict]:
        """
        Predict whether two robots will violate the
        safety distance within the prediction horizon.
        """

        # Do not generate a conflict between two robots
        # that have already completed their tasks.
        if (
            agent_a.intent == "ARRIVED"
            and agent_b.intent == "ARRIVED"
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

        minimum_distance = float(
            "inf"
        )

        for i in range(count):

            time_a, position_a = (
                trajectory_a[i]
            )

            time_b, position_b = (
                trajectory_b[i]
            )

            future_time = min(
                time_a,
                time_b,
            )

            distance = np.linalg.norm(
                position_a
                - position_b
            )

            minimum_distance = min(
                minimum_distance,
                distance,
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
                        future_time
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

    # ==================================================================
    # FLEET CONFLICTS
    # ==================================================================

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

        # Process the most urgent conflicts first.
        conflicts.sort(
            key=lambda conflict:
            conflict.time_to_conflict
        )

        return conflicts
