import numpy as np

from algorithms.conflict_detector import (
    ConflictDetector,
)


# ==========================================================================
# FAST CONFLICT DETECTOR
# ==========================================================================

class _HorizonView:

    """
    Read-only view of an agent whose planned path is cut just past the
    point the robot can reach within the prediction horizon. Everything
    ConflictDetector.predict_trajectory reads is forwarded unchanged.
    """

    def __init__(
        self,
        agent,
        reach,
    ):

        self.robot_id = agent.robot_id
        self.state = agent.state
        self.stopped = getattr(agent, "stopped", False)
        self.intent = agent.intent
        self.max_speed = agent.max_speed

        path = agent.planned_path[agent.waypoint_index:]
        position = agent.state.position
        kept = []
        travelled = 0.0

        for point in path:

            kept.append(point)
            travelled += float(np.linalg.norm(point - position))
            position = point

            # One point beyond the reachable distance keeps the sampled
            # positions identical to the full path.
            if travelled > reach:
                break

        self.planned_path = kept
        self.waypoint_index = 0


class FastConflictDetector(ConflictDetector):

    """
    Same conflicts as ConflictDetector.detect_all, found faster:

        * each robot's trajectory is predicted once per call, not once per
          pair it belongs to
        * only the part of the path reachable within the horizon is measured
        * pairs too far apart to come within the safety distance inside the
          horizon are skipped (exact: both robots together cannot close the
          gap in time)

    The prediction and pair logic are the original NEXUS methods.
    """

    def _horizon_reach(
        self,
        agent,
    ):

        times = np.arange(
            0.0,
            self.prediction_horizon
            + self.prediction_dt,
            self.prediction_dt,
        )

        return float(
            max(agent.max_speed, 0.0)
            * times[-1]
        )

    def predict_trajectory(
        self,
        agent,
    ):

        cache = getattr(
            self,
            "_cache",
            None,
        )

        if cache is not None and agent.robot_id in cache:

            return cache[agent.robot_id]

        prediction = super().predict_trajectory(
            _HorizonView(
                agent,
                self._horizon_reach(agent),
            )
        )

        if cache is not None:

            cache[agent.robot_id] = prediction

        return prediction

    def detect_pair(
        self,
        agent_a,
        agent_b,
    ):

        gap = float(
            np.linalg.norm(
                agent_a.state.position
                - agent_b.state.position
            )
        )

        if gap >= (
            self._horizon_reach(agent_a)
            + self._horizon_reach(agent_b)
            + self.safety_distance
        ):

            return None

        return super().detect_pair(
            agent_a,
            agent_b,
        )

    def detect_all(
        self,
        agents,
    ):

        self._cache = {}

        try:

            return super().detect_all(
                agents
            )

        finally:

            self._cache = None


# ==========================================================================
# END
# ==========================================================================
