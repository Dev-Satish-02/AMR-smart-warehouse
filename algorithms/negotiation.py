from dataclasses import dataclass

import numpy as np

from algorithms.reservation import ReservationManager
from algorithms.conflict_detector import PredictedConflict


@dataclass
class NegotiationDecision:
    """
    Result of a pairwise negotiation.
    """

    robot_id: str

    opponent_id: str

    priority: bool

    action: str

    reason: str

    conflict_position: np.ndarray

    time_to_conflict: float


class NegotiationManager:
    """
    NEXUS distributed-style conflict negotiation.

    Priority is determined by:

        1. Earlier ETA to conflict region
        2. Lower robot ID as deterministic tie-breaker

    The actual reservation uses absolute simulation time.
    """

    def __init__(
        self,
        reservation_manager=None,
    ):

        if reservation_manager is None:
            reservation_manager = (
                ReservationManager()
            )

        self.reservations = (
            reservation_manager
        )

    # ==============================================================
    # ETA
    # ==============================================================

    @staticmethod
    def estimate_eta(
        agent,
        conflict_position,
    ):
        """
        Estimate time required to reach the conflict region.
        """

        conflict_position = np.asarray(
            conflict_position,
            dtype=float,
        )

        distance = np.linalg.norm(
            agent.state.position
            - conflict_position
        )

        speed = max(
            agent.max_speed,
            1e-6,
        )

        return float(
            distance / speed
        )

    # ==============================================================
    # NEGOTIATION
    # ==============================================================

    def negotiate(
        self,
        agent_a,
        agent_b,
        conflict: PredictedConflict,
        current_time,
    ):
        """
        Negotiate access to a shared conflict region.
        """

        conflict_position = (
            conflict.conflict_position
        )

        eta_a = self.estimate_eta(
            agent_a,
            conflict_position,
        )

        eta_b = self.estimate_eta(
            agent_b,
            conflict_position,
        )

        a_has_priority = (
            self.reservations.has_priority(
                agent_a.robot_id,
                eta_a,
                agent_b.robot_id,
                eta_b,
            )
        )

        if a_has_priority:

            winner = agent_a
            loser = agent_b
            winner_eta = eta_a
            loser_eta = eta_b

        else:

            winner = agent_b
            loser = agent_a
            winner_eta = eta_b
            loser_eta = eta_a

        # ----------------------------------------------------------
        # Create absolute-time reservation
        # ----------------------------------------------------------

        reservation = (
            self.reservations.create_reservation(
                robot_id=winner.robot_id,
                resource_position=conflict_position,
                current_time=current_time,
                eta=winner_eta,
                priority_score=-winner_eta,
            )
        )

        # ----------------------------------------------------------
        # Winner
        # ----------------------------------------------------------

        winner_decision = NegotiationDecision(
            robot_id=winner.robot_id,
            opponent_id=loser.robot_id,
            priority=True,
            action="PROCEED",
            reason=(
                f"{winner.robot_id} reaches "
                f"the shared resource earlier"
            ),
            conflict_position=(
                conflict_position.copy()
            ),
            time_to_conflict=winner_eta,
        )

        # ----------------------------------------------------------
        # Loser
        # ----------------------------------------------------------

        loser_decision = NegotiationDecision(
            robot_id=loser.robot_id,
            opponent_id=winner.robot_id,
            priority=False,
            action="YIELD",
            reason=(
                f"{loser.robot_id} yields to "
                f"{winner.robot_id}"
            ),
            conflict_position=(
                conflict_position.copy()
            ),
            time_to_conflict=loser_eta,
        )

        return (
            winner_decision,
            loser_decision,
            reservation,
        )
