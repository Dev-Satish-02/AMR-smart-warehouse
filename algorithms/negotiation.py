from dataclasses import dataclass

import numpy as np

from algorithms.conflict_detector import PredictedConflict


# ==========================================================================
# NEGOTIATION DECISION
# ==========================================================================

@dataclass
class NegotiationDecision:

    robot_id: str
    opponent_id: str

    priority: bool

    action: str

    reason: str

    conflict_position: np.ndarray

    time_to_conflict: float


# ==========================================================================
# NEGOTIATION MANAGER
# ==========================================================================

class NegotiationManager:
    """
    Simple decentralized-style priority negotiation.

    For two robots approaching the same conflict region:

        earlier ETA -> priority

    If ETAs are effectively equal:

        lower robot ID -> priority

    The lower-priority robot simply stops.
    """

    def __init__(
        self,
        eta_margin=0.25,
    ):

        self.eta_margin = eta_margin

    # ======================================================================
    # ETA
    # ======================================================================

    @staticmethod
    def estimate_eta(
        agent,
        conflict_position,
    ):

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

    # ======================================================================
    # NEGOTIATE
    # ======================================================================

    def negotiate(
        self,
        agent_a,
        agent_b,
        conflict: PredictedConflict,
    ):

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

        # ------------------------------------------------------------------
        # Priority
        # ------------------------------------------------------------------

        if (
            eta_a
            < eta_b - self.eta_margin
        ):

            winner = agent_a
            loser = agent_b
            winner_eta = eta_a
            loser_eta = eta_b

            reason = (
                f"{agent_a.robot_id} "
                "reaches the conflict "
                "region earlier"
            )

        elif (
            eta_b
            < eta_a - self.eta_margin
        ):

            winner = agent_b
            loser = agent_a
            winner_eta = eta_b
            loser_eta = eta_a

            reason = (
                f"{agent_b.robot_id} "
                "reaches the conflict "
                "region earlier"
            )

        else:

            # Deterministic tie-break.

            if (
                agent_a.robot_id
                < agent_b.robot_id
            ):

                winner = agent_a
                loser = agent_b
                winner_eta = eta_a
                loser_eta = eta_b

            else:

                winner = agent_b
                loser = agent_a
                winner_eta = eta_b
                loser_eta = eta_a

            reason = (
                "ETA tie; deterministic "
                "robot-ID priority"
            )

        # ------------------------------------------------------------------
        # Decisions
        # ------------------------------------------------------------------

        winner_decision = (
            NegotiationDecision(
                robot_id=winner.robot_id,
                opponent_id=loser.robot_id,
                priority=True,
                action="PROCEED",
                reason=reason,
                conflict_position=(
                    conflict_position.copy()
                ),
                time_to_conflict=(
                    winner_eta
                ),
            )
        )

        loser_decision = (
            NegotiationDecision(
                robot_id=loser.robot_id,
                opponent_id=winner.robot_id,
                priority=False,
                action="STOP",
                reason=(
                    f"{loser.robot_id} "
                    f"stops for "
                    f"{winner.robot_id}"
                ),
                conflict_position=(
                    conflict_position.copy()
                ),
                time_to_conflict=(
                    loser_eta
                ),
            )
        )

        return (
            winner_decision,
            loser_decision,
        )
