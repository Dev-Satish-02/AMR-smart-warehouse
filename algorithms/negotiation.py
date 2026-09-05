from dataclasses import dataclass

import numpy as np

from algorithms.conflict_detector import PredictedConflict


# ======================================================================
# RESERVATION
# ======================================================================

@dataclass
class Reservation:
    robot_id: str
    resource_position: np.ndarray
    radius: float

    entry_time: float
    exit_time: float

    priority_score: float = 0.0


# ======================================================================
# RESERVATION MANAGER
# ======================================================================

class ReservationManager:
    """
    Spatial-temporal reservation manager.

    A reservation represents ownership of a physical conflict
    region during a specific time interval.
    """

    def __init__(
        self,
        zone_radius=1.25,
        traversal_time=2.5,
    ):

        self.zone_radius = float(
            zone_radius
        )

        self.traversal_time = float(
            traversal_time
        )

        self.reservations = []

    # ------------------------------------------------------------------
    # PRIORITY
    # ------------------------------------------------------------------

    @staticmethod
    def has_priority(
        robot_a,
        eta_a,
        robot_b,
        eta_b,
    ):
        """
        Earlier ETA gets priority.

        Robot ID provides a deterministic tie-breaker.
        """

        if eta_a < eta_b:
            return True

        if eta_b < eta_a:
            return False

        return robot_a < robot_b

    # ------------------------------------------------------------------
    # SPATIAL OVERLAP
    # ------------------------------------------------------------------

    @staticmethod
    def spatially_overlaps(
        position_a,
        radius_a,
        position_b,
        radius_b,
    ):

        distance = np.linalg.norm(
            np.asarray(position_a)
            - np.asarray(position_b)
        )

        return (
            distance
            <= radius_a + radius_b
        )

    # ------------------------------------------------------------------
    # TEMPORAL OVERLAP
    # ------------------------------------------------------------------

    @staticmethod
    def temporally_overlaps(
        entry_a,
        exit_a,
        entry_b,
        exit_b,
    ):

        return not (
            exit_a <= entry_b
            or exit_b <= entry_a
        )

    # ------------------------------------------------------------------
    # FIND CONFLICTING RESERVATION
    # ------------------------------------------------------------------

    def find_conflict(
        self,
        resource_position,
        radius,
        entry_time,
        exit_time,
        exclude_robot=None,
    ):

        for reservation in self.reservations:

            if (
                exclude_robot is not None
                and reservation.robot_id
                == exclude_robot
            ):
                continue

            spatial_match = (
                self.spatially_overlaps(
                    resource_position,
                    radius,
                    reservation.resource_position,
                    reservation.radius,
                )
            )

            if not spatial_match:
                continue

            temporal_match = (
                self.temporally_overlaps(
                    entry_time,
                    exit_time,
                    reservation.entry_time,
                    reservation.exit_time,
                )
            )

            if temporal_match:
                return reservation

        return None

    # ------------------------------------------------------------------
    # CREATE RESERVATION
    # ------------------------------------------------------------------

    def create_reservation(
        self,
        robot_id,
        resource_position,
        current_time,
        eta,
        priority_score,
        radius=None,
    ):

        if radius is None:
            radius = self.zone_radius

        entry_time = (
            current_time
            + max(
                float(eta),
                0.0,
            )
        )

        exit_time = (
            entry_time
            + self.traversal_time
        )

        reservation = Reservation(
            robot_id=robot_id,
            resource_position=np.asarray(
                resource_position,
                dtype=float,
            ).copy(),
            radius=float(radius),
            entry_time=float(entry_time),
            exit_time=float(exit_time),
            priority_score=float(
                priority_score
            ),
        )

        self.reservations.append(
            reservation
        )

        return reservation

    # ------------------------------------------------------------------
    # CLEAR EXPIRED
    # ------------------------------------------------------------------

    def clear_expired(
        self,
        current_time,
    ):

        self.reservations = [
            reservation
            for reservation in self.reservations
            if reservation.exit_time
            > current_time
        ]

    # ------------------------------------------------------------------
    # RELEASE ROBOT
    # ------------------------------------------------------------------

    def release_robot(
        self,
        robot_id,
    ):

        self.reservations = [
            reservation
            for reservation in self.reservations
            if reservation.robot_id
            != robot_id
        ]


# ======================================================================
# NEGOTIATION DECISION
# ======================================================================

@dataclass
class NegotiationDecision:

    robot_id: str

    opponent_id: str

    priority: bool

    action: str

    reason: str

    conflict_position: np.ndarray

    time_to_conflict: float

    yield_position: object = None


# ======================================================================
# NEGOTIATION MANAGER
# ======================================================================

class NegotiationManager:
    """
    NEXUS distributed-style conflict negotiation.

    Priority:

        1. Earlier ETA to conflict region
        2. Lower robot ID as deterministic tie-breaker

    Each negotiation produces a spatial-temporal reservation
    and a physical yield point for the losing robot.
    """

    def __init__(
        self,
        reservation_manager=None,
        zone_radius=1.25,
        traversal_time=2.5,
        yield_distance=1.50,
    ):

        if reservation_manager is None:

            reservation_manager = (
                ReservationManager(
                    zone_radius=zone_radius,
                    traversal_time=traversal_time,
                )
            )

        self.reservations = (
            reservation_manager
        )

        self.zone_radius = float(
            zone_radius
        )

        self.traversal_time = float(
            traversal_time
        )

        self.yield_distance = float(
            yield_distance
        )

    # ==================================================================
    # ETA
    # ==================================================================

    @staticmethod
    def estimate_eta(
        agent,
        conflict_position,
    ):
        """
        Estimate ETA using distance along the remaining A* path.

        This is preferable to straight-line distance because the
        warehouse contains obstacles and the robots follow the
        planned path.
        """

        if not agent.planned_path:

            return float("inf")

        conflict_position = np.asarray(
            conflict_position,
            dtype=float,
        )

        points = [
            agent.state.position.copy()
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

        best_distance = float(
            "inf"
        )

        distance_so_far = 0.0

        # ----------------------------------------------------------
        # Find closest point on remaining path
        # to the conflict position.
        # ----------------------------------------------------------

        for i in range(
            len(points) - 1
        ):

            start = points[i]
            end = points[i + 1]

            segment = (
                end - start
            )

            segment_length = np.linalg.norm(
                segment
            )

            if segment_length < 1e-8:
                continue

            t = np.dot(
                conflict_position - start,
                segment,
            ) / (
                segment_length
                ** 2
            )

            t = np.clip(
                t,
                0.0,
                1.0,
            )

            projection = (
                start
                + t * segment
            )

            distance_to_conflict = (
                np.linalg.norm(
                    projection
                    - conflict_position
                )
            )

            # Conflict detector normally produces a point
            # directly on/near the robot trajectories.
            if (
                distance_to_conflict
                <= 1.75
            ):

                candidate_distance = (
                    distance_so_far
                    + t
                    * segment_length
                )

                best_distance = min(
                    best_distance,
                    candidate_distance,
                )

            distance_so_far += (
                segment_length
            )

        # ----------------------------------------------------------
        # Fallback
        # ----------------------------------------------------------

        if not np.isfinite(
            best_distance
        ):

            best_distance = np.linalg.norm(
                agent.state.position
                - conflict_position
            )

        speed = max(
            agent.max_speed,
            1e-6,
        )

        return float(
            best_distance
            / speed
        )

    # ==================================================================
    # YIELD POSITION
    # ==================================================================

    def compute_yield_position(
        self,
        agent,
        conflict_position,
    ):
        """
        Find a point on the robot's current A* path approximately
        yield_distance metres before the conflict region.
        """

        conflict_position = np.asarray(
            conflict_position,
            dtype=float,
        )

        if not agent.planned_path:

            return agent.state.position.copy()

        points = [
            agent.state.position.copy()
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

        # ----------------------------------------------------------
        # Locate conflict point on path.
        # ----------------------------------------------------------

        best_segment = None
        best_t = 0.0
        best_distance = float("inf")

        for i in range(
            len(points) - 1
        ):

            start = points[i]
            end = points[i + 1]

            segment = (
                end - start
            )

            length_sq = np.dot(
                segment,
                segment,
            )

            if length_sq < 1e-10:
                continue

            t = np.dot(
                conflict_position - start,
                segment,
            ) / length_sq

            t = np.clip(
                t,
                0.0,
                1.0,
            )

            projection = (
                start
                + t * segment
            )

            distance = np.linalg.norm(
                projection
                - conflict_position
            )

            if distance < best_distance:

                best_distance = distance
                best_segment = i
                best_t = t

        if best_segment is None:

            return agent.state.position.copy()

        # ----------------------------------------------------------
        # Distance from current robot position to conflict point.
        # ----------------------------------------------------------

        distance_to_conflict = 0.0

        for i in range(
            best_segment
        ):

            distance_to_conflict += (
                np.linalg.norm(
                    points[i + 1]
                    - points[i]
                )
            )

        segment_length = np.linalg.norm(
            points[
                best_segment + 1
            ]
            - points[
                best_segment
            ]
        )

        distance_to_conflict += (
            best_t
            * segment_length
        )

        # ----------------------------------------------------------
        # Stop before conflict zone.
        # ----------------------------------------------------------

        target_distance = max(
            0.0,
            distance_to_conflict
            - self.yield_distance,
        )

        # ----------------------------------------------------------
        # Walk along path to target distance.
        # ----------------------------------------------------------

        travelled = 0.0

        for i in range(
            len(points) - 1
        ):

            start = points[i]
            end = points[i + 1]

            length = np.linalg.norm(
                end - start
            )

            if length < 1e-10:
                continue

            if (
                travelled + length
                >= target_distance
            ):

                ratio = (
                    target_distance
                    - travelled
                ) / length

                ratio = np.clip(
                    ratio,
                    0.0,
                    1.0,
                )

                return (
                    start
                    + ratio
                    * (end - start)
                )

            travelled += length

        return points[-1].copy()

    # ==================================================================
    # NEGOTIATE
    # ==================================================================

    def negotiate(
        self,
        agent_a,
        agent_b,
        conflict: PredictedConflict,
        current_time,
    ):
        """
        Negotiate access to the shared conflict region.
        """

        conflict_position = (
            conflict.conflict_position
        )

        # ----------------------------------------------------------
        # ETA
        # ----------------------------------------------------------

        eta_a = self.estimate_eta(
            agent_a,
            conflict_position,
        )

        eta_b = self.estimate_eta(
            agent_b,
            conflict_position,
        )

        # ----------------------------------------------------------
        # Determine priority
        # ----------------------------------------------------------

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
        # Desired reservation interval
        # ----------------------------------------------------------

        desired_entry = (
            current_time
            + max(
                winner_eta,
                0.0,
            )
        )

        desired_exit = (
            desired_entry
            + self.traversal_time
        )

        # ----------------------------------------------------------
        # Check existing reservations
        # ----------------------------------------------------------

        existing = (
            self.reservations.find_conflict(
                resource_position=(
                    conflict_position
                ),
                radius=self.zone_radius,
                entry_time=desired_entry,
                exit_time=desired_exit,
                exclude_robot=winner.robot_id,
            )
        )

        if existing is not None:

            # Existing opponent reservation wins.
            if existing.robot_id == loser.robot_id:

                winner, loser = (
                    loser,
                    winner,
                )

                winner_eta = (
                    self.estimate_eta(
                        winner,
                        conflict_position,
                    )
                )

                loser_eta = (
                    self.estimate_eta(
                        loser,
                        conflict_position,
                    )
                )

                desired_entry = (
                    current_time
                    + max(
                        winner_eta,
                        0.0,
                    )
                )

                desired_exit = (
                    desired_entry
                    + self.traversal_time
                )

            else:

                # Another robot currently owns the resource.
                desired_entry = max(
                    desired_entry,
                    existing.exit_time,
                )

                desired_exit = (
                    desired_entry
                    + self.traversal_time
                )

        # ----------------------------------------------------------
        # Create reservation
        # ----------------------------------------------------------

        reservation = Reservation(
            robot_id=winner.robot_id,
            resource_position=np.asarray(
                conflict_position,
                dtype=float,
            ).copy(),
            radius=self.zone_radius,
            entry_time=float(
                desired_entry
            ),
            exit_time=float(
                desired_exit
            ),
            priority_score=float(
                -winner_eta
            ),
        )

        self.reservations.reservations.append(
            reservation
        )

        # ----------------------------------------------------------
        # Compute physical yield point
        # ----------------------------------------------------------

        yield_position = (
            self.compute_yield_position(
                loser,
                conflict_position,
            )
        )

        # ----------------------------------------------------------
        # Winner decision
        # ----------------------------------------------------------

        winner_decision = NegotiationDecision(
            robot_id=winner.robot_id,
            opponent_id=loser.robot_id,
            priority=True,
            action="PROCEED",
            reason=(
                f"{winner.robot_id} has priority "
                f"for the shared conflict zone"
            ),
            conflict_position=(
                conflict_position.copy()
            ),
            time_to_conflict=winner_eta,
        )

        # ----------------------------------------------------------
        # Loser decision
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
            yield_position=(
                yield_position.copy()
            ),
        )

        return (
            winner_decision,
            loser_decision,
            reservation,
        )
