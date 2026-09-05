from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Reservation:
    """
    Time-space reservation for a shared warehouse resource.
    """

    robot_id: str

    resource_position: np.ndarray

    entry_time: float
    exit_time: float

    priority_score: float


class ReservationManager:
    """
    NEXUS time-space reservation manager.

    Reservations use ABSOLUTE simulation time.

    Example:

        current simulation time = 13.3 s
        ETA to resource         = 2.7 s

        reservation:
            entry = 16.0 s
            exit  = 18.0 s
    """

    def __init__(
        self,
        resource_radius=1.2,
        crossing_time=2.0,
    ):
        self.resource_radius = resource_radius
        self.crossing_time = crossing_time

        self.reservations = {}

    # ==============================================================
    # PRIORITY
    # ==============================================================

    @staticmethod
    def robot_number(robot_id):
        """
        Extract numerical portion from IDs such as R1, R2, R10.
        """

        digits = "".join(
            character
            for character in robot_id
            if character.isdigit()
        )

        if digits:
            return int(digits)

        return 999999

    def priority_key(
        self,
        eta,
        robot_id,
    ):
        """
        Lower ETA wins.

        Robot ID provides deterministic tie-breaking.
        """

        return (
            float(eta),
            self.robot_number(robot_id),
        )

    def has_priority(
        self,
        robot_a_id,
        eta_a,
        robot_b_id,
        eta_b,
    ):
        """
        Determine which robot gets priority.
        """

        return self.priority_key(
            eta_a,
            robot_a_id,
        ) < self.priority_key(
            eta_b,
            robot_b_id,
        )

    # ==============================================================
    # RESERVATION
    # ==============================================================

    def create_reservation(
        self,
        robot_id,
        resource_position,
        current_time,
        eta,
        crossing_time=None,
        priority_score=0.0,
    ):
        """
        Create an absolute-time reservation.

        current_time:
            Current simulation time.

        eta:
            Estimated time from current position to resource.
        """

        if crossing_time is None:
            crossing_time = self.crossing_time

        entry_time = (
            float(current_time)
            + float(eta)
        )

        exit_time = (
            entry_time
            + float(crossing_time)
        )

        reservation = Reservation(
            robot_id=robot_id,
            resource_position=np.asarray(
                resource_position,
                dtype=float,
            ),
            entry_time=entry_time,
            exit_time=exit_time,
            priority_score=float(
                priority_score
            ),
        )

        self.reservations[robot_id] = reservation

        return reservation

    def get_reservation(
        self,
        robot_id,
    ) -> Optional[Reservation]:

        return self.reservations.get(
            robot_id
        )

    def active_reservations(
        self,
        current_time,
    ):
        """
        Return reservations that have not expired.
        """

        return [
            reservation
            for reservation
            in self.reservations.values()
            if reservation.exit_time
            >= current_time
        ]

    def clear_expired(
        self,
        current_time,
    ):
        """
        Remove expired reservations.
        """

        expired = [
            robot_id
            for robot_id, reservation
            in self.reservations.items()
            if reservation.exit_time
            < current_time
        ]

        for robot_id in expired:
            del self.reservations[robot_id]

    # ==============================================================
    # RESOURCE AVAILABILITY
    # ==============================================================

    def resource_available(
        self,
        resource_position,
        entry_time,
        exit_time,
        requesting_robot_id,
    ):
        """
        Check whether the resource is available during
        [entry_time, exit_time].
        """

        resource_position = np.asarray(
            resource_position,
            dtype=float,
        )

        for reservation in self.reservations.values():

            if (
                reservation.robot_id
                == requesting_robot_id
            ):
                continue

            spatial_distance = np.linalg.norm(
                reservation.resource_position
                - resource_position
            )

            if (
                spatial_distance
                > self.resource_radius
            ):
                continue

            temporal_overlap = (
                entry_time
                <= reservation.exit_time
                and
                exit_time
                >= reservation.entry_time
            )

            if temporal_overlap:
                return False

        return True
