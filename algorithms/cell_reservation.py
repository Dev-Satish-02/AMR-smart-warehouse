from typing import Dict, Iterable, List, Optional, Set, Tuple

from algorithms.reservation import (
    ReservationManager,
)


Cell = Tuple[int, int]


class CellReservationTable:
    """
    Exclusive cell reservations for grid (lane) navigation.

    A robot may only enter a cell it has reserved. Cells are reserved ahead
    along the planned path and released once the robot has left them, so two
    robots can never occupy or enter the same cell.

    Priority ordering reuses ReservationManager (lower ETA wins, robot ID
    breaks ties).
    """

    def __init__(self):

        self.owner: Dict[Cell, str] = {}

        self._priority = ReservationManager()

    # ==============================================================
    # PRIORITY
    # ==============================================================

    def priority_key(
        self,
        eta,
        robot_id,
    ):

        return self._priority.priority_key(
            eta,
            robot_id,
        )

    # ==============================================================
    # RESERVATION
    # ==============================================================

    def holder(
        self,
        cell,
    ) -> Optional[str]:

        return self.owner.get(
            tuple(cell)
        )

    def reserve(
        self,
        robot_id,
        cell,
    ) -> bool:

        cell = tuple(cell)

        current = self.owner.get(
            cell
        )

        if current is not None and current != robot_id:

            return False

        self.owner[cell] = robot_id

        return True

    def held_by(
        self,
        robot_id,
    ) -> List[Cell]:

        return [
            cell
            for cell, owner in self.owner.items()
            if owner == robot_id
        ]

    def retain_only(
        self,
        robot_id,
        keep: Iterable[Cell],
    ):
        """Release every cell of robot_id that is not in keep."""

        keep = {
            tuple(cell)
            for cell in keep
        }

        for cell in self.held_by(robot_id):

            if cell not in keep:

                del self.owner[cell]

    def release_robot(
        self,
        robot_id,
    ):

        self.retain_only(
            robot_id,
            [],
        )

    def clear(self):

        self.owner = {}

    # ==============================================================
    # DEADLOCK
    # ==============================================================

    @staticmethod
    def find_cycle(
        wait_for: Dict[str, str],
    ) -> Optional[List[str]]:
        """
        wait_for maps robot -> robot it is blocked by.
        Returns the robots of one wait-for cycle, or None.
        """

        for start in wait_for:

            seen: List[str] = []
            visited: Set[str] = set()
            current = start

            while current in wait_for and current not in visited:

                visited.add(current)
                seen.append(current)
                current = wait_for[current]

            if current in visited:

                return seen[seen.index(current):]

        return None
