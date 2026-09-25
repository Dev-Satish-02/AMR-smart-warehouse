import heapq
import math

import numpy as np

from algorithms.astar import (
    GridConfig,
    WarehouseGrid,
)


# ==========================================================================
# LANE GRID
# ==========================================================================

# Headings used by the turn-aware search: index -> (dx, dy)
_DIRECTIONS = [
    (1, 0),
    (0, 1),
    (-1, 0),
    (0, -1),
]


class LaneGrid(WarehouseGrid):

    """
    A* over a painted lane layout (nexus.layout.Layout).

    Same interface as WarehouseGrid, but:

        * only lane / station cells are drivable
        * one-way lanes are respected
        * stations are endpoints only (never driven through)
        * cell (x, y) has its centre at ((x + 0.5), (y + 0.5)) * cell_size
        * slow cells (safety zones) cost more to cross
        * turning costs extra, so paths are straight runs joined by
          right-angle turns instead of staircases
    """

    def __init__(
        self,
        layout,
        robot_radius=0.35,
        safety_margin=0.15,
        turn_penalty=1.0,
    ):

        super().__init__(
            GridConfig(
                width=layout.width * layout.cell_size,
                height=layout.height * layout.cell_size,
                resolution=layout.cell_size,
                robot_radius=robot_radius,
                safety_margin=safety_margin,
            )
        )

        self.layout = layout

        # Static geometry comes from the layout, not from segments.
        self.obstacles = []

        self.grid_width = layout.width
        self.grid_height = layout.height

        self.turn_penalty = float(turn_penalty)

    # ======================================================================
    # COORDINATE CONVERSION
    # ======================================================================

    def world_to_grid(
        self,
        position,
    ):

        return self.layout.world_to_cell(
            position
        )

    def grid_to_world(
        self,
        cell,
    ):

        return np.array(
            self.layout.cell_center(
                cell
            ),
            dtype=float,
        )

    # ======================================================================
    # VALIDITY
    # ======================================================================

    def is_valid(
        self,
        cell,
        dynamic_obstacles=None,
        allow_start=False,
    ):

        if not self.layout.is_drivable(
            cell
        ):

            return False

        return super().is_valid(
            cell,
            dynamic_obstacles=dynamic_obstacles,
            allow_start=allow_start,
        )

    # ======================================================================
    # A* WITH TURN COST
    # ======================================================================

    def plan(
        self,
        start,
        goal,
        dynamic_obstacles=None,
    ):

        start_cell = self.world_to_grid(
            start
        )

        goal_cell = self.world_to_grid(
            goal
        )

        if start_cell == goal_cell:

            return [
                self.grid_to_world(
                    start_cell
                )
            ]

        if not self.is_valid(
            goal_cell,
            dynamic_obstacles=dynamic_obstacles,
        ):

            return []

        # State: (cell, heading index). Start heading is free (-1).
        start_state = (
            start_cell,
            -1,
        )

        open_set = [
            (
                0.0,
                0,
                start_state,
            )
        ]

        counter = 1

        cost_so_far = {
            start_state: 0.0
        }

        came_from = {}

        while open_set:

            _, _, state = heapq.heappop(
                open_set
            )

            cell, heading = state

            if cell == goal_cell:

                cells = [
                    cell
                ]

                while state in came_from:

                    state = came_from[
                        state
                    ]

                    cells.append(
                        state[0]
                    )

                cells.reverse()

                return [
                    self.grid_to_world(
                        c
                    )
                    for c in cells
                ]

            # Stations are endpoints only.
            if (
                cell != start_cell
                and self.layout.is_station(
                    cell
                )
            ):

                continue

            for index, (dx, dy) in enumerate(
                _DIRECTIONS
            ):

                neighbour = (
                    cell[0] + dx,
                    cell[1] + dy,
                )

                if not self.layout.move_allowed(
                    cell,
                    neighbour,
                ):

                    continue

                if not self.is_valid(
                    neighbour,
                    dynamic_obstacles=dynamic_obstacles,
                    allow_start=(
                        neighbour
                        == start_cell
                    ),
                ):

                    continue

                # Slow cells (crosswalks, slow zones) cost as long as
                # they take to cross, so A* avoids them when it can.
                step_cost = self.layout.cost_factor(
                    neighbour
                )

                if heading >= 0 and index != heading:

                    turns = (
                        2
                        if (index - heading) % 4 == 2
                        else 1
                    )

                    step_cost += (
                        self.turn_penalty
                        * turns
                    )

                next_state = (
                    neighbour,
                    index,
                )

                new_cost = (
                    cost_so_far[state]
                    + step_cost
                )

                if new_cost < cost_so_far.get(
                    next_state,
                    math.inf,
                ):

                    cost_so_far[
                        next_state
                    ] = new_cost

                    came_from[
                        next_state
                    ] = state

                    priority = (
                        new_cost
                        + self.heuristic(
                            neighbour,
                            goal_cell,
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            priority,
                            counter,
                            next_state,
                        ),
                    )

                    counter += 1

        return []


# ==========================================================================
# END
# ==========================================================================
