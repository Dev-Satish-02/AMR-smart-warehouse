import heapq
import math
from dataclasses import dataclass

import numpy as np


# ==========================================================================
# GRID CONFIGURATION
# ==========================================================================

@dataclass
class GridConfig:

    width: float = 30.0
    height: float = 20.0

    resolution: float = 0.5

    robot_radius: float = 0.35
    safety_margin: float = 0.15


# ==========================================================================
# WAREHOUSE GRID
# ==========================================================================

class WarehouseGrid:

    def __init__(
        self,
        config=None,
    ):

        if config is None:
            config = GridConfig()

        self.config = config

        self.width = config.width
        self.height = config.height
        self.resolution = config.resolution

        self.robot_radius = (
            config.robot_radius
        )

        self.safety_margin = (
            config.safety_margin
        )

        # ------------------------------------------------------------------
        # Static warehouse obstacles.
        #
        # Each obstacle is represented as:
        #
        #     ((x1, y1), (x2, y2))
        #
        # These correspond to the shelves/choke points in warehouse.yaml.
        # ------------------------------------------------------------------

        self.obstacles = [

            (
                (4.0, 5.0),
                (11.0, 5.0),
            ),

            (
                (11.0, 5.0),
                (11.0, 8.0),
            ),

            (
                (15.0, 5.0),
                (22.0, 5.0),
            ),

            (
                (22.0, 5.0),
                (22.0, 8.0),
            ),

            (
                (4.0, 12.0),
                (11.0, 12.0),
            ),

            (
                (11.0, 12.0),
                (11.0, 15.0),
            ),

            (
                (15.0, 12.0),
                (22.0, 12.0),
            ),

            (
                (22.0, 12.0),
                (22.0, 15.0),
            ),

            (
                (13.0, 7.0),
                (13.0, 10.0),
            ),

            (
                (13.0, 10.0),
                (17.0, 10.0),
            ),

            (
                (17.0, 10.0),
                (17.0, 7.0),
            ),
        ]

        self.grid_width = int(
            round(
                self.width
                / self.resolution
            )
        )

        self.grid_height = int(
            round(
                self.height
                / self.resolution
            )
        )

    # ======================================================================
    # COORDINATE CONVERSION
    # ======================================================================

    def world_to_grid(
        self,
        position,
    ):

        position = np.asarray(
            position,
            dtype=float,
        )

        x = int(
            round(
                position[0]
                / self.resolution
            )
        )

        y = int(
            round(
                position[1]
                / self.resolution
            )
        )

        x = np.clip(
            x,
            0,
            self.grid_width - 1,
        )

        y = np.clip(
            y,
            0,
            self.grid_height - 1,
        )

        return (
            int(x),
            int(y),
        )

    def grid_to_world(
        self,
        cell,
    ):

        return np.array(
            [
                cell[0]
                * self.resolution,

                cell[1]
                * self.resolution,
            ],
            dtype=float,
        )

    # ======================================================================
    # GEOMETRY
    # ======================================================================

    @staticmethod
    def point_to_segment_distance(
        point,
        segment_start,
        segment_end,
    ):

        point = np.asarray(
            point,
            dtype=float,
        )

        segment_start = np.asarray(
            segment_start,
            dtype=float,
        )

        segment_end = np.asarray(
            segment_end,
            dtype=float,
        )

        segment = (
            segment_end
            - segment_start
        )

        length_squared = np.dot(
            segment,
            segment,
        )

        if length_squared < 1e-12:

            return float(
                np.linalg.norm(
                    point
                    - segment_start
                )
            )

        t = np.dot(
            point
            - segment_start,
            segment,
        ) / length_squared

        t = np.clip(
            t,
            0.0,
            1.0,
        )

        projection = (
            segment_start
            + t * segment
        )

        return float(
            np.linalg.norm(
                point
                - projection
            )
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

        x, y = cell

        # ------------------------------------------------------------------
        # World bounds.
        # ------------------------------------------------------------------

        if (
            x < 0
            or x >= self.grid_width
            or y < 0
            or y >= self.grid_height
        ):

            return False

        point = self.grid_to_world(
            cell
        )

        # ------------------------------------------------------------------
        # Static obstacles.
        # ------------------------------------------------------------------

        static_clearance = (
            self.robot_radius
            + self.safety_margin
        )

        for start, end in self.obstacles:

            distance = (
                self.point_to_segment_distance(
                    point,
                    start,
                    end,
                )
            )

            if distance < static_clearance:

                return False

        # ------------------------------------------------------------------
        # Dynamic obstacles.
        #
        # dynamic_obstacles:
        #
        # [
        #     (position, radius),
        #     ...
        # ]
        # ------------------------------------------------------------------

        if dynamic_obstacles:

            for (
                obstacle_position,
                obstacle_radius,
            ) in dynamic_obstacles:

                obstacle_position = np.asarray(
                    obstacle_position,
                    dtype=float,
                )

                distance = np.linalg.norm(
                    point
                    - obstacle_position
                )

                if (
                    distance
                    < obstacle_radius
                ):

                    # The current robot position is allowed even if it
                    # lies just inside the temporary obstacle. This is
                    # important because replanning can start close to
                    # the stopped robot.
                    if allow_start:

                        continue

                    return False

        return True

    # ======================================================================
    # NEIGHBOURS
    # ======================================================================

    @staticmethod
    def neighbours(
        cell,
    ):

        x, y = cell

        return [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]

    # ======================================================================
    # HEURISTIC
    # ======================================================================

    @staticmethod
    def heuristic(
        current,
        goal,
    ):

        return (
            abs(
                current[0]
                - goal[0]
            )
            +
            abs(
                current[1]
                - goal[1]
            )
        )

    # ======================================================================
    # A*
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

        # ------------------------------------------------------------------
        # A*.
        # ------------------------------------------------------------------

        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                start_cell,
            ),
        )

        came_from = {}

        cost_so_far = {
            start_cell: 0.0
        }

        while open_set:

            _, current = (
                heapq.heappop(
                    open_set
                )
            )

            # --------------------------------------------------------------
            # Goal.
            # --------------------------------------------------------------

            if current == goal_cell:

                path = [
                    current
                ]

                while current in came_from:

                    current = (
                        came_from[
                            current
                        ]
                    )

                    path.append(
                        current
                    )

                path.reverse()

                return [
                    self.grid_to_world(
                        cell
                    )
                    for cell in path
                ]

            # --------------------------------------------------------------
            # Explore neighbours.
            # --------------------------------------------------------------

            for neighbour in self.neighbours(
                current
            ):

                valid = self.is_valid(
                    neighbour,
                    dynamic_obstacles=(
                        dynamic_obstacles
                    ),
                    allow_start=(
                        neighbour
                        == start_cell
                    ),
                )

                if not valid:

                    continue

                new_cost = (
                    cost_so_far[current]
                    + 1.0
                )

                if (
                    neighbour
                    not in cost_so_far
                    or new_cost
                    < cost_so_far[
                        neighbour
                    ]
                ):

                    cost_so_far[
                        neighbour
                    ] = new_cost

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
                            neighbour,
                        ),
                    )

                    came_from[
                        neighbour
                    ] = current

        # ------------------------------------------------------------------
        # No path.
        # ------------------------------------------------------------------

        return []


# ==========================================================================
# END
# ==========================================================================
