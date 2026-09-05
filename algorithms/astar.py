from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class GridConfig:
    width: float = 30.0
    height: float = 20.0
    resolution: float = 0.5

    # Robot radius + additional safety margin.
    robot_radius: float = 0.35
    safety_margin: float = 0.15

    @property
    def inflation_radius(self) -> float:
        return self.robot_radius + self.safety_margin


class WarehouseGrid:
    """
    Discretized representation of the NEXUS warehouse.

    Coordinates are expressed in metres.
    """

    def __init__(self, config: GridConfig | None = None):
        self.config = config or GridConfig()

        self.width_cells = int(
            self.config.width / self.config.resolution
        )

        self.height_cells = int(
            self.config.height / self.config.resolution
        )

        # Obstacles represented as line segments:
        #
        # (x1, y1) -> (x2, y2)
        #
        # These correspond to the shelf/choke-point geometry
        # used in the IR-SIM warehouse.
        self.obstacles = [
            # Upper-left shelf
            ((4, 5), (11, 5)),
            ((11, 5), (11, 8)),

            # Upper-right shelf
            ((15, 5), (22, 5)),
            ((22, 5), (22, 8)),

            # Lower-left shelf
            ((4, 12), (11, 12)),
            ((11, 12), (11, 15)),

            # Lower-right shelf
            ((15, 12), (22, 12)),
            ((22, 12), (22, 15)),

            # Central choke-point structure
            ((13, 7), (13, 10)),
            ((13, 10), (17, 10)),
            ((17, 10), (17, 7)),
        ]

    # ---------------------------------------------------------
    # Coordinate conversion
    # ---------------------------------------------------------

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        gx = round(x / self.config.resolution)
        gy = round(y / self.config.resolution)

        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        x = gx * self.config.resolution
        y = gy * self.config.resolution

        return x, y

    # ---------------------------------------------------------
    # Geometry
    # ---------------------------------------------------------

    @staticmethod
    def point_to_segment_distance(
        px: float,
        py: float,
        ax: float,
        ay: float,
        bx: float,
        by: float,
    ) -> float:

        abx = bx - ax
        aby = by - ay

        apx = px - ax
        apy = py - ay

        ab_squared = abx * abx + aby * aby

        if ab_squared == 0:
            return math.hypot(
                px - ax,
                py - ay,
            )

        t = (
            apx * abx +
            apy * aby
        ) / ab_squared

        t = max(0.0, min(1.0, t))

        closest_x = ax + t * abx
        closest_y = ay + t * aby

        return math.hypot(
            px - closest_x,
            py - closest_y,
        )

    def is_obstacle(self, x: float, y: float) -> bool:
        """
        Returns True if the world coordinate is too close
        to a warehouse obstacle.
        """

        margin = self.config.inflation_radius

        for (a, b) in self.obstacles:

            distance = self.point_to_segment_distance(
                x,
                y,
                a[0],
                a[1],
                b[0],
                b[1],
            )

            if distance <= margin:
                return True

        return False

    def is_valid(self, node: tuple[int, int]) -> bool:

        gx, gy = node

        if gx < 0 or gx > self.width_cells:
            return False

        if gy < 0 or gy > self.height_cells:
            return False

        x, y = self.grid_to_world(gx, gy)

        return not self.is_obstacle(x, y)

    # ---------------------------------------------------------
    # Neighbours
    # ---------------------------------------------------------

    def neighbours(
        self,
        node: tuple[int, int],
    ) -> Iterable[tuple[tuple[int, int], float]]:

        gx, gy = node

        moves = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
        ]

        for dx, dy, cost in moves:

            neighbour = (
                gx + dx,
                gy + dy,
            )

            if self.is_valid(neighbour):
                yield neighbour, cost

    # ---------------------------------------------------------
    # Heuristic
    # ---------------------------------------------------------

    @staticmethod
    def heuristic(
        a: tuple[int, int],
        b: tuple[int, int],
    ) -> float:

        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # ---------------------------------------------------------
    # A*
    # ---------------------------------------------------------

    def plan(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
    ) -> list[tuple[float, float]]:

        start_node = self.world_to_grid(
            start[0],
            start[1],
        )

        goal_node = self.world_to_grid(
            goal[0],
            goal[1],
        )

        if not self.is_valid(start_node):
            raise ValueError(
                f"Start position is blocked: {start}"
            )

        if not self.is_valid(goal_node):
            raise ValueError(
                f"Goal position is blocked: {goal}"
            )

        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                start_node,
            ),
        )

        came_from = {}

        g_score = {
            start_node: 0.0
        }

        while open_set:

            _, current = heapq.heappop(
                open_set
            )

            if current == goal_node:

                return self._reconstruct_path(
                    came_from,
                    current,
                )

            for neighbour, move_cost in self.neighbours(
                current
            ):

                tentative_g = (
                    g_score[current] +
                    move_cost
                )

                if (
                    neighbour not in g_score
                    or tentative_g < g_score[neighbour]
                ):

                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g

                    f_score = (
                        tentative_g +
                        self.heuristic(
                            neighbour,
                            goal_node,
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score,
                            neighbour,
                        ),
                    )

        raise RuntimeError(
            f"A* could not find a path "
            f"from {start} to {goal}"
        )

    # ---------------------------------------------------------
    # Path reconstruction
    # ---------------------------------------------------------

    def _reconstruct_path(
        self,
        came_from: dict,
        current: tuple[int, int],
    ) -> list[tuple[float, float]]:

        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return [
            self.grid_to_world(
                gx,
                gy,
            )
            for gx, gy in path
        ]
