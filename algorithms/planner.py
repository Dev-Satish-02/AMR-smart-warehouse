from __future__ import annotations

from algorithms.astar import (
    GridConfig,
    WarehouseGrid,
)


class NEXUSPlanner:
    """
    High-level planning interface for NEXUS.

    A* is the current implementation.

    Later this interface can support:

        A*
        CBS
        ECBS
        other MAPF planners
    """

    def __init__(self):

        config = GridConfig(
            width=30.0,
            height=20.0,
            resolution=0.5,
            robot_radius=0.35,
            safety_margin=0.15,
        )

        self.grid = WarehouseGrid(config)

    def plan(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
    ) -> list[tuple[float, float]]:

        return self.grid.plan(
            start,
            goal,
        )
