from algorithms.astar import (
    GridConfig,
    WarehouseGrid,
)


class NEXUSPlanner:

    def __init__(self):

        config = GridConfig(
            width=30.0,
            height=20.0,
            resolution=0.5,
            robot_radius=0.35,
            safety_margin=0.15,
        )

        self.grid = WarehouseGrid(
            config
        )

    def plan(
        self,
        start,
        goal,
        dynamic_obstacles=None,
    ):

        return self.grid.plan(
            start,
            goal,
            dynamic_obstacles=(
                dynamic_obstacles
            ),
        )
