import matplotlib.pyplot as plt
import numpy as np

from agents.robot_agent import RobotAgent
from algorithms.planner import NEXUSPlanner


# ---------------------------------------------------------
# Warehouse geometry
# ---------------------------------------------------------

OBSTACLES = [
    ((4, 5), (11, 5)),
    ((11, 5), (11, 8)),

    ((15, 5), (22, 5)),
    ((22, 5), (22, 8)),

    ((4, 12), (11, 12)),
    ((11, 12), (11, 15)),

    ((15, 12), (22, 12)),
    ((22, 12), (22, 15)),

    ((13, 7), (13, 10)),
    ((13, 10), (17, 10)),
    ((17, 10), (17, 7)),
]


ROBOTS = [
    ("R1", (2, 3), (28, 17)),
    ("R2", (2, 7), (28, 13)),
    ("R3", (2, 11), (28, 9)),
    ("R4", (28, 17), (2, 3)),
    ("R5", (28, 13), (2, 7)),
    ("R6", (28, 9), (2, 11)),
]


def draw_warehouse(ax):

    for start, end in OBSTACLES:

        x_values = [
            start[0],
            end[0],
        ]

        y_values = [
            start[1],
            end[1],
        ]

        ax.plot(
            x_values,
            y_values,
            linewidth=5,
            solid_capstyle="butt",
        )


def main():

    planner = NEXUSPlanner()

    print()
    print("=" * 70)
    print("              NEXUS PLANNED TRAJECTORIES")
    print("=" * 70)

    fig, ax = plt.subplots(
        figsize=(14, 8)
    )

    draw_warehouse(ax)

    for robot_id, start, goal in ROBOTS:

        # -------------------------------------------------
        # We only need the planning component here.
        # A real IR-SIM robot will be attached later.
        # -------------------------------------------------

        path = planner.plan(
            start=start,
            goal=goal,
        )

        distance = 0.0

        for i in range(
            1,
            len(path),
        ):

            x1, y1 = path[i - 1]
            x2, y2 = path[i]

            distance += np.hypot(
                x2 - x1,
                y2 - y1,
            )

        x = [
            point[0]
            for point in path
        ]

        y = [
            point[1]
            for point in path
        ]

        # Planned trajectory
        ax.plot(
            x,
            y,
            linewidth=2,
            label=robot_id,
        )

        # Start
        ax.scatter(
            start[0],
            start[1],
            s=80,
            marker="o",
            zorder=5,
        )

        # Goal
        ax.scatter(
            goal[0],
            goal[1],
            s=100,
            marker="x",
            zorder=5,
        )

        print(
            f"{robot_id}: "
            f"{len(path):3d} waypoints | "
            f"{distance:6.2f} m"
        )

    ax.set_title(
        "NEXUS — A* Planned Multi-Robot Trajectories",
        fontsize=16,
    )

    ax.set_xlabel(
        "X [m]"
    )

    ax.set_ylabel(
        "Y [m]"
    )

    ax.set_xlim(
        0,
        30,
    )

    ax.set_ylim(
        0,
        20,
    )

    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    ax.grid(
        True,
        alpha=0.2,
    )

    ax.legend(
        title="AMRs"
    )

    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
