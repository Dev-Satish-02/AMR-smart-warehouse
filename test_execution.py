import time

import numpy as np
import matplotlib.pyplot as plt
import irsim

from algorithms.planner import NEXUSPlanner
from agents.robot_agent import RobotAgent


ROBOT_CONFIG = [
    ("R1", np.array([2.0, 3.0]), np.array([28.0, 17.0])),
    ("R2", np.array([2.0, 7.0]), np.array([28.0, 13.0])),
    ("R3", np.array([2.0, 11.0]), np.array([28.0, 9.0])),

    ("R4", np.array([28.0, 17.0]), np.array([2.0, 3.0])),
    ("R5", np.array([28.0, 13.0]), np.array([2.0, 7.0])),
    ("R6", np.array([28.0, 9.0]), np.array([2.0, 11.0])),
]


def main():

    print("=" * 70)
    print("              NEXUS PATH EXECUTION TEST")
    print("=" * 70)

    # --------------------------------------------------------------
    # Environment
    # --------------------------------------------------------------

    env = irsim.make(
        "configs/warehouse.yaml"
    )

    # --------------------------------------------------------------
    # Planner
    # --------------------------------------------------------------

    planner = NEXUSPlanner()

    # --------------------------------------------------------------
    # Agents
    # --------------------------------------------------------------

    agents = []

    for i, (
        robot_id,
        start,
        goal,
    ) in enumerate(ROBOT_CONFIG):

        robot = env.robot_list[i]

        agent = RobotAgent(
            robot_id=robot_id,
            robot=robot,
            planner=planner,
        )

        agent.set_task(
            task_id=f"TASK_{robot_id}",
            goal=goal,
        )

        path = agent.plan_path(
            start=start,
            goal=goal,
        )

        agents.append(agent)

        print(
            f"{robot_id}: "
            f"{len(path)} waypoints | "
            f"{agent.path_length():.2f} m"
        )

    # --------------------------------------------------------------
    # Store actual trajectories
    # --------------------------------------------------------------

    trajectories = {
        agent.robot_id: []
        for agent in agents
    }

    # --------------------------------------------------------------
    # Simulation
    # --------------------------------------------------------------

    simulation_time = 0.0
    max_steps = 800

    for step in range(max_steps):

        # ----------------------------------------------------------
        # Read current robot states
        # ----------------------------------------------------------

        for agent in agents:
            agent.update(simulation_time)

            trajectories[agent.robot_id].append(
                agent.state.position.copy()
            )

        # ----------------------------------------------------------
        # Generate NEXUS differential-drive commands
        # ----------------------------------------------------------

        actions = [
            agent.desired_velocity()
            for agent in agents
        ]

        # ----------------------------------------------------------
        # Execute actions
        # ----------------------------------------------------------

        env.step(actions)

        # ----------------------------------------------------------
        # Diagnostics
        # ----------------------------------------------------------

        if step % 20 == 0:

            print(
                f"[T={simulation_time:5.1f}s]"
            )

            for agent in agents:
                print(
                    "   "
                    + agent.summary()
                )

        # ----------------------------------------------------------
        # Render IR-SIM
        # ----------------------------------------------------------

        if step % 2 == 0:

            env.render(0.001)

            # ------------------------------------------------------
            # Draw planned A* paths
            # ------------------------------------------------------

            ax = env._env_plot.ax

            for agent in agents:

                if not agent.planned_path:
                    continue

                path = np.array(
                    agent.planned_path
                )

                ax.plot(
                    path[:, 0],
                    path[:, 1],
                    "--",
                    linewidth=1.0,
                    alpha=0.35,
                )

            # ------------------------------------------------------
            # Draw actual executed trajectories
            # ------------------------------------------------------

            for agent in agents:

                traj = np.array(
                    trajectories[agent.robot_id]
                )

                if len(traj) < 2:
                    continue

                ax.plot(
                    traj[:, 0],
                    traj[:, 1],
                    linewidth=2.0,
                    alpha=0.75,
                )

            plt.pause(0.001)

        # ----------------------------------------------------------
        # Stop if all robots arrive
        # ----------------------------------------------------------

        if all(
            agent.intent == "ARRIVED"
            for agent in agents
        ):

            print()
            print("=" * 70)
            print("ALL ROBOTS ARRIVED")
            print("=" * 70)

            break

        simulation_time += 0.1

    else:

        print()
        print("=" * 70)
        print("SIMULATION TIMEOUT")
        print("=" * 70)

    # --------------------------------------------------------------
    # Final diagnostics
    # --------------------------------------------------------------

    print()
    print("=" * 70)
    print("FINAL ROBOT STATES")
    print("=" * 70)

    for agent in agents:

        distance_to_goal = np.linalg.norm(
            agent.state.position
            - agent.state.goal
        )

        print(
            f"{agent.robot_id}: "
            f"position=("
            f"{agent.state.position[0]:.2f}, "
            f"{agent.state.position[1]:.2f}) "
            f"goal_error="
            f"{distance_to_goal:.2f} m "
            f"intent={agent.intent}"
        )

    env.end()


if __name__ == "__main__":
    main()
