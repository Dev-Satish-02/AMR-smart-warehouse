import time

import numpy as np
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

    env = irsim.make("configs/warehouse.yaml")

    # --------------------------------------------------------------
    # Planner
    # --------------------------------------------------------------

    planner = NEXUSPlanner()

    # --------------------------------------------------------------
    # Create agents
    # --------------------------------------------------------------

    agents = []

    for i, (robot_id, start, goal) in enumerate(ROBOT_CONFIG):

        robot = env.robot_list[i]

        agent = RobotAgent(
            robot_id=robot_id,
            robot=robot,
            planner=planner,
        )

        agent.state.position = start.copy()

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

    print()
    print("Starting simulation...")
    print()

    # --------------------------------------------------------------
    # Simulation
    # --------------------------------------------------------------

    simulation_time = 0.0

    max_steps = 800

    for step in range(max_steps):

        # ----------------------------------------------------------
        # Update each NEXUS agent
        # ----------------------------------------------------------

        for agent in agents:
            agent.update(simulation_time)

        # ----------------------------------------------------------
        # Generate velocity commands
        # ----------------------------------------------------------

        actions = []

        for agent in agents:

            velocity = agent.desired_velocity()

            actions.append(velocity)

        # ----------------------------------------------------------
        # IR-SIM execution
        # ----------------------------------------------------------

        env.step(actions)

        if step % 20 == 0:

            print(
                f"[T={simulation_time:5.1f}s] "
                + " | ".join(
                    [
                        (
                            f"{agent.robot_id}: "
                            f"wp={agent.waypoint_index}/"
                            f"{len(agent.planned_path)}"
                        )
                        for agent in agents
                    ]
                )
            )

        # ----------------------------------------------------------
        # Rendering
        # ----------------------------------------------------------

        env.render()

        # ----------------------------------------------------------
        # Check completion
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

        time.sleep(0.01)

    else:

        print()
        print("=" * 70)
        print("SIMULATION TIMEOUT")
        print("=" * 70)


if __name__ == "__main__":
    main()
