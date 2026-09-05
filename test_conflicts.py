import numpy as np
import irsim

from algorithms.planner import NEXUSPlanner
from algorithms.conflict_detector import ConflictDetector
from agents.robot_agent import RobotAgent
from communication.p2p import P2PNetwork


ROBOT_CONFIG = [
    ("R1", np.array([2.0, 3.0]), np.array([28.0, 17.0])),
    ("R2", np.array([2.0, 7.0]), np.array([28.0, 13.0])),
    ("R3", np.array([2.0, 11.0]), np.array([28.0, 9.0])),

    ("R4", np.array([28.0, 17.0]), np.array([2.0, 3.0])),
    ("R5", np.array([28.0, 13.0]), np.array([2.0, 7.0])),
    ("R6", np.array([28.0, 9.0]), np.array([2.0, 11.0])),
]


def main():

    print("=" * 72)
    print("          NEXUS PREDICTIVE CONFLICT DETECTION")
    print("=" * 72)

    env = irsim.make(
        "configs/warehouse.yaml"
    )

    planner = NEXUSPlanner()

    detector = ConflictDetector(
        prediction_horizon=4.0,
        prediction_dt=0.2,
        safety_distance=0.90,
    )

    network = P2PNetwork()

    agents = []

    # ==============================================================
    # INITIALIZE ROBOTS
    # ==============================================================

    for i, (
        robot_id,
        start,
        goal,
    ) in enumerate(ROBOT_CONFIG):

        agent = RobotAgent(
            robot_id=robot_id,
            robot=env.robot_list[i],
            planner=planner,
        )

        agent.set_task(
            task_id=f"TASK_{robot_id}",
            goal=goal,
        )

        agent.plan_path(
            start=start,
            goal=goal,
        )

        agent.update(0.0)

        agents.append(agent)

        network.register(agent)

    print()
    print(f"Robots: {len(agents)}")
    print("Communication: P2P")
    print("Conflict prediction horizon: 4.0 s")
    print("Safety distance: 0.90 m")
    print()

    # ==============================================================
    # SIMULATION
    # ==============================================================

    simulation_time = 0.0
    max_steps = 600

    previous_conflicts = set()

    for step in range(max_steps):

        # ----------------------------------------------------------
        # Update local robot state
        # ----------------------------------------------------------

        for agent in agents:
            agent.update(
                simulation_time
            )

        # ----------------------------------------------------------
        # P2P state exchange
        # ----------------------------------------------------------

        network.broadcast_all()

        # ----------------------------------------------------------
        # Predict fleet conflicts
        # ----------------------------------------------------------

        conflicts = detector.detect_all(
            agents
        )

        current_conflicts = set()

        for conflict in conflicts:

            pair = tuple(
                sorted(
                    [
                        conflict.robot_a,
                        conflict.robot_b,
                    ]
                )
            )

            current_conflicts.add(pair)

            # Only print newly detected conflicts
            if pair not in previous_conflicts:

                x = conflict.conflict_position[0]
                y = conflict.conflict_position[1]

                print(
                    f"[T={simulation_time:5.1f}s] "
                    f"PREDICTED CONFLICT: "
                    f"{conflict.robot_a} <-> "
                    f"{conflict.robot_b} | "
                    f"TTC="
                    f"{conflict.time_to_conflict:.1f}s | "
                    f"position="
                    f"({x:.2f}, {y:.2f}) | "
                    f"d_min="
                    f"{conflict.minimum_distance:.2f}m"
                )

        previous_conflicts = (
            current_conflicts
        )

        # ----------------------------------------------------------
        # IMPORTANT:
        #
        # Detector observes only.
        # It does NOT modify robot motion yet.
        # ----------------------------------------------------------

        actions = [
            agent.desired_velocity()
            for agent in agents
        ]

        env.step(actions)

        # ----------------------------------------------------------
        # Diagnostics
        # ----------------------------------------------------------

        if step % 50 == 0:

            print(
                f"[T={simulation_time:5.1f}s] "
                f"active_conflicts="
                f"{len(conflicts)}"
            )

        # ----------------------------------------------------------
        # Render
        # ----------------------------------------------------------

        if step % 2 == 0:
            env.render(0.001)

        simulation_time += 0.1

    env.end()


if __name__ == "__main__":
    main()
