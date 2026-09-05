import numpy as np
import irsim

from agents.robot_agent import RobotAgent
from algorithms.planner import NEXUSPlanner
from algorithms.conflict_detector import ConflictDetector
from algorithms.negotiation import NegotiationManager
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
    print("             NEXUS DISTRIBUTED NEGOTIATION")
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

    negotiation = NegotiationManager()

    network = P2PNetwork()

    agents = []

    # ==============================================================
    # INITIALIZE
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

    # ==============================================================
    # SIMULATION
    # ==============================================================

    simulation_time = 0.0

    max_steps = 400

    negotiated_pairs = set()

    for step in range(max_steps):

        # ----------------------------------------------------------
        # Update states
        # ----------------------------------------------------------

        for agent in agents:
            agent.update(
                simulation_time
            )

        # ----------------------------------------------------------
        # P2P exchange
        # ----------------------------------------------------------

        network.broadcast_all()

        # ----------------------------------------------------------
        # Detect conflicts
        # ----------------------------------------------------------

        conflicts = detector.detect_all(
            agents
        )

        # ----------------------------------------------------------
        # Negotiate every newly observed pair
        # ----------------------------------------------------------

        for conflict in conflicts:

            pair = tuple(
                sorted(
                    [
                        conflict.robot_a,
                        conflict.robot_b,
                    ]
                )
            )

            if pair in negotiated_pairs:
                continue

            agent_a = next(
                agent
                for agent in agents
                if agent.robot_id
                == conflict.robot_a
            )

            agent_b = next(
                agent
                for agent in agents
                if agent.robot_id
                == conflict.robot_b
            )

            (
                winner,
                loser,
                reservation,
            ) = negotiation.negotiate(
                agent_a,
                agent_b,
                conflict,
            )

            print()
            print(
                f"[T={simulation_time:5.1f}s] "
                f"CONFLICT "
                f"{conflict.robot_a} <-> "
                f"{conflict.robot_b}"
            )

            print(
                f"   POSITION: "
                f"({conflict.conflict_position[0]:.2f}, "
                f"{conflict.conflict_position[1]:.2f})"
            )

            print(
                f"   WINNER:   "
                f"{winner.robot_id}"
            )

            print(
                f"   ACTION:   "
                f"{winner.action}"
            )

            print(
                f"   LOSER:    "
                f"{loser.robot_id}"
            )

            print(
                f"   ACTION:   "
                f"{loser.action}"
            )

            print(
                f"   RESERVATION: "
                f"{reservation.entry_time:.2f}s"
                f" → "
                f"{reservation.exit_time:.2f}s"
            )

            negotiated_pairs.add(
                pair
            )

        # ----------------------------------------------------------
        # Continue normal motion.
        #
        # Negotiation is OBSERVATIONAL in this test.
        # ----------------------------------------------------------

        actions = [
            agent.desired_velocity()
            for agent in agents
        ]

        env.step(actions)

        simulation_time += 0.1

    print()
    print("=" * 72)
    print(
        f"NEGOTIATION TEST COMPLETE"
    )
    print(
        f"Negotiated conflicts: "
        f"{len(negotiated_pairs)}"
    )
    print("=" * 72)

    env.end()


if __name__ == "__main__":
    main()
