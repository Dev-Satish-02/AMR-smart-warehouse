import numpy as np
import matplotlib.pyplot as plt
import irsim

from algorithms.planner import NEXUSPlanner
from algorithms.conflict_detector import ConflictDetector
from algorithms.negotiation import NegotiationManager
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
    print("             NEXUS ACTIVE COORDINATION")
    print("=" * 72)

    # ==============================================================
    # ENVIRONMENT
    # ==============================================================

    env = irsim.make(
        "configs/warehouse.yaml"
    )

    # ==============================================================
    # NEXUS MODULES
    # ==============================================================

    planner = NEXUSPlanner()

    detector = ConflictDetector(
        prediction_horizon=4.0,
        prediction_dt=0.2,
        safety_distance=0.90,
    )

    negotiation = NegotiationManager()

    network = P2PNetwork()

    # ==============================================================
    # CREATE ROBOT AGENTS
    # ==============================================================

    agents = []

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

        agents.append(agent)

        network.register(agent)

        print(
            f"{robot_id}: "
            f"{len(agent.planned_path)} waypoints | "
            f"{agent.path_length():.2f} m"
        )

    print()
    print("Communication: P2P")
    print("Planning: A*")
    print("Conflict prediction: 4.0 s horizon")
    print("Safety distance: 0.90 m")
    print("Coordination: ETA-based negotiation")
    print()

    # ==============================================================
    # TRAJECTORY HISTORY
    # ==============================================================

    trajectories = {
        agent.robot_id: []
        for agent in agents
    }

    # ==============================================================
    # ACTIVE NEGOTIATIONS
    # ==============================================================

    active_negotiations = {}

    # ==============================================================
    # SIMULATION
    # ==============================================================

    simulation_time = 0.0

    max_steps = 600

    for step in range(max_steps):

        # ----------------------------------------------------------
        # Update all agents
        # ----------------------------------------------------------

        for agent in agents:

            agent.update(
                simulation_time
            )

            trajectories[
                agent.robot_id
            ].append(
                agent.state.position.copy()
            )

        # ----------------------------------------------------------
        # Remove expired reservations
        # ----------------------------------------------------------

        negotiation.reservations.clear_expired(
            simulation_time
        )

        # ----------------------------------------------------------
        # P2P state exchange
        # ----------------------------------------------------------

        network.broadcast_all()

        # ----------------------------------------------------------
        # Predict future conflicts
        # ----------------------------------------------------------

        conflicts = detector.detect_all(
            agents
        )

        # ----------------------------------------------------------
        # Process predicted conflicts
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

            # ------------------------------------------------------
            # Check whether this pair already has an active
            # negotiation.
            # ------------------------------------------------------

            if pair in active_negotiations:

                reservation = (
                    active_negotiations[pair]
                )

                if (
                    simulation_time
                    <= reservation.exit_time
                ):

                    continue

                del active_negotiations[
                    pair
                ]

            # ------------------------------------------------------
            # Find actual RobotAgent objects.
            # ------------------------------------------------------

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

            # ------------------------------------------------------
            # NEGOTIATION
            # ------------------------------------------------------

            (
                winner_decision,
                loser_decision,
                reservation,
            ) = negotiation.negotiate(
                agent_a,
                agent_b,
                conflict,
                simulation_time,
            )

            # ------------------------------------------------------
            # IMPORTANT:
            #
            # winner_decision / loser_decision are NOT RobotAgent
            # objects.
            #
            # They are NegotiationDecision objects.
            #
            # Therefore we use their robot_id to retrieve the
            # corresponding RobotAgent.
            # ------------------------------------------------------

            winner_agent = next(
                agent
                for agent in agents
                if agent.robot_id
                == winner_decision.robot_id
            )

            loser_agent = next(
                agent
                for agent in agents
                if agent.robot_id
                == loser_decision.robot_id
            )

            # ------------------------------------------------------
            # WINNER
            # ------------------------------------------------------

            winner_agent.clear_yield()

            # ------------------------------------------------------
            # LOSER
            # ------------------------------------------------------

            loser_agent.set_yield(
                yielding_to=winner_agent.robot_id,
                yield_until=reservation.exit_time,
                yield_position=(
                    conflict.conflict_position
                ),
            )

            # ------------------------------------------------------
            # Store active negotiation.
            # ------------------------------------------------------

            active_negotiations[
                pair
            ] = reservation

            # ------------------------------------------------------
            # Diagnostics
            # ------------------------------------------------------

            print()
            print(
                "-" * 72
            )

            print(
                f"[T={simulation_time:5.1f}s] "
                f"CONFLICT DETECTED"
            )

            print(
                f"   Robots: "
                f"{conflict.robot_a} <-> "
                f"{conflict.robot_b}"
            )

            print(
                f"   Conflict position: "
                f"("
                f"{conflict.conflict_position[0]:.2f}, "
                f"{conflict.conflict_position[1]:.2f}"
                f")"
            )

            print(
                f"   Predicted minimum distance: "
                f"{conflict.minimum_distance:.2f} m"
            )

            print(
                f"   Predicted TTC: "
                f"{conflict.time_to_conflict:.2f} s"
            )

            print(
                f"   PROCEED: "
                f"{winner_agent.robot_id}"
            )

            print(
                f"   YIELD:   "
                f"{loser_agent.robot_id}"
            )

            print(
                f"   Reservation: "
                f"{reservation.entry_time:.2f}s"
                f" -> "
                f"{reservation.exit_time:.2f}s"
            )

            print(
                "-" * 72
            )

        # ----------------------------------------------------------
        # Generate robot actions
        # ----------------------------------------------------------

        actions = [
            agent.desired_velocity()
            for agent in agents
        ]

        # ----------------------------------------------------------
        # Execute NEXUS commands
        # ----------------------------------------------------------

        env.step(actions)

        # ----------------------------------------------------------
        # Diagnostics
        # ----------------------------------------------------------

        if step % 20 == 0:

            print(
                f"[T={simulation_time:5.1f}s] "
                + " | ".join(
                    [
                        (
                            f"{agent.robot_id}:"
                            f"{agent.intent}"
                        )
                        for agent in agents
                    ]
                )
            )

        # ----------------------------------------------------------
        # Render
        # ----------------------------------------------------------

        if step % 2 == 0:

            env.render(0.001)

            # Draw actual trajectories.
            for agent in agents:

                trajectory = np.asarray(
                    trajectories[
                        agent.robot_id
                    ]
                )

                if len(trajectory) < 2:
                    continue

                plt.plot(
                    trajectory[:, 0],
                    trajectory[:, 1],
                    linewidth=1.5,
                    alpha=0.7,
                )

            plt.pause(0.001)

        # ----------------------------------------------------------
        # Completion
        # ----------------------------------------------------------

        if all(
            agent.intent == "ARRIVED"
            for agent in agents
        ):

            print()
            print("=" * 72)
            print("ALL ROBOTS ARRIVED")
            print("=" * 72)

            break

        simulation_time += 0.1

    else:

        print()
        print("=" * 72)
        print("SIMULATION TIMEOUT")
        print("=" * 72)

        # ==============================================================
    # FINAL REPORT
    # ==============================================================

    print()
    print("=" * 72)
    print("FINAL COORDINATION REPORT")
    print("=" * 72)

    for agent in agents:

        goal_error = np.linalg.norm(
            agent.state.position
            - agent.state.goal
        )

        print(
            f"{agent.robot_id}: "
            f"goal_error="
            f"{goal_error:.2f} m | "
            f"intent="
            f"{agent.intent}"
        )

    print()

    print(
        f"Active negotiations: "
        f"{len(active_negotiations)}"
    )

    print(
        f"Reservations stored: "
        f"{len(negotiation.reservations.reservations)}"
    )

    env.end()


if __name__ == "__main__":
    main()
