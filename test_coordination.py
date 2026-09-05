import numpy as np
import matplotlib.pyplot as plt
import irsim

from algorithms.planner import (
    NEXUSPlanner,
)

from algorithms.conflict_detector import (
    ConflictDetector,
)

from algorithms.negotiation import (
    NegotiationManager,
)

from agents.robot_agent import (
    RobotAgent,
)

from communication.p2p import (
    P2PNetwork,
)


# ==========================================================================
# ROBOTS
# ==========================================================================

ROBOT_CONFIG = [

    (
        "R1",
        np.array([2.0, 3.0]),
        np.array([28.0, 17.0]),
    ),

    (
        "R2",
        np.array([2.0, 7.0]),
        np.array([28.0, 13.0]),
    ),

    (
        "R3",
        np.array([2.0, 11.0]),
        np.array([28.0, 9.0]),
    ),

    (
        "R4",
        np.array([28.0, 17.0]),
        np.array([2.0, 3.0]),
    ),

    (
        "R5",
        np.array([28.0, 13.0]),
        np.array([2.0, 7.0]),
    ),

    (
        "R6",
        np.array([28.0, 9.0]),
        np.array([2.0, 11.0]),
    ),
]


# ==========================================================================
# PARAMETERS
# ==========================================================================

# How far the winner must travel beyond the original conflict point
# before the stopped robot is released.

CLEARANCE_DISTANCE = 1.5

# Temporary obstacle around a stopped robot used by A*.

DYNAMIC_OBSTACLE_RADIUS = 1.0

# Emergency physical safety distance.

HARD_SAFETY_DISTANCE = 0.70


# ==========================================================================
# MAIN
# ==========================================================================

def main():

    print("=" * 72)

    print(
        "                 NEXUS COORDINATION"
    )

    print("=" * 72)

    # ======================================================================
    # ENVIRONMENT
    # ======================================================================

    env = irsim.make(
        "configs/warehouse.yaml"
    )

    # ======================================================================
    # MODULES
    # ======================================================================

    planner = NEXUSPlanner()

    detector = ConflictDetector(
        prediction_horizon=3.0,
        prediction_dt=0.2,
        safety_distance=1.40,
    )

    negotiation = NegotiationManager(
        eta_margin=0.25
    )

    network = P2PNetwork()

    # ======================================================================
    # CREATE ROBOTS
    # ======================================================================

    agents = []

    for i, (
        robot_id,
        start,
        goal,
    ) in enumerate(
        ROBOT_CONFIG
    ):

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

        agents.append(
            agent
        )

        network.register(
            agent
        )

        print(
            f"{robot_id}: "
            f"{len(agent.planned_path)} waypoints | "
            f"{agent.path_length():.2f} m"
        )

    print()

    print(
        "Communication: P2P"
    )

    print(
        "Global planning: A*"
    )

    print(
        "Conflict prediction: 3.0 s"
    )

    print(
        "Conflict threshold: 1.40 m"
    )

    print(
        "Coordination: STOP + dynamic A* reroute"
    )

    print()

    # ======================================================================
    # TRAJECTORIES
    # ======================================================================

    trajectories = {
        agent.robot_id: []
        for agent in agents
    }

    # ======================================================================
    # ACTIVE COORDINATION
    #
    # pair -> {
    #     winner,
    #     loser,
    #     conflict_position
    # }
    # ======================================================================

    active_conflicts = {}

    # ======================================================================
    # METRICS
    # ======================================================================

    conflict_count = 0

    reroute_count = 0

    stop_count = 0

    safety_violations = 0

    # ======================================================================
    # SIMULATION
    # ======================================================================

    simulation_time = 0.0

    max_steps = 900

    for step in range(
        max_steps
    ):

        # ------------------------------------------------------------------
        # UPDATE
        # ------------------------------------------------------------------

        for agent in agents:

            agent.update(
                simulation_time
            )

            trajectories[
                agent.robot_id
            ].append(
                agent.state.position.copy()
            )

        # ------------------------------------------------------------------
        # P2P
        # ------------------------------------------------------------------

        network.broadcast_all()

        # ------------------------------------------------------------------
        # RELEASE STOPPED ROBOTS
        # ------------------------------------------------------------------

        for pair in list(
            active_conflicts.keys()
        ):

            info = (
                active_conflicts[pair]
            )

            winner = info[
                "winner"
            ]

            loser = info[
                "loser"
            ]

            conflict_position = info[
                "position"
            ]

            distance_from_conflict = (
                np.linalg.norm(
                    winner.state.position
                    - conflict_position
                )
            )

            # --------------------------------------------------------------
            # Only release the loser after the winner has physically
            # cleared the conflict area.
            # --------------------------------------------------------------

            if (
                distance_from_conflict
                >= CLEARANCE_DISTANCE
                and not winner.stopped
            ):

                loser.resume()

                print(
                    f"[T={simulation_time:5.1f}s] "
                    f"{winner.robot_id} cleared "
                    f"conflict -> "
                    f"{loser.robot_id} RESUMES"
                )

                del active_conflicts[
                    pair
                ]

        # ------------------------------------------------------------------
        # PREDICT CONFLICTS
        # ------------------------------------------------------------------

        conflicts = detector.detect_all(
            agents
        )

        # ------------------------------------------------------------------
        # PROCESS CONFLICTS
        # ------------------------------------------------------------------

        for conflict in conflicts:

            pair = tuple(
                sorted(
                    [
                        conflict.robot_a,
                        conflict.robot_b,
                    ]
                )
            )

            # --------------------------------------------------------------
            # Already being handled.
            # --------------------------------------------------------------

            if pair in active_conflicts:

                continue

            # --------------------------------------------------------------
            # Find agents.
            # --------------------------------------------------------------

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

            # --------------------------------------------------------------
            # If either robot is already waiting for another robot,
            # don't create another negotiation involving it.
            # --------------------------------------------------------------

            if (
                agent_a.stopped
                or agent_b.stopped
            ):

                continue

            # --------------------------------------------------------------
            # NEGOTIATE
            # --------------------------------------------------------------

            (
                winner_decision,
                loser_decision,
            ) = negotiation.negotiate(
                agent_a,
                agent_b,
                conflict,
            )

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

            # --------------------------------------------------------------
            # STOP LOSER FIRST.
            # --------------------------------------------------------------

            loser_agent.stop(
                reason=(
                    f"WAIT_FOR_"
                    f"{winner_agent.robot_id}"
                )
            )

            # --------------------------------------------------------------
            # NOW REPLAN WINNER AROUND LOSER.
            #
            # This is the important new behavior.
            # --------------------------------------------------------------

            rerouted = (
                winner_agent.replan_around(
                    obstacle_position=(
                        loser_agent.state.position
                    ),
                    obstacle_radius=(
                        DYNAMIC_OBSTACLE_RADIUS
                    ),
                )
            )

            # --------------------------------------------------------------
            # If A* cannot find a safe alternative, the winner stops too.
            #
            # This is much safer than allowing it to drive straight into
            # the other robot.
            # --------------------------------------------------------------

            if not rerouted:

                winner_agent.stop(
                    reason="NO_SAFE_REROUTE"
                )

                print(
                    f"[T={simulation_time:5.1f}s] "
                    f"NO SAFE REROUTE: "
                    f"{winner_agent.robot_id} "
                    f"also stopped"
                )

                continue

            # --------------------------------------------------------------
            # Register active conflict.
            # --------------------------------------------------------------

            active_conflicts[
                pair
            ] = {
                "winner": winner_agent,
                "loser": loser_agent,
                "position": (
                    conflict.conflict_position.copy()
                ),
            }

            conflict_count += 1

            reroute_count += 1

            stop_count += 1

            # --------------------------------------------------------------
            # Diagnostics.
            # --------------------------------------------------------------

            print()

            print(
                "-" * 72
            )

            print(
                f"[T={simulation_time:5.1f}s] "
                f"CONFLICT"
            )

            print(
                f"   Robots: "
                f"{conflict.robot_a} <-> "
                f"{conflict.robot_b}"
            )

            print(
                f"   Conflict point: "
                f"("
                f"{conflict.conflict_position[0]:.2f}, "
                f"{conflict.conflict_position[1]:.2f}"
                f")"
            )

            print(
                f"   Predicted distance: "
                f"{conflict.minimum_distance:.2f} m"
            )

            print(
                f"   TTC: "
                f"{conflict.time_to_conflict:.2f} s"
            )

            print(
                f"   PROCEED + REROUTE: "
                f"{winner_agent.robot_id}"
            )

            print(
                f"   STOP: "
                f"{loser_agent.robot_id}"
            )

            print(
                f"   New path: "
                f"{len(winner_agent.planned_path)} "
                f"waypoints"
            )

            print(
                "-" * 72
            )

        # ------------------------------------------------------------------
        # HARD SAFETY CHECK
        # ------------------------------------------------------------------

        for i in range(
            len(agents)
        ):

            for j in range(
                i + 1,
                len(agents),
            ):

                agent_a = agents[i]
                agent_b = agents[j]

                distance = np.linalg.norm(
                    agent_a.state.position
                    - agent_b.state.position
                )

                if (
                    distance
                    < HARD_SAFETY_DISTANCE
                ):

                    safety_violations += 1

                    agent_a.stop(
                        reason="HARD_SAFETY"
                    )

                    agent_b.stop(
                        reason="HARD_SAFETY"
                    )

        # ------------------------------------------------------------------
        # ACTIONS
        # ------------------------------------------------------------------

        actions = [
            agent.desired_velocity()
            for agent in agents
        ]

        # ------------------------------------------------------------------
        # SIMULATION STEP
        # ------------------------------------------------------------------

        env.step(
            actions
        )

        # ------------------------------------------------------------------
        # STATUS
        # ------------------------------------------------------------------

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

        # ------------------------------------------------------------------
        # RENDER
        # ------------------------------------------------------------------

        if step % 2 == 0:

            env.render(
                0.001
            )

            for agent in agents:

                trajectory = np.asarray(
                    trajectories[
                        agent.robot_id
                    ]
                )

                if len(
                    trajectory
                ) < 2:

                    continue

                plt.plot(
                    trajectory[:, 0],
                    trajectory[:, 1],
                    linewidth=1.5,
                    alpha=0.7,
                )

            plt.pause(
                0.001
            )

        # ------------------------------------------------------------------
        # COMPLETE
        # ------------------------------------------------------------------

        if all(
            agent.intent == "ARRIVED"
            for agent in agents
        ):

            print()

            print(
                "=" * 72
            )

            print(
                "ALL ROBOTS ARRIVED"
            )

            print(
                "=" * 72
            )

            break

        simulation_time += 0.1

    else:

        print()

        print(
            "=" * 72
        )

        print(
            "SIMULATION TIMEOUT"
        )

        print(
            "=" * 72
        )

    # ======================================================================
    # FINAL REPORT
    # ======================================================================

    print()

    print(
        "=" * 72
    )

    print(
        "FINAL NEXUS REPORT"
    )

    print(
        "=" * 72
    )

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
        f"Conflicts handled: "
        f"{conflict_count}"
    )

    print(
        f"Dynamic reroutes: "
        f"{reroute_count}"
    )

    print(
        f"Stop events: "
        f"{stop_count}"
    )

    print(
        f"Safety violations: "
        f"{safety_violations}"
    )

    print(
        f"Active conflicts: "
        f"{len(active_conflicts)}"
    )

    env.end()


# ==========================================================================
# ENTRY POINT
# ==========================================================================

if __name__ == "__main__":

    main()
