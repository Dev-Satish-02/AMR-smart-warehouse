import numpy as np
import matplotlib.pyplot as plt
import irsim

from algorithms.planner import NEXUSPlanner
from algorithms.conflict_detector import ConflictDetector
from algorithms.negotiation import NegotiationManager
from agents.robot_agent import RobotAgent
from communication.p2p import P2PNetwork


# ==========================================================================
# ROBOT CONFIGURATION
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
# COORDINATION PARAMETERS
# ==========================================================================

# Once the priority robot is this far beyond the conflict point,
# release the stopped robot.

CLEARANCE_DISTANCE = 1.5

# Hard physical safety stop.
#
# This is independent of prediction/negotiation.
# If two robots actually get this close, both stop.
#
# We intentionally keep this separate from the normal STOP/GO logic.

HARD_SAFETY_DISTANCE = 0.72


# ==========================================================================
# MAIN
# ==========================================================================

def main():

    print("=" * 72)

    print(
        "                 NEXUS SIMPLE COORDINATION"
    )

    print("=" * 72)

    # ======================================================================
    # ENVIRONMENT
    # ======================================================================

    env = irsim.make(
        "configs/warehouse.yaml"
    )

    # ======================================================================
    # NEXUS MODULES
    # ======================================================================

    planner = NEXUSPlanner()

    detector = ConflictDetector(
        prediction_horizon=3.0,
        prediction_dt=0.2,
        safety_distance=0.95,
    )

    negotiation = NegotiationManager(
        eta_margin=0.25
    )

    network = P2PNetwork()

    # ======================================================================
    # ROBOT AGENTS
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

        agents.append(agent)

        network.register(agent)

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
        "Planning: A*"
    )

    print(
        "Conflict prediction: 3.0 s horizon"
    )

    print(
        "Coordination: STOP / GO"
    )

    print(
        "Priority: ETA + robot-ID tie-break"
    )

    print(
        f"Clearance distance: "
        f"{CLEARANCE_DISTANCE:.2f} m"
    )

    print(
        f"Hard safety distance: "
        f"{HARD_SAFETY_DISTANCE:.2f} m"
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
    # ACTIVE CONFLICTS
    #
    # pair -> information about who has priority and where the conflict is.
    #
    # Example:
    #
    # ("R1", "R4") -> {
    #     "winner": R1,
    #     "loser": R4,
    #     "position": [10, 17]
    # }
    # ======================================================================

    active_conflicts = {}

    # ======================================================================
    # METRICS
    # ======================================================================

    collision_count = 0

    conflict_count = 0

    stop_events = 0

    # ======================================================================
    # SIMULATION
    # ======================================================================

    simulation_time = 0.0

    max_steps = 600

    for step in range(
        max_steps
    ):

        # ------------------------------------------------------------------
        # Update state.
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
        # P2P broadcast.
        # ------------------------------------------------------------------

        network.broadcast_all()

        # ------------------------------------------------------------------
        # Release completed conflicts.
        #
        # The winner must physically move beyond the conflict region.
        # ------------------------------------------------------------------

        for pair in list(
            active_conflicts.keys()
        ):

            info = (
                active_conflicts[pair]
            )

            winner = info["winner"]
            loser = info["loser"]

            conflict_position = (
                info["position"]
            )

            winner_distance = np.linalg.norm(
                winner.state.position
                - conflict_position
            )

            # --------------------------------------------------------------
            # Winner has cleared the region.
            # --------------------------------------------------------------

            if (
                winner_distance
                >= CLEARANCE_DISTANCE
            ):

                loser.resume()

                del active_conflicts[
                    pair
                ]

                print(
                    f"[T={simulation_time:5.1f}s] "
                    f"CONFLICT CLEARED: "
                    f"{winner.robot_id} passed "
                    f"{pair} -> "
                    f"{loser.robot_id} RESUMES"
                )

        # ------------------------------------------------------------------
        # Detect new predicted conflicts.
        # ------------------------------------------------------------------

        conflicts = detector.detect_all(
            agents
        )

        # ------------------------------------------------------------------
        # Process conflicts.
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
            # Already coordinated.
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
            # Ignore if either robot is already stopped for another
            # conflict.
            #
            # This prevents contradictory commands such as:
            #
            # R3 stops for R2
            # R3 immediately becomes winner against R6
            # R3 starts moving
            #
            # A robot already waiting simply remains waiting.
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
            # STOP LOSER
            # --------------------------------------------------------------

            loser_agent.stop(
                reason=(
                    f"WAIT_FOR_{winner_agent.robot_id}"
                )
            )

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
            stop_events += 1

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
                f"   Predicted TTC: "
                f"{conflict.time_to_conflict:.2f} s"
            )

            print(
                f"   PROCEED: "
                f"{winner_agent.robot_id}"
            )

            print(
                f"   STOP:    "
                f"{loser_agent.robot_id}"
            )

            print(
                f"   Reason: "
                f"{winner_decision.reason}"
            )

            print(
                "-" * 72
            )

        # ------------------------------------------------------------------
        # HARD SAFETY CHECK
        #
        # This is intentionally simple.
        #
        # If two robots actually get too close, stop both.
        #
        # This is a safety fallback, NOT the coordination algorithm.
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

                    # If they are already stopped due to coordination,
                    # don't count it as a new collision event.

                    if (
                        not agent_a.stopped
                        or not agent_b.stopped
                    ):

                        collision_count += 1

                    agent_a.stop(
                        reason="HARD_SAFETY"
                    )

                    agent_b.stop(
                        reason="HARD_SAFETY"
                    )

        # ------------------------------------------------------------------
        # Generate actions.
        # ------------------------------------------------------------------

        actions = [
            agent.desired_velocity()
            for agent in agents
        ]

        # ------------------------------------------------------------------
        # Execute.
        # ------------------------------------------------------------------

        env.step(
            actions
        )

        # ------------------------------------------------------------------
        # Status.
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
        # Render.
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

                if len(trajectory) < 2:

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
        # Completion.
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
        "FINAL COORDINATION REPORT"
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
        f"Stop events: "
        f"{stop_events}"
    )

    print(
        f"Safety violations: "
        f"{collision_count}"
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
