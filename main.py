import irsim

from agents.robot_agent import RobotAgent
from communication.p2p import P2PNetwork


def main():

    # --------------------------------------------------
    # 1. Start simulation
    # --------------------------------------------------

    env = irsim.make("configs/warehouse.yaml")

    # --------------------------------------------------
    # 2. Create NEXUS agent for every AMR
    # --------------------------------------------------

    agents = []

    for robot_id, robot in enumerate(env.robot_list):

        agent = RobotAgent(
            robot_id=robot_id,
            robot=robot
        )

        agents.append(agent)

    # --------------------------------------------------
    # 3. Create decentralized P2P network
    # --------------------------------------------------

    network = P2PNetwork()

    for agent in agents:
        network.register(agent)

    print("\n========================================")
    print("         NEXUS INITIALIZED")
    print("========================================")

    print(f"Robots: {len(agents)}")
    print("Communication: P2P")
    print("Central controller: NONE")

    # --------------------------------------------------
    # 4. Simulation loop
    # --------------------------------------------------

    while not env.done():

        simulation_time = env.time

        # Update every robot's local state
        for agent in agents:

            agent.update(simulation_time)

            agent.set_intent("MOVING")

        # --------------------------------------------------
        # 5. P2P state exchange
        # --------------------------------------------------

        network.broadcast_all()

        # --------------------------------------------------
        # 6. Debug output every 10 simulation steps
        # --------------------------------------------------

        if int(simulation_time * 10) % 10 == 0:

            print(
                f"\n[T={simulation_time:.1f}s]"
            )

            for agent in agents:

                state = agent.state

                print(
                    f"R{agent.robot_id + 1}: "
                    f"pos=({state.position[0]:.2f}, "
                    f"{state.position[1]:.2f}) "
                    f"peers={len(agent.peer_states)} "
                    f"intent={state.intent}"
                )

        # --------------------------------------------------
        # 7. Execute current simulator behavior
        # --------------------------------------------------

        env.step()

        env.render(0.01)

    env.end()


if __name__ == "__main__":
    main()
