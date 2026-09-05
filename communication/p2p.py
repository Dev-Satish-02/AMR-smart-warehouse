class P2PNetwork:

    def __init__(self):
        self.robots = {}

    def register(self, robot_agent):
        self.robots[robot_agent.robot_id] = robot_agent

    def broadcast(self, sender_id):
        """
        Broadcast sender's state to every other robot.

        This simulates decentralized peer-to-peer
        state exchange.
        """

        sender = self.robots[sender_id]

        for robot_id, receiver in self.robots.items():

            if robot_id == sender_id:
                continue

            receiver.receive_peer_state(sender.state)

    def broadcast_all(self):

        for robot_id in self.robots:
            self.broadcast(robot_id)

    def get_network_state(self):

        return {
            robot_id: robot.state
            for robot_id, robot in self.robots.items()
        }
