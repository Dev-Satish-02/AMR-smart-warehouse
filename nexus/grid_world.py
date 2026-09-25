"""
Kinematic grid world: the built-in replacement for IR-SIM physics when
running on a lane layout.

Robots drive like rail-guided AGVs: straight along cell centres, braking to
a standstill before every turn, rotating in place, then accelerating again.
They never leave the segment between two cell centres, so paths are straight
lines joined by right-angle turns.

GridRobot exposes the same attributes RobotAgent reads from an IR-SIM robot
(`state` = [x, y, theta] and `velocity`), so the existing agent code is reused
unchanged.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

import numpy as np


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class GridRobot:

    def __init__(self, position: Sequence[float], heading: float = 0.0):
        self.state = np.array([float(position[0]), float(position[1]), float(heading)])
        self.velocity = np.zeros(2)
        self.speed = 0.0
        self.turning = False
        self.distance_travelled = 0.0

    @property
    def position(self) -> np.ndarray:
        return self.state[:2]

    @property
    def heading(self) -> float:
        return float(self.state[2])


class GridWorld:

    def __init__(
        self,
        max_speed: float = 1.0,
        acceleration: float = 1.2,
        angular_speed: float = 2.0,
        time_step: float = 0.1,
    ):
        self.max_speed = float(max_speed)
        self.acceleration = float(acceleration)
        self.angular_speed = float(angular_speed)
        self.time_step = float(time_step)
        self.time = 0.0
        self.robot_list: List[GridRobot] = []
        # Speed caps (safety zones) apply from this far before a capped
        # route point, i.e. from the edge of its cell.
        self.cap_lead = 0.5

    def add_robot(self, position, heading: float = 0.0) -> GridRobot:
        robot = GridRobot(position, heading)
        self.robot_list.append(robot)
        return robot

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    @staticmethod
    def _stop_distance(position: np.ndarray, route: List[np.ndarray]) -> float:
        """Distance along route until the first turn or the end of route."""
        total = 0.0
        previous = position
        direction: Optional[np.ndarray] = None
        for point in route:
            segment = point - previous
            length = float(np.linalg.norm(segment))
            if length < 1e-9:
                continue
            unit = segment / length
            if direction is not None and float(np.dot(unit, direction)) < 0.999:
                break
            direction = unit
            total += length
            previous = point
        return total

    def _cap_speed(self, position: np.ndarray, route: List[np.ndarray], caps: List[Optional[float]], limit: float) -> float:
        """Highest speed from which the robot can still slow to every capped
        route point's limit (by the edge of its cell) before reaching it."""
        allowed = limit
        travelled = 0.0
        previous = position
        for point, cap in zip(route, caps):
            travelled += float(np.linalg.norm(point - previous))
            previous = point
            if cap is None:
                continue
            distance = max(0.0, travelled - self.cap_lead)
            allowed = min(allowed, math.sqrt(cap * cap + 2.0 * self.acceleration * distance))
            if travelled > 4.0:  # far enough ahead
                break
        return allowed

    def _move(self, robot: GridRobot, route: List[np.ndarray], max_speed: float, dt: float,
              caps: Optional[List[Optional[float]]] = None) -> int:
        """Drive along route; returns how many route points were reached."""
        route = [np.asarray(p, dtype=float) for p in route]
        caps = list(caps) if caps is not None else [None] * len(route)
        total = len(route)
        time_left = dt
        robot.turning = False
        start = robot.position.copy()

        while time_left > 1e-9:
            # Drop waypoints we are already standing on.
            while route and np.linalg.norm(route[0] - robot.position) < 1e-6:
                route.pop(0)
                caps.pop(0)
            if not route:
                robot.speed = 0.0
                break

            delta = route[0] - robot.position
            desired = math.atan2(delta[1], delta[0])
            error = wrap_angle(desired - robot.heading)

            if abs(error) > 1e-3:
                # Rotate in place (the robot is at rest at a cell centre).
                robot.speed = 0.0
                robot.turning = True
                max_turn = self.angular_speed * time_left
                turn = max(-max_turn, min(max_turn, error))
                robot.state[2] = wrap_angle(robot.heading + turn)
                time_left -= abs(turn) / self.angular_speed
                continue

            robot.state[2] = desired
            stop_distance = self._stop_distance(robot.position, route)
            braking_speed = math.sqrt(max(0.0, 2.0 * self.acceleration * stop_distance))
            zone_speed = self._cap_speed(robot.position, route, caps, max_speed)
            speed = min(robot.speed + self.acceleration * time_left, max_speed, braking_speed, zone_speed)

            travel = min(speed * time_left, stop_distance)
            robot.speed = speed

            # Advance along collinear route points.
            remaining = travel
            while remaining > 1e-9 and route:
                segment = route[0] - robot.position
                length = float(np.linalg.norm(segment))
                if length <= remaining + 1e-9:
                    robot.state[0], robot.state[1] = route[0][0], route[0][1]
                    remaining -= length
                    route.pop(0)
                    caps.pop(0)
                    if route:
                        nxt = route[0] - robot.position
                        if np.linalg.norm(nxt) > 1e-9 and abs(wrap_angle(math.atan2(nxt[1], nxt[0]) - robot.heading)) > 1e-3:
                            break
                else:
                    step = segment / length * remaining
                    robot.state[0] += step[0]
                    robot.state[1] += step[1]
                    remaining = 0.0

            # One translation phase per step; a turn starts next step.
            time_left = 0.0
            if not route:
                robot.speed = 0.0

        moved = robot.position - start
        robot.distance_travelled += float(np.linalg.norm(moved))
        robot.velocity = moved / dt if dt > 0 else np.zeros(2)
        return total - len(route)

    def step(self, routes: Sequence[List[np.ndarray]], speeds: Optional[Sequence[float]] = None,
             caps: Optional[Sequence[List[Optional[float]]]] = None) -> List[int]:
        """
        routes[i]: world points robot i may drive through this step, in order,
        ending at the last cell it holds a reservation for.
        speeds[i]: speed limit right now (robot max, current safety zone).
        caps[i][k]: speed cap of the cell of routes[i][k] (None = no cap).
        Returns, per robot, the number of route points reached.
        """
        dt = self.time_step
        reached = []
        for index, robot in enumerate(self.robot_list):
            route = routes[index] if index < len(routes) else []
            limit = self.max_speed if speeds is None else min(self.max_speed, speeds[index])
            route_caps = caps[index] if caps is not None and index < len(caps) else None
            reached.append(self._move(robot, list(route), limit, dt, route_caps))
        self.time = round(self.time + dt, 9)
        return reached

    def done(self) -> bool:
        return False


__all__ = ["GridRobot", "GridWorld", "wrap_angle"]
