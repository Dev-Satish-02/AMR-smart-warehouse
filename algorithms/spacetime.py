"""
Space-time path planning for NEXUS (decentralised, prioritised).

Every robot broadcasts its timed path (cell -> when it expects to be there,
see GridSimulation._timelines). A robot that needs a new route plans in
space AND time against its peers' broadcast plans:

    state      (cell, heading)
    cost       arrival time (seconds): driving + rotating in place + waiting
    conflict   being on a cell within `clearance` seconds of a peer
    waiting    allowed on the current cell while that cell stays free
               ("safe interval" search: the earliest safe departure only)

So a robot picks the route that actually arrives first: a parallel lane or
another aisle when the shortest one is busy, or a short planned wait when
that is quicker than a detour. Lane rules are the same as LaneGrid (one-way
lanes, stations are endpoints only, slow zones cost more, turns take time).
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

Cell = Tuple[int, int]
Interval = Tuple[float, float]

_DIRECTIONS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def merge(intervals: Sequence[Interval]) -> List[Interval]:
    """Sort and merge overlapping intervals."""
    out: List[Interval] = []
    for lo, hi in sorted(intervals):
        if out and lo <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], hi))
        else:
            out.append((lo, hi))
    return out


def blocked_at(intervals: Sequence[Interval], t: float) -> Optional[Interval]:
    for lo, hi in intervals:
        if lo > t:
            return None
        if t < hi:
            return (lo, hi)
    return None


def free_between(intervals: Sequence[Interval], t0: float, t1: float) -> bool:
    return all(hi <= t0 or lo >= t1 for lo, hi in intervals)


class SpaceTimePlanner:

    def __init__(self, layout, speed: float = 1.0, angular_speed: float = 2.0,
                 clearance: float = 1.6, max_wait: float = 30.0, max_expansions: int = 60000):
        self.layout = layout
        self.speed = float(speed)
        self.angular_speed = float(angular_speed)
        self.clearance = float(clearance)   # seconds either side of a peer's pass
        self.max_wait = float(max_wait)     # longest single planned wait
        self.max_expansions = max_expansions
        self.quarter_turn = (math.pi / 2.0) / self.angular_speed

    def step_time(self, cell: Cell) -> float:
        # Slow cells (crosswalks, slow zones) take longer to cross.
        return self.layout.cell_size * self.layout.cost_factor(cell) / self.speed

    def reserved_intervals(self, timelines: Dict[str, List[Tuple[Cell, float]]], exclude: str) -> Dict[Cell, List[Interval]]:
        """Peers' timed cells -> per-cell blocked intervals (merged)."""
        table: Dict[Cell, List[Interval]] = {}
        c = self.clearance
        for robot_id, timeline in timelines.items():
            if robot_id == exclude:
                continue
            for cell, t in timeline:
                table.setdefault(cell, []).append((t - c, t + c))
        return {cell: merge(iv) for cell, iv in table.items()}

    def plan(self, start: Cell, goal: Cell, t0: float, reserved: Dict[Cell, List[Interval]],
             start_heading: int = -1) -> Optional[List[Tuple[Cell, float]]]:
        """Timed path [(cell, time at its centre)] from start at t0 to goal,
        or None if unreachable. Waits show up as a time gap between cells."""
        if start == goal:
            return [(start, t0)]
        layout = self.layout
        gx, gy = goal
        per_cell = layout.cell_size / self.speed

        def h(cell: Cell) -> float:
            return (abs(cell[0] - gx) + abs(cell[1] - gy)) * per_cell

        start_state = (start, start_heading)
        best: Dict[Tuple[Cell, int], float] = {start_state: t0}
        parent: Dict[Tuple[Cell, int], Tuple[Tuple[Cell, int], float]] = {}
        heap = [(t0 + h(start), 0, t0, start_state)]
        counter = 1
        expansions = 0
        while heap:
            _, _, t, state = heapq.heappop(heap)
            if t > best.get(state, math.inf) + 1e-9:
                continue
            cell, heading = state
            if cell == goal:
                path = [(cell, t)]
                while state in parent:
                    state, t_prev = parent[state]
                    path.append((state[0], t_prev))
                path.reverse()
                return path
            expansions += 1
            if expansions > self.max_expansions:
                return None
            if cell != start and layout.is_station(cell):
                continue  # stations are endpoints only
            here = reserved.get(cell, ())
            for index, (dx, dy) in enumerate(_DIRECTIONS):
                nxt = (cell[0] + dx, cell[1] + dy)
                if not layout.move_allowed(cell, nxt):
                    continue
                if layout.is_station(nxt) and nxt != goal:
                    continue
                turn = 0.0
                if heading >= 0 and index != heading:
                    turn = self.quarter_turn * (2 if (index - heading) % 4 == 2 else 1)
                travel = turn + self.step_time(nxt)
                there = reserved.get(nxt, ())
                # Earliest departure >= t that reaches nxt outside its blocked
                # intervals while we can keep waiting where we are.
                depart = t
                ok = False
                for _ in range(8):
                    arrive = depart + travel
                    clash = blocked_at(there, arrive)
                    if clash is None:
                        ok = free_between(here, t, depart + travel * 0.5) if depart > t else True
                        break
                    depart = clash[1] - travel + 1e-3
                    if depart - t > self.max_wait:
                        break
                if not ok:
                    continue
                arrive = depart + travel
                nstate = (nxt, index)
                if arrive < best.get(nstate, math.inf) - 1e-9:
                    best[nstate] = arrive
                    parent[nstate] = (state, t)
                    heapq.heappush(heap, (arrive + h(nxt), counter, arrive, nstate))
                    counter += 1
        return None

    def time_path(self, cells: Sequence[Cell], t0: float, reserved: Dict[Cell, List[Interval]],
                  start_heading: int = -1) -> List[Tuple[Cell, float]]:
        """Arrival times along a fixed cell path under the same rules
        (waiting where needed): how long the current route would take."""
        t, heading = t0, start_heading
        out = [(cells[0], t0)] if cells else []
        for a, b in zip(cells, cells[1:]):
            step = (b[0] - a[0], b[1] - a[1])
            index = _DIRECTIONS.index(step) if step in _DIRECTIONS else heading
            turn = 0.0
            if heading >= 0 and index != heading:
                turn = self.quarter_turn * (2 if (index - heading) % 4 == 2 else 1)
            travel = turn + self.step_time(b)
            there = reserved.get(b, ())
            depart = t
            for _ in range(8):
                clash = blocked_at(there, depart + travel)
                if clash is None:
                    break
                depart = clash[1] - travel + 1e-3
            t, heading = depart + travel, index
            out.append((b, t))
        return out
