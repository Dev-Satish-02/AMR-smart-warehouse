from dataclasses import dataclass
from typing import Optional


@dataclass
class StateMessage:

    sender_id: int

    position: list
    velocity: list
    heading: float

    task_id: Optional[str]

    goal: Optional[list]

    planned_path: list

    intent: Optional[str]

    eta: Optional[float]

    timestamp: float
