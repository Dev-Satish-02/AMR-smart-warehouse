from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class WarehouseConfig:
    name: str = "Warehouse A"
    width: float = 30.0
    height: float = 20.0
    resolution: float = 0.5
    obstacles: List[Dict[str, Any]] = field(default_factory=list)
    stations: List[Dict[str, Any]] = field(default_factory=list)
    charging_stations: List[Dict[str, Any]] = field(default_factory=list)
    path: str = "configs/warehouse.yaml"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "WarehouseConfig":
        return cls(**payload)


@dataclass
class RobotConfig:
    id: str = "R1"
    start: List[float] = field(default_factory=lambda: [0.0, 0.0])
    goal: List[float] = field(default_factory=lambda: [0.0, 0.0])
    heading: float = 0.0
    max_speed: float = 0.8
    battery: float = 100.0
    task_id: Optional[str] = None
    robot_type: str = "AMR"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "RobotConfig":
        return cls(**payload)


@dataclass
class TaskConfig:
    id: str = "T1"
    pickup: List[float] = field(default_factory=lambda: [0.0, 0.0])
    dropoff: List[float] = field(default_factory=lambda: [0.0, 0.0])
    priority: int = 1
    assigned_robot: Optional[str] = None
    status: str = "PENDING"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "TaskConfig":
        return cls(**payload)


@dataclass
class SimulationConfig:
    warehouse: WarehouseConfig = field(default_factory=WarehouseConfig)
    robots: List[RobotConfig] = field(default_factory=list)
    tasks: List[TaskConfig] = field(default_factory=list)
    simulation: Dict[str, Any] = field(
        default_factory=lambda: {
            "time_step": 0.1,
            "max_steps": 900,
            "clearance_distance": 1.5,
            "dynamic_obstacle_radius": 1.0,
            "hard_safety_distance": 0.70,
            "strategy": "nexus",
        }
    )

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        payload["warehouse"] = self.warehouse.to_dict()
        payload["robots"] = [robot.to_dict() for robot in self.robots]
        payload["tasks"] = [task.to_dict() for task in self.tasks]
        return payload

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "SimulationConfig":
        warehouse = WarehouseConfig.from_dict(payload.get("warehouse", {}))
        robots = [RobotConfig.from_dict(item) for item in payload.get("robots", [])]
        tasks = [TaskConfig.from_dict(item) for item in payload.get("tasks", [])]
        return cls(
            warehouse=warehouse,
            robots=robots,
            tasks=tasks,
            simulation=payload.get("simulation", {}),
        )


@dataclass
class SimulationMetrics:
    robots: int = 0
    active_tasks: int = 0
    completed_tasks: int = 0
    robots_moving: int = 0
    robots_waiting: int = 0
    robots_rerouting: int = 0
    conflicts_detected: int = 0
    safety_violations: int = 0
    simulation_time: float = 0.0
    throughput: float = 0.0
    status: str = "READY"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "SimulationMetrics":
        return cls(**payload)


__all__ = [
    "WarehouseConfig",
    "RobotConfig",
    "TaskConfig",
    "SimulationConfig",
    "SimulationMetrics",
]
