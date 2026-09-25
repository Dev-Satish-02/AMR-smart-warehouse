# Graph Report - nexus  (2026-09-24)

## Corpus Check
- Corpus is ~10,728 words - fits in a single context window. You may not need a graph.

## Summary
- 221 nodes · 460 edges · 12 communities (9 shown, 3 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 19 edges (avg confidence: 0.88)
- Token cost: 0 input · 32,766 output

## Community Hubs (Navigation)
- Conflict Detection & Negotiation
- A* Grid Planning
- Warehouse Configs & Deps
- Robot Agent Behavior
- Dashboard API Server
- Simulation Manager
- Time-Space Reservations
- Config Data Models
- YAML/JSON Persistence

## God Nodes (most connected - your core abstractions)
1. `RobotAgent` - 37 edges
2. `SimulationManager` - 32 edges
3. `NEXUSPlanner` - 20 edges
4. `P2PNetwork` - 17 edges
5. `ConflictDetector` - 14 edges
6. `NegotiationManager` - 13 edges
7. `WarehouseGrid` - 12 edges
8. `ReservationManager` - 11 edges
9. `save_config()` - 10 edges
10. `load_config()` - 6 edges

## Surprising Connections (you probably didn't know these)
- `SimulationManager` --uses--> `RobotAgent`  [INFERRED]
  nexus/simulation_manager.py → agents/robot_agent.py
- `SimulationManager` --uses--> `ConflictDetector`  [INFERRED]
  nexus/simulation_manager.py → algorithms/conflict_detector.py
- `SimulationManager` --uses--> `NegotiationManager`  [INFERRED]
  nexus/simulation_manager.py → algorithms/negotiation.py
- `SimulationManager` --uses--> `NEXUSPlanner`  [INFERRED]
  nexus/simulation_manager.py → algorithms/planner.py
- `SimulationManager` --uses--> `P2PNetwork`  [INFERRED]
  nexus/simulation_manager.py → communication/p2p.py

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Warehouse shelf/obstacle layout** — warehouse_yaml_upper_left_shelf, warehouse_yaml_upper_right_shelf, warehouse_yaml_lower_left_shelf, warehouse_yaml_lower_right_shelf, warehouse_yaml_central_chokepoint [EXTRACTED]
- **NEXUS Python dependencies** — requirements_ir_sim, requirements_pyrvo, requirements_numpy, requirements_scipy, requirements_matplotlib, requirements_pyyaml, requirements_pandas [EXTRACTED]

## Communities (12 total, 3 thin omitted)

### Community 0 - "Conflict Detection & Negotiation"
Cohesion: 0.12
Nodes (18): ConflictDetector, PredictedConflict, NegotiationDecision, NegotiationManager, Simple peer-to-peer priority negotiation. Earlier ETA wins. If ETAs are…, StateMessage, P2PNetwork, Broadcast sender's state to every other robot. This simulates decentralized… (+10 more)

### Community 1 - "A* Grid Planning"
Cohesion: 0.14
Nodes (11): GridConfig, WarehouseGrid, NEXUSPlanner, heapq, math, matplotlib_pyplot, main(), draw_warehouse() (+3 more)

### Community 2 - "Warehouse Configs & Deps"
Cohesion: 0.10
Nodes (25): demo_warehouse.yaml (2-robot head-on demo config), warehouse.yaml (main 6-robot warehouse config), Head-on swap: robots (2,10)<->(28,10) along y=10, Empty obstacle list, Demo robot group: 2 diff-drive robots, Demo world: 30x20 grid, step_time 0.1, IR-SIM YAML world/robot/obstacle schema, ir-sim (+17 more)

### Community 3 - "Robot Agent Behavior"
Cohesion: 0.12
Nodes (4): Treat a stopped peer as a temporary obstacle and generate a new A* path around…, NEXUS AMR agent. Architecture: P2P world model ↓ A* global planning ↓ conflict…, RobotAgent, RobotState

### Community 4 - "Dashboard API Server"
Cohesion: 0.13
Nodes (22): BaseModel, ConfigPayload, dashboard_page(), ensure_stream(), get_config(), get_state(), load_default_config(), pause_simulation() (+14 more)

### Community 5 - "Simulation Manager"
Cohesion: 0.22
Nodes (3): Any, Reusable simulation shell around the existing NEXUS engine. The dashboard uses…, SimulationManager

### Community 6 - "Time-Space Reservations"
Cohesion: 0.12
Nodes (11): Create an absolute-time reservation. current_time: Current simulation time.…, Return reservations that have not expired., Remove expired reservations., Check whether the resource is available during [entry_time, exit_time]., NEXUS time-space reservation manager. Reservations use ABSOLUTE simulation…, Extract numerical portion from IDs such as R1, R2, R10., Lower ETA wins. Robot ID provides deterministic tie-breaking., Time-space reservation for a shared warehouse resource. (+3 more)

### Community 7 - "Config Data Models"
Cohesion: 0.22
Nodes (6): Any, RobotConfig, SimulationConfig, SimulationMetrics, TaskConfig, WarehouseConfig

### Community 8 - "YAML/JSON Persistence"
Cohesion: 0.31
Nodes (13): json, ensure_parent(), load_config(), load_json(), load_yaml(), Any, save_config(), save_json() (+5 more)

## Knowledge Gaps
- **12 isolated node(s):** `StateMessage`, `Upper horizontal shelf (4,5)-(11,5)-(11,8)`, `Upper-right shelf (15,5)-(22,5)-(22,8)`, `Lower-left shelf (4,12)-(11,12)-(11,15)`, `Lower-right shelf (15,12)-(22,12)-(22,15)` (+7 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 53 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **3 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `SimulationManager` connect `Simulation Manager` to `Conflict Detection & Negotiation`, `A* Grid Planning`, `Robot Agent Behavior`, `Dashboard API Server`, `Config Data Models`, `YAML/JSON Persistence`?**
  _High betweenness centrality (0.230) - this node is a cross-community bridge._
- **Why does `RobotAgent` connect `Robot Agent Behavior` to `Conflict Detection & Negotiation`, `A* Grid Planning`, `Simulation Manager`?**
  _High betweenness centrality (0.188) - this node is a cross-community bridge._
- **Why does `ReservationManager` connect `Time-Space Reservations` to `Conflict Detection & Negotiation`?**
  _High betweenness centrality (0.117) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `SimulationManager` (e.g. with `load_default_config()` and `RobotAgent`) actually correct?**
  _`SimulationManager` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `NEXUSPlanner` (e.g. with `GridConfig` and `WarehouseGrid`) actually correct?**
  _`NEXUSPlanner` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `StateMessage`, `Upper horizontal shelf (4,5)-(11,5)-(11,8)`, `Upper-right shelf (15,5)-(22,5)-(22,8)` to the rest of the system?**
  _12 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Conflict Detection & Negotiation` be split into smaller, more focused modules?**
  _Cohesion score 0.11829268292682926 - nodes in this community are weakly interconnected._