# NEXUS: Decentralized AMR Fleet Coordination

**Team A\*** · Smart India Hackathon 2026 · Problem Statement **SIH26123**: *Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses*

NEXUS coordinates a fleet of warehouse robots **without a central traffic controller**. Every robot runs the same agent. It shares its position and intent with its neighbours peer-to-peer, plans its own route in space and time, and negotiates right of way directly with the robots it meets. Where a classical system makes a robot stop and wait, a NEXUS robot slows down and crosses just behind the other one.

The project includes a lane-based multi-robot warehouse simulator, a live web fleet dashboard (the **NEXUS Control Room**), a visual layout editor, and a benchmark harness that compares NEXUS with traditional stop-and-wait traffic control.

## Demo video

[![NEXUS demo video](https://img.youtube.com/vi/oSFoE7Ld5Dc/maxresdefault.jpg)](https://www.youtube.com/watch?v=oSFoE7Ld5Dc)

▶ **[Watch the demo on YouTube](https://www.youtube.com/watch?v=oSFoE7Ld5Dc)**

---

## Results

Same layouts, same robots, same order lists (3 random seeds per scenario). Only the traffic control differs.

| Scenario | Zone lock (classical) | Stop & wait | **NEXUS** | NEXUS vs zone lock |
|---|---|---|---|---|
| 4-way intersection, 8 AMRs | 381 s | 353 s | **332 s** | +13 % |
| 4-way intersection, 12 AMRs | 476 s | 403 s | **362 s** | +24 % |
| 4-way intersection, 16 AMRs | 598 s | 496 s | **428 s** | +28 % |
| City grid, 16 intersections, 16 AMRs | 405 s | 368 s | **332 s** | +18 % |
| Single-lane corridor + passing bay | 1247 s | 434 s | **433 s** | +65 % |
| Rack aisles with crossings | 582 s | 400 s | **363 s** | +38 % |
| Head-on swap | 72 s | 34 s | **33 s** | +54 % |
| Cross traffic, 6 robots | 172 s | 49 s | **46 s** | +74 % |
| BEL warehouse batch, 40 orders | 890 s | 778 s | **763 s** | +14 % |

- **35.9 % less total task completion time** than classical zone-lock stop-and-wait (6 of 9 scenarios above 20 %)
- **66 % fewer full stops** in traffic
- **Zero inter-robot collisions** and zero stuck runs, for every strategy in every run
- The advantage **grows with fleet density**

---

## Features

### Coordination (per robot, no central controller)
- **Peer-to-peer state and intent exchange:** position, speed, battery, task, planned path with timing, and the cells being claimed.
- **Space-time path planning:** A* over the lane graph, then a safe-interval search around the timed paths peers have broadcast. Robots re-check every few seconds and switch to a quieter route when it arrives sooner.
- **Conflict prediction:** looks a few seconds ahead at intersections, corridors and head-on encounters.
- **Peer-to-peer negotiation:** the first to arrive goes first, and a robot already on the other's path always goes first. The crossing order never creates a loop of robots waiting on each other.
- **Flow-through crossing:** the robot that gives way slows to arrive just as the other clears, instead of stopping.
- **Cost-aware lane change and re-routing:** a detour is taken only when it beats the expected wait.
- **Deadlock resolution:** loops of robots waiting on each other at choke points are detected, and one robot backs off into a side cell.
- **Safety gate:** robots enter only cells they have claimed and never stop inside an intersection. Robots closer than 0.7 m raise an alarm.
- **Human-aware floors:** robots never enter walkways, slow to 0.3 m/s on crosswalks and to 0.5 m/s in slow zones.

### Fleet management
- **Transport orders:** created from mission flows or by an operator.
- **Auction dispatch:** each free robot bids its travel time, and a robot bids only if its battery covers the whole trip.
- **Batteries:** drain with distance and idle time; robots charge automatically. An order goes back to the auction when a robot must charge.
- **Operator controls:** fleet-wide E-stop, per-robot hold and release, charge now.

### NEXUS Control Room (web dashboard)
| Page | What it shows |
|---|---|
| **Overview** | Live map (robots, routes, claimed cells, predicted conflicts), KPIs, fleet list with battery, missions, coordination log, playback at 0.5–8×, coordination strategy switch |
| **Missions** | Order queue, status filters, lead times, mission flows, new orders |
| **Fleet** | One card per robot: battery, status, task, distance, time breakdown, and hold / charge / locate |
| **Alerts** | Active and historical alerts (battery, blocked, charger wait, backlog, safety, deadlock, E-stop) |
| **Reports** | Shift report with throughput, lead times, utilisation and traffic losses; CSV export, print to PDF |
| **Benchmark** | NEXUS vs zone lock vs stop & wait, with a live replay of any scenario under any strategy |
| **Layout Editor** | Draw lanes, junctions, racks, walls, walkways, stations, equipment, safety zones, robots, routes and mission flows; validate; run in Live |

---

## Quick start

Requirements: Python 3.12, and Node.js only if you want to run the frontend tests.

```bash
git clone git@github.com:Dev-Satish-02/AMR-smart-warehouse.git
cd AMR-smart-warehouse
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt

python -m nexus_gui                 # → http://127.0.0.1:8000
```

Options: `python -m nexus_gui --host 0.0.0.0 --port 8080`

### A 2-minute tour
1. **Overview:** pick **BEL Warehouse (Prototype)** in the *Layout* dropdown and press **Play**. Click a robot to follow it.
2. Turn on *Planned paths*, *Cell reservations* and *Conflicts* in the layers menu (stack icon).
3. Load **Cross-Traffic (6 robots)** and watch the coordination log for *"slows to cross behind it"* and *"changes lane"*.
4. **Benchmark** tab: open any scenario row and press **Zone lock**, then **NEXUS**, to replay the same orders under each strategy.
5. **Layout Editor:** draw your own warehouse and press **Run in Live**.

---

## Benchmark from the command line

```bash
python -m nexus.benchmark                    # all scenarios, 3 seeds (≈4 min on 12 cores)
python -m nexus.benchmark --quick            # 1 seed, skips the heavy scenarios
python -m nexus.benchmark --scenarios city_grid corridor --seeds 5
```

Results print as a table and are saved to `data/benchmark/latest.json`, which the Benchmark page reads.

**Strategies** (`simulation.strategy` in a layout, or the Coordination switch in the GUI):

| Strategy | Behaviour |
|---|---|
| `nexus` | Everything above (default) |
| `stop_and_wait` | Fixed priority by robot number; the lower-priority robot stops before the shared stretch and waits until it is fully clear. No rerouting. |
| `zone_lock` | Classical AGV block control: junctions and two-way stretches are split into 3×3 m blocks, one robot per block. Robots stop at each boundary, request the block (1 s handshake) and wait until it's granted. Fixed routes, fixed priority. |

All strategies share the same safety layer (cell claims and deadlock back-off).

---

## Tests

```bash
python test_benchmark.py         # baselines, flow-through, space-time planner, scenarios, harness
python test_grid_simulation.py   # lane simulation, reservations, deadlocks
python test_fleet.py             # missions, dispatch, batteries, charging
python test_operations.py        # alerts, reports, CSV
python test_gui_server.py        # REST + WebSocket API
python test_bel_layout.py        # BEL warehouse layout
# also: test_human_layout.py, test_layout_editing.py, test_editor_routes.py,
#       test_fast_detector.py, test_planner.py, test_paths.py

node test_editor_ops.mjs         # frontend editor logic (also test_editor_docs, test_editor_objects, test_ops_format)
```

---

## Project structure

```
agents/robot_agent.py        per-robot agent: local state, path, intent
communication/p2p.py         peer-to-peer state broadcast
algorithms/
  lane_grid.py               A* on the lane grid (one-way lanes, turn and zone costs)
  spacetime.py               space-time (safe-interval) planner
  fast_conflict_detector.py  predictive conflict detection
  negotiation.py             ETA-based priority negotiation
  cell_reservation.py        cell claims + wait-for cycle detection
nexus/
  grid_simulation.py         simulation runtime and coordination strategies
  grid_world.py              kinematics: acceleration, braking, rotate-in-place, speed caps
  layout.py                  layout format and validation (single source of truth)
  catalog.py                 racks, equipment, areas and safety zones
  fleet.py                   missions, auction dispatch, batteries, charging
  alerts.py, report.py       alerts and shift reports
  benchmark.py               scenarios and benchmark harness
nexus_gui/
  server.py                  FastAPI + WebSocket server
  static/                    dashboard and layout editor (plain JavaScript + SVG)
layouts/                     BEL Warehouse (Prototype), Fulfillment Center, demo layouts
data/benchmark/latest.json   latest benchmark results
```

`main.py`, `dashboard_server.py` and `configs/` belong to the earlier continuous-space (IR-SIM) prototype and aren't needed for the Control Room.

### Layout format (short version)
Layouts are JSON files in `layouts/`: a grid of cell characters plus stations, robots, objects and mission flows.

| Char | Meaning |
|---|---|
| `+` | two-way lane / junction |
| `> < ^ v` | one-way lane (direction of travel) |
| `S` | shelf |
| `#` | wall |
| `H` | human-only area |
| `W` | pedestrian walkway |
| `.` | floor (not drivable) |

The Layout Editor writes this format, so you rarely need to edit it by hand. See the docstring in `nexus/layout.py` for the full format.

---

## Edge deployment path
Each AMR runs the NEXUS agent on an onboard edge computer (Raspberry Pi 4 / Jetson Nano class):
- **ROS 2 nodes** run the agent loop: broadcast, planning, prediction, negotiation and the safety gate.
- **DDS over UDP** carries peer messages without any broker or central node.
- **Localization** comes from the robot's own odometry, IMU and LiDAR.
- **The dashboard only monitors.** It subscribes to the same peer messages, and the fleet keeps running if it goes offline.

Next steps: profile the agent on edge hardware, network impairment tests (latency, packet loss, Wi-Fi dead zones), a 3-robot pilot, then 10+ robots.

---

## Team
**Team A\*** · SIH 2026 · Team ID 152046
