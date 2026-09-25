"""
Grid simulation invariants on every layout in layouts/.

For each layout, run the simulation and check that robots:
    * only ever stand on lane / station cells
    * move only along straight lines through cell centres (no curves)
    * never make a move the lane graph forbids (one-way lanes, shelves, ...)
    * never come closer than 0.7 cell sizes (no contact)

Run:  python test_grid_simulation.py
"""

from pathlib import Path

from nexus.grid_simulation import GridSimulation
from nexus.layout import load_layout


MAX_STEPS = 3000


def run_layout(path):
    layout = load_layout(path)
    sim = GridSimulation(layout)
    cs = layout.cell_size
    off_grid = 0
    illegal = 0
    previous = {}

    for _ in range(MAX_STEPS):
        sim.step()
        for agent in sim.agents:
            x, y = agent.state.position
            fx = (x / cs - 0.5) % 1.0
            fy = (y / cs - 0.5) % 1.0
            on_line = min(fx, 1 - fx) < 1e-6 or min(fy, 1 - fy) < 1e-6
            cell = layout.world_to_cell((x, y))
            if not on_line or not layout.is_drivable(cell):
                off_grid += 1
            last = previous.get(agent.robot_id)
            if last is not None and last != cell and not layout.move_allowed(last, cell):
                illegal += 1
            previous[agent.robot_id] = cell
        if sim.status != "RUNNING":
            break

    return sim, off_grid, illegal


def main():
    print("=== NEXUS grid simulation invariants ===")
    failures = 0

    for path in sorted(Path("layouts").glob("*.json")):
        sim, off_grid, illegal = run_layout(path)
        m = sim.metrics()
        ok = off_grid == 0 and illegal == 0 and m["safety_violations"] == 0 and sim.status != "INVALID"
        failures += 0 if ok else 1
        print(
            f"[{'PASS' if ok else 'FAIL'}] {path.stem:22s} "
            f"{sim.status:9s} t={m['time']:6.1f}s robots={m['robots']:2d} "
            f"trips={m['trips_completed']:3d} negotiations={m['negotiations']:3d} "
            f"deadlocks={m['deadlocks_resolved']:2d} safety={m['safety_violations']} "
            f"off-grid={off_grid} illegal={illegal}"
        )

    print("ALL PASS" if failures == 0 else f"{failures} layout(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
