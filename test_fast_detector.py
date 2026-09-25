"""
FastConflictDetector must find exactly the same conflicts as NEXUS's
ConflictDetector (algorithms/conflict_detector.py), only faster.

At many points during real simulations (layouts with lots of cross traffic)
both detectors are run on the same robots and their outputs compared field
by field.

Run:  python test_fast_detector.py
"""

import time

from algorithms.conflict_detector import ConflictDetector
from nexus.grid_simulation import GridSimulation
from nexus.layout import load_layout

LAYOUTS = ["head_on_2", "cross_traffic_6", "fulfillment_center", "bel_warehouse"]


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def key(conflicts):
    return [
        (c.robot_a, c.robot_b, round(c.time_to_conflict, 9), tuple(round(float(v), 9) for v in c.conflict_position),
         round(c.minimum_distance, 9))
        for c in conflicts
    ]


def main():
    print("=== FastConflictDetector equivalence ===")
    failures = 0
    total_found = 0
    fast_time = ref_time = 0.0
    for name in LAYOUTS:
        sim = GridSimulation(load_layout(f"layouts/{name}.json"))
        ref = ConflictDetector(prediction_horizon=sim.detector.prediction_horizon,
                               prediction_dt=sim.detector.prediction_dt,
                               safety_distance=sim.detector.safety_distance)
        compared = mismatches = found = 0
        for step in range(1500):
            sim.step()
            if sim.status != "RUNNING":
                break
            t = time.perf_counter()
            fast = sim.detector.detect_all(sim.agents)
            fast_time += time.perf_counter() - t
            t = time.perf_counter()
            slow = ref.detect_all(sim.agents)
            ref_time += time.perf_counter() - t
            compared += 1
            found += len(slow)
            mismatches += key(fast) != key(slow)
        total_found += found
        failures += check(mismatches == 0, f"{name}: {compared} snapshots, {found} conflicts, identical results")
    failures += check(total_found >= 20, f"comparison includes real conflicts ({total_found})")
    failures += check(ref_time > 3 * fast_time, f"faster: {ref_time / max(fast_time, 1e-9):.1f}x")
    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
