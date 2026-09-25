from nexus.simulation_manager import SimulationManager
from nexus.persistence import save_config


def main():
    manager = SimulationManager()

    print("=== NEXUS dashboard smoke test ===")
    print(f"initial status: {manager.status}")
    print(f"robot count: {len(manager.agents)}")

    payload = manager.snapshot()
    print(f"snapshot time: {payload['simulation_time']:.1f}s")
    print(f"first robot: {payload['robots'][0]['id']} -> {payload['robots'][0]['intent']}")

    save_path = "data/simulations/test_dashboard.json"
    save_config(manager.config, save_path)
    print(f"saved config to: {save_path}")

    for i in range(5):
        payload = manager.step()
        print(f"step {i + 1}: time={payload['simulation_time']:.1f}s status={payload['status']} robots_moving={payload['metrics']['robots_moving']}")

    final = manager.snapshot()
    print("final metrics:")
    print(final["metrics"])


if __name__ == "__main__":
    main()
