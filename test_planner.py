from algorithms.planner import NEXUSPlanner


def main():

    planner = NEXUSPlanner()

    test_cases = [
        (
            "R1",
            (2.0, 3.0),
            (28.0, 17.0),
        ),
        (
            "R2",
            (2.0, 7.0),
            (28.0, 13.0),
        ),
        (
            "R3",
            (2.0, 11.0),
            (28.0, 9.0),
        ),
        (
            "R4",
            (28.0, 17.0),
            (2.0, 3.0),
        ),
        (
            "R5",
            (28.0, 13.0),
            (2.0, 7.0),
        ),
        (
            "R6",
            (28.0, 9.0),
            (2.0, 11.0),
        ),
    ]

    print()
    print("=" * 60)
    print("              NEXUS A* PLANNER TEST")
    print("=" * 60)

    for robot_id, start, goal in test_cases:

        path = planner.plan(
            start,
            goal,
        )

        distance = 0.0

        for i in range(1, len(path)):

            x1, y1 = path[i - 1]
            x2, y2 = path[i]

            distance += (
                (x2 - x1) ** 2 +
                (y2 - y1) ** 2
            ) ** 0.5

        print()
        print(
            f"{robot_id}: "
            f"{start} -> {goal}"
        )

        print(
            f"  Waypoints : {len(path)}"
        )

        print(
            f"  Distance  : {distance:.2f} m"
        )

        print(
            f"  Start     : {path[0]}"
        )

        print(
            f"  End       : {path[-1]}"
        )

    print()
    print("=" * 60)
    print("A* PLANNING TEST COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()
