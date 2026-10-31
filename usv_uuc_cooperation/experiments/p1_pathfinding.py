"""P1's saved three-scenario experiment, with the original shortest-path rules."""
from usv_uuc_cooperation.planning import p1_astar as planner


def run():
    direct = planner.straight_nodes()
    results = {}
    for name, obstacles in planner.SCENARIOS.items():
        blocked = planner.inflated_nodes(obstacles)
        path = planner.plan(blocked)
        if path is not None and not planner.check_path(path, blocked):
            raise ValueError(f"Invalid path: {name}")
        results[name] = {
            "obstacles": sorted(map(list, obstacles)),
            "blocked_count": len(blocked),
            "straight_length": len(direct) - 1,
            "straight_clear": all(node not in blocked for node in direct),
            "status": "found" if path is not None else "unreachable",
            "astar_length": len(path) - 1 if path is not None else None,
            "path": path,
        }
    return results
