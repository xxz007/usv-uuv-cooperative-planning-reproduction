"""Independent Dijkstra and closed-square geometry audit for X1-L1."""
import heapq
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.planning.grid_planner import plan, heuristic
from usv_uuc_cooperation.planning.astar import astar


def segment_safe(a, b, config, blocked):
    """Independent slab intersection; closed-square boundary contact is unsafe."""
    grid = config["map"]
    if any(not (grid["x_range"][0] <= p[0] <= grid["x_range"][1]
                and grid["y_range"][0] <= p[1] <= grid["y_range"][1]) for p in (a, b)):
        return False
    half = grid["spacing_m"] / 2
    for cell in blocked:
        enter, leave = 0.0, 1.0
        for axis in (0, 1):
            delta = b[axis] - a[axis]
            lo, hi = cell[axis] - half, cell[axis] + half
            if delta == 0:
                if not lo <= a[axis] <= hi:
                    break
            else:
                u, v = sorted(((lo - a[axis]) / delta, (hi - a[axis]) / delta))
                enter, leave = max(enter, u), min(leave, v)
                if enter > leave:
                    break
        else:
            return False
    return True


def edges(node, config, blocked, adjacency):
    x, y = node
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if (dx == 0 and dy == 0) or (adjacency == 4 and dx and dy):
                continue
            nxt = (x + dx, y + dy)
            if dx and dy and ((x + dx, y) in blocked or (x, y + dy) in blocked):
                continue
            if segment_safe(node, nxt, config, blocked):
                yield nxt, math.dist(node, nxt)


def dijkstra(start, goal, config, blocked, adjacency):
    start, goal = tuple(start), tuple(goal)
    if not all(segment_safe(p, p, config, blocked) for p in (start, goal)):
        return None
    distances, queue = {start: 0.0}, [(0.0, start)]
    while queue:
        distance, current = heapq.heappop(queue)
        if distance != distances[current]:
            continue
        if current == goal:
            return distance
        for nxt, cost in edges(current, config, blocked, adjacency):
            candidate = distance + cost
            if candidate < distances.get(nxt, math.inf):
                distances[nxt] = candidate
                heapq.heappush(queue, (candidate, nxt))
    return None


def audit_result(result):
    config = result["config"]
    blocked = forbidden_nodes(config)
    assert blocked == set(map(tuple, result["blocked_nodes"]))
    rows = {}
    for key, route in result["routes"].items():
        mode = int(key)
        assert route["adjacency"] == mode
        assert route["heuristic"] == ("manhattan" if mode == 4 else "octile")
        path = route["path"]
        assert path[0] == config["start_m"] and path[-1] == config["goal_m"]
        for point in path:
            assert len(point) == 2 and all(math.isfinite(v) and v == int(v) for v in point)
            assert segment_safe(point, point, config, blocked)
        diagonal = 0
        lengths = []
        for a, b in zip(path, path[1:]):
            dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
            assert max(dx, dy) == 1 and (mode == 8 or dx + dy == 1)
            assert segment_safe(a, b, config, blocked)
            if dx and dy:
                assert (a[0], b[1]) not in blocked and (b[0], a[1]) not in blocked
                diagonal += 1
            lengths.append(math.dist(a, b))
        length = math.fsum(lengths)
        oracle = dijkstra(path[0], path[-1], config, blocked, mode)
        assert math.isclose(length, oracle, abs_tol=1e-9)
        assert math.isclose(length, route["distance_m"], abs_tol=1e-9)
        assert math.isclose(length, route["search_cost_m"], abs_tol=1e-9)
        assert route["step_count"] == len(path) - 1
        assert route["diagonal_edges"] == diagonal
        assert route["straight_edges"] == len(path) - 1 - diagonal
        rows[key] = {"status": "PASS", "euclidean_sum_m": length, "dijkstra_m": oracle,
                     "step_count": len(path) - 1, "straight_edges": len(path) - 1 - diagonal,
                     "diagonal_edges": diagonal, "whole_segments_safe": True,
                     "no_corner_cutting": True}
    return {"status": "PASS", "routes": rows}


def teacher_checks(config):
    small = {"map": {"x_range": [0, 3], "y_range": [0, 3], "spacing_m": 1,
                      "circles": [], "safety_buffer_m": 1}}
    checks = []
    def check(name, condition):
        assert condition, name
        checks.append({"name": name, "status": "PASS"})

    for mode, expected in ((4, 5), (8, 1 + 2 * math.sqrt(2))):
        route = plan([0, 0], [3, 2], small, adjacency=mode)
        check(f"empty_map_{mode}_Astar_Dijkstra", math.isclose(route["distance_m"], expected)
              and math.isclose(route["distance_m"], dijkstra([0, 0], [3, 2], small, set(), mode)))
        check(f"empty_map_{mode}_matching_heuristic", math.isclose(heuristic([0, 0], [3, 2], mode), expected))
    for name, cells in (("one_side", {(1, 0)}), ("two_sides", {(1, 0), (0, 1)})):
        route = plan([0, 0], [1, 1], small, cells, adjacency=8)
        oracle = dijkstra([0, 0], [1, 1], small, cells, 8)
        check(f"corner_{name}_direct_diagonal_rejected", not segment_safe([0, 0], [1, 1], small, cells)
              and (route is None or route["step_count"] > 1))
        check(f"corner_{name}_Dijkstra", (route is None and oracle is None)
              or (route is not None and oracle is not None and math.isclose(route["distance_m"], oracle)))
    check("safe_endpoints_not_sufficient_for_long_segment",
          segment_safe([0, 1], [0, 1], small, {(1, 1)})
          and segment_safe([2, 1], [2, 1], small, {(1, 1)})
          and not segment_safe([0, 1], [2, 1], small, {(1, 1)}))
    check("singleton_safe_zero_length", plan([0, 0], [0, 0], small)["distance_m"] == 0)
    check("blocked_and_out_of_bounds_endpoints_rejected",
          plan([0, 0], [1, 1], small, {(0, 0)}) is None
          and plan([-1, 0], [1, 1], small) is None)
    blocked = forbidden_nodes(config)
    legacy = astar(config["start_m"], config["goal_m"], config["map"], blocked)
    default = plan(config["start_m"], config["goal_m"], config, blocked)
    check("default_four_legacy_path_and_length_unchanged", default["adjacency"] == 4
          and default["path"] == legacy["path"] and default["distance_m"] == legacy["distance_m"])
    routes = {str(mode): plan(config["start_m"], config["goal_m"], config, blocked, adjacency=mode)
              for mode in (4, 8)}
    audit = audit_result({"config": config, "blocked_nodes": sorted(map(list, blocked)), "routes": routes})
    check("main_case_legal_diagonal_benefit", routes["8"]["diagonal_edges"] > 0
          and routes["8"]["distance_m"] < routes["4"]["distance_m"])
    for mode in (4, 8):
        goal = config["goal_m"]
        consistent = all(heuristic(node, goal, mode) <= cost + heuristic(nxt, goal, mode) + 1e-9
                         for x in range(config["map"]["x_range"][0], config["map"]["x_range"][1] + 1)
                         for y in range(config["map"]["y_range"][0], config["map"]["y_range"][1] + 1)
                         if (node := (x, y)) not in blocked
                         for nxt, cost in edges(node, config, blocked, mode))
        check(f"main_{mode}_heuristic_consistent_on_legal_edges", consistent)
    return {"status": "PASS", "checks": checks, "main_case_audit": audit,
            "learner_acceptance": "not implied by teacher checks"}
