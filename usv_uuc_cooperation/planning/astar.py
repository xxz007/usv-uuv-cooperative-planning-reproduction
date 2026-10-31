"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P2/P2-L4/源码/p2_l4_routes.py
"""
import heapq
import itertools
import math



def euclidean(a, b):
    return math.dist(a, b)


def inflated_nodes(case_map, obstacles):
    xmin, xmax = case_map["x_range"]
    ymin, ymax = case_map["y_range"]
    blocked = set()
    for x in range(xmin, xmax + 1):
        for y in range(ymin, ymax + 1):
            if any(max(abs(x - ox), abs(y - oy)) <= 1 for ox, oy in obstacles):
                blocked.add((x, y))
    return blocked


def segment_touches_square(a, b, center, half=0.5):
    """Inclusive Liang-Barsky segment/square test (boundary touch is unsafe)."""
    xmin, xmax = center[0] - half, center[0] + half
    ymin, ymax = center[1] - half, center[1] + half
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - xmin), (dx, xmax - a[0]),
                 (-dy, a[1] - ymin), (dy, ymax - a[1])):
        if abs(p) < 1e-12:
            if q < -1e-12:
                return False
            continue
        r = q / p
        if p < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1 + 1e-12:
            return False
    return True


def euclidean_leg(a, b, blocked):
    collision = sorted(
        (x, y) for x, y in blocked
        if segment_touches_square(a, b, (x, y))
    )
    return {
        "distance_m": euclidean(a, b),
        "collision": bool(collision),
        "collision_cells": [list(p) for p in collision],
    }


def astar(start, goal, case_map, blocked):
    start, goal = tuple(start), tuple(goal)
    xmin, xmax = case_map["x_range"]
    ymin, ymax = case_map["y_range"]
    if start in blocked or goal in blocked:
        return None

    def h(p):
        return abs(p[0] - goal[0]) + abs(p[1] - goal[1])

    # Deterministic tie-break: f, then g, then x, then y.
    queue = [(h(start), 0, start[0], start[1])]
    best = {start: 0}
    parent = {start: None}
    directions = ((1, 0), (0, 1), (-1, 0), (0, -1))
    while queue:
        _, g, x, y = heapq.heappop(queue)
        current = (x, y)
        if g != best.get(current):
            continue
        if current == goal:
            path = []
            while current is not None:
                path.append(list(current))
                current = parent[current]
            path.reverse()
            return {"path": path, "distance_m": len(path) - 1}
        for dx, dy in directions:
            nxt = (x + dx, y + dy)
            if not (xmin <= nxt[0] <= xmax and ymin <= nxt[1] <= ymax):
                continue
            if nxt in blocked:
                continue
            ng = g + 1
            if ng < best.get(nxt, math.inf):
                best[nxt] = ng
                parent[nxt] = current
                heapq.heappush(queue, (ng + h(nxt), ng, nxt[0], nxt[1]))
    return None


def points_for(case):
    result = dict(case["starts"])
    result.update(case["points"])
    return {name: tuple(coord) for name, coord in result.items()}


def leg_tables(case, obstacles):
    pts = points_for(case)
    blocked = inflated_nodes(case["map"], obstacles)
    names = list(pts)
    euclidean_table, astar_table = {}, {}
    for a, b in itertools.permutations(names, 2):
        key = f"{a}->{b}"
        euclidean_table[key] = euclidean_leg(pts[a], pts[b], blocked)
        astar_table[key] = astar(pts[a], pts[b], case["map"], blocked)
    return pts, blocked, euclidean_table, astar_table


def assignments(case):
    tasks = case["tasks"]
    cap = case["capacity"]
    result = []
    for owners in itertools.product(("U1", "U2"), repeat=len(tasks)):
        by_boat = {boat: [] for boat in case["boats"]}
        for task, boat in zip(tasks, owners):
            by_boat[boat].append(task)
        if all(len(by_boat[b]) <= cap for b in case["boats"]):
            result.append(by_boat)
    return result


def route_for(boat, sequence, case, table, mode):
    start = boat
    waypoints = [start, *sequence]
    legs = []
    total = 0.0
    full_path = []
    for a, b in zip(waypoints, waypoints[1:]):
        key = f"{a}->{b}"
        if mode == "euclidean":
            detail = table[key]
            legs.append({"from": a, "to": b, **detail})
            total += detail["distance_m"]
            full_path.append([list(points_for(case)[a]), list(points_for(case)[b])])
        else:
            detail = table[key]
            if detail is None:
                return {"status": "unreachable", "path": None,
                        "distance_m": None, "task_times": None,
                        "completion_time_s": None, "legs": legs + [{"from": a, "to": b,
                        "status": "unreachable", "path": None, "distance_m": None}]}
            legs.append({"from": a, "to": b, **detail})
            total += detail["distance_m"]
            segment = detail["path"]
            full_path.extend(segment if not full_path else segment[1:])
    if mode == "euclidean":
        colliding = any(leg["collision"] for leg in legs)
        status = "blocked" if colliding else "reachable"
        if colliding:
            return {"status": status, "path": full_path, "distance_m": total,
                    "legs": legs, "task_times": None,
                    "completion_time_s": None}

    # Compute arrival and service completion at every scheduled task.
    speed = case["speed_m_per_s"][boat]
    elapsed_distance, elapsed_service = 0.0, 0.0
    task_times = []
    for leg, task in zip(legs, sequence):
        elapsed_distance += leg["distance_m"]
        arrival = elapsed_distance / speed + elapsed_service
        done = arrival + case["service_s"][task]
        task_times.append({"task": task, "arrival_s": arrival,
                           "service_complete_s": done})
        elapsed_service += case["service_s"][task]
    finish = total / speed + elapsed_service
    return {"status": "reachable", "path": full_path, "distance_m": total,
            "legs": legs, "task_times": task_times,
            "completion_time_s": finish}


def valid_grid_path(path, start, goal, case_map, blocked):
    if not path or tuple(path[0]) != tuple(start) or tuple(path[-1]) != tuple(goal):
        return False
    xmin, xmax = case_map["x_range"]
    ymin, ymax = case_map["y_range"]
    for x, y in path:
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            return False
        if (x, y) in blocked:
            return False
    return all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
               for a, b in zip(path, path[1:]))


def enumerate_routes(case, euclidean_table, astar_table, mode, blocked=None):
    if mode == "astar" and blocked is None:
        raise ValueError("A* route validation requires the actual blocked-node set")
    table = euclidean_table if mode == "euclidean" else astar_table
    rows = []
    for alloc in assignments(case):
        for order1 in itertools.permutations(alloc["U1"]):
            for order2 in itertools.permutations(alloc["U2"]):
                routes = {
                    "U1": route_for("U1", order1, case, table, mode),
                    "U2": route_for("U2", order2, case, table, mode),
                }
                scheduled = list(order1) + list(order2)
                coverage = {task: scheduled.count(task) for task in case["tasks"]}
                coverage_ok = all(coverage[task] == 1 for task in case["tasks"])
                capacity_ok = all(len(order) <= case["capacity"]
                                  for order in (order1, order2))
                route_ok = all(r["status"] == "reachable" for r in routes.values())
                if mode == "astar" and route_ok:
                    pts = points_for(case)
                    route_ok = all(
                        valid_grid_path(route["path"], pts[boat],
                                        pts[route["legs"][-1]["to"]], case["map"], blocked)
                        and route["distance_m"] == len(route["path"]) - 1
                        and all(valid_grid_path(leg["path"], pts[leg["from"]],
                                                pts[leg["to"]], case["map"], blocked)
                                and leg["distance_m"] == len(leg["path"]) - 1
                                for leg in route["legs"])
                        for boat, route in routes.items())
                checks = {"coverage": coverage, "coverage_ok": coverage_ok,
                          "boat_task_counts": {"U1": len(order1), "U2": len(order2)},
                          "capacity_ok": capacity_ok, "route_ok": route_ok}
                feasible = coverage_ok and capacity_ok and route_ok
                rows.append({
                    "assignment": {"U1": list(order1), "U2": list(order2)},
                    "feasible": feasible,
                    "checks": checks,
                    "routes": routes,
                    "total_distance_m": (sum(r["distance_m"] for r in routes.values())
                                          if feasible else None),
                    "team_completion_time_s": (max(r["completion_time_s"] for r in routes.values())
                                                if feasible else None),
                })
    valid = [r for r in rows if r["feasible"]]
    best_distance = min((r["total_distance_m"] for r in valid), default=None)
    best = [r for r in valid if math.isclose(r["total_distance_m"], best_distance,
                                             rel_tol=0, abs_tol=1e-9)] if valid else []
    best_makespan = min((r["team_completion_time_s"] for r in valid), default=None)
    best_time = [r for r in valid if math.isclose(r["team_completion_time_s"], best_makespan,
                                                   rel_tol=0, abs_tol=1e-9)] if valid else []
    return {
        "feasibility_definition": ("continuous straight-line reference: every leg avoids buffered cells; "
                                   "diagonal travel is permitted only in this geometric reference"
                                   if mode == "euclidean" else
                                   "four-neighbor grid routes: all legs reachable, safe and within bounds"),
        "ownership_candidate_count": len(case["boats"]) ** len(case["tasks"]),
        "capacity_feasible_ownership_count": len(assignments(case)),
        "candidate_route_count": len(rows),
        "feasible_route_count": len(valid),
        "excluded_route_count": len(rows) - len(valid),
        "optimal_total_distance_m": best_distance,
        "optimal_distance_routes": best,
        "optimal_makespan_s": best_makespan,
        "optimal_makespan_routes": best_time,
        "all_routes": rows,
    }

