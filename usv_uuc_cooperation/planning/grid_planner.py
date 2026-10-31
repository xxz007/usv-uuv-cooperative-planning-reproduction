"""Standalone safe 4/8-neighbor A* on the existing frozen 1 m map.

Existing astar/continuous-route callers retain their accepted four-neighbor code.
"""
import heapq
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes, unsafe_reason


def heuristic(point, goal, adjacency=4, spacing_m=1.0):
    """Manhattan/octile distance for index differences, in metres."""
    if adjacency not in (4, 8):
        raise ValueError("adjacency must be 4 or 8")
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("spacing_m must be finite and positive")
    dx, dy = abs(point[0] - goal[0]), abs(point[1] - goal[1])
    return spacing_m * (dx + dy if adjacency == 4 else
                        max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy))


def plan(start, goal, config, blocked=None, *, adjacency=4):
    """Return a safe geometric path, or None; no executor/scheduler is connected.

Coordinates are physical metres on the existing integer 1 m grid. Other grid
spacings require a separate raster/coordinate adaptation and are rejected here.
"""
    if adjacency not in (4, 8):
        raise ValueError("adjacency must be 4 or 8")
    if config["map"]["spacing_m"] != 1:
        raise ValueError("This planner uses the frozen 1 m physical-coordinate grid")
    for point in (start, goal):
        if len(point) != 2 or any(not math.isfinite(v) or v != int(v) for v in point):
            raise ValueError("start and goal must be finite integer grid coordinates")
    start, goal = tuple(map(int, start)), tuple(map(int, goal))
    blocked = forbidden_nodes(config) if blocked is None else set(map(tuple, blocked))
    if unsafe_reason(start, start, config, blocked) or unsafe_reason(goal, goal, config, blocked):
        return None
    directions = ((1, 0), (0, 1), (-1, 0), (0, -1))
    if adjacency == 8:
        directions += ((1, 1), (-1, 1), (-1, -1), (1, -1))
    best, parent = {start: 0.0}, {start: None}
    queue = [(heuristic(start, goal, adjacency), 0.0, *start)]
    expanded = 0
    while queue:
        _, g, x, y = heapq.heappop(queue)
        current = (x, y)
        if g != best.get(current):
            continue
        expanded += 1
        if current == goal:
            path = []
            while current is not None:
                path.append(list(current))
                current = parent[current]
            path.reverse()
            straight = sum(a[0] == b[0] or a[1] == b[1] for a, b in zip(path, path[1:]))
            steps = len(path) - 1
            return {"path": path, "distance_m": math.fsum(math.dist(a, b) for a, b in zip(path, path[1:])),
                    "search_cost_m": g, "step_count": steps, "straight_edges": straight,
                    "diagonal_edges": steps - straight, "adjacency": adjacency,
                    "heuristic": "manhattan" if adjacency == 4 else "octile",
                    "expanded_nodes": expanded}
        for dx, dy in directions:
            nxt = (x + dx, y + dy)
            if dx and dy and ((x + dx, y) in blocked or (x, y + dy) in blocked):
                continue
            if unsafe_reason(current, nxt, config, blocked):
                continue
            ng = g + math.hypot(dx, dy)
            if ng < best.get(nxt, math.inf):
                best[nxt], parent[nxt] = ng, current
                heapq.heappush(queue, (ng + heuristic(nxt, goal, adjacency), ng, *nxt))
    return None
