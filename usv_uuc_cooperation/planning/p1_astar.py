"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P1/P1-L3/源码/p1_l3_astar_verify.py
"""
import heapq

SIZE=7
START=(0,1)
GOAL=(6,1)
SCENARIOS={'baseline': {(3,3)}, 'added': {(3,3),(3,1)}, 'unreachable': {(3,3),(3,1),(3,5)}}

def inflated_nodes(obstacles):
    """One-node square buffer, matching the P1-L1 classroom rule."""
    return {
        (x, y)
        for x in range(SIZE)
        for y in range(SIZE)
        if any(max(abs(x - ox), abs(y - oy)) <= 1 for ox, oy in obstacles)
    }


def neighbors(node, blocked):
    x, y = node
    for nxt in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
        if 0 <= nxt[0] < SIZE and 0 <= nxt[1] < SIZE and nxt not in blocked:
            yield nxt


def heuristic(node):
    return abs(node[0] - GOAL[0]) + abs(node[1] - GOAL[1])


def plan(blocked):
    """Return a start-to-goal shortest path, or None when no path exists."""
    if START in blocked or GOAL in blocked:
        return None
    queue = [(heuristic(START), 0, START)]
    best_cost = {START: 0}
    parent = {START: None}

    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost != best_cost[current]:
            continue
        if current == GOAL:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1]
        for nxt in neighbors(current, blocked):
            next_cost = cost + 1
            if next_cost < best_cost.get(nxt, float("inf")):
                best_cost[nxt] = next_cost
                parent[nxt] = current
                heapq.heappush(queue, (next_cost + heuristic(nxt), next_cost, nxt))
    return None


def straight_nodes():
    """The proposed direct route for this horizontal start-goal pair."""
    return [(x, START[1]) for x in range(START[0], GOAL[0] + 1)]


def check_path(path, blocked):
    return (
        path is not None
        and path[0] == START
        and path[-1] == GOAL
        and all(node not in blocked for node in path)
        and all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
                for a, b in zip(path, path[1:]))
    )

