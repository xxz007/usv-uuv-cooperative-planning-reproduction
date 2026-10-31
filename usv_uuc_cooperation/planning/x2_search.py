"""Instrumented A* and independent h=0 Dijkstra on one observed safe graph."""
import hashlib
import heapq
import json
import math
import time

from usv_uuc_cooperation.env.obstacle import forbidden_nodes, unsafe_reason
from usv_uuc_cooperation.planning.grid_planner import heuristic


class ObservedGraph:
    def __init__(self, config, blocked, version):
        stamp = time.perf_counter_ns()
        self.config, self.blocked, self.version = config, set(map(tuple, blocked)), version
        m = config['map']
        if m['spacing_m'] != 1 or m['adjacency'] != 8:
            raise ValueError('X2 requires the frozen 1m eight-neighbor graph')
        self.edges = {}
        directions = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))
        for x in range(m['x_range'][0], m['x_range'][1] + 1):
            for y in range(m['y_range'][0], m['y_range'][1] + 1):
                a = x, y
                if unsafe_reason(a, a, config, self.blocked):
                    continue
                edges = []
                for dx, dy in directions:
                    b = x + dx, y + dy
                    if dx and dy and ((x + dx, y) in self.blocked or (x, y + dy) in self.blocked):
                        continue
                    if unsafe_reason(a, b, config, self.blocked) is None:
                        edges.append((b, math.hypot(dx, dy)))
                self.edges[a] = edges
        payload = {'map': m, 'blocked': sorted(self.blocked), 'version': version}
        self.sha256 = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
        self.build_ns = time.perf_counter_ns() - stamp


def _result(start, goal, best, parent, expanded, pops, pushes, stale, checks):
    path = None
    if goal in best and expanded and expanded[-1]['node'] == list(goal):
        path, tail = [], goal
        while tail is not None:
            path.append(list(tail))
            tail = parent[tail]
        path.reverse()
    return {'status': 'REACHABLE' if path is not None else 'UNREACHABLE', 'path': path,
            'distance_m': math.fsum(math.dist(a, b) for a, b in zip(path, path[1:])) if path is not None else None,
            'search_cost_m': best[goal] if path is not None else None,
            'expanded_nodes': len(expanded), 'unique_expanded_nodes': len({tuple(e['node']) for e in expanded}),
            'heap_pops': pops, 'heap_pushes': pushes, 'stale_pops': stale, 'edge_checks': checks,
            'expansion_trace': expanded, 'best_g_m': [[*p, g] for p, g in sorted(best.items())],
            'parent': [[*p, list(q) if q is not None else None] for p, q in sorted(parent.items())]}


def astar_search(start, goal, graph):
    """A* priority (g+octile,g,x,y); stale entries never count as effective pops."""
    best, parent = {start: 0.0}, {start: None}
    queue = [(heuristic(start, goal, 8), 0.0, *start)]
    expanded, pops, pushes, stale, checks = [], 0, 1, 0, 0
    stamp = time.perf_counter_ns()
    while queue:
        f, g, x, y = heapq.heappop(queue)
        pops += 1
        a = x, y
        if g != best.get(a):
            stale += 1
            continue
        expanded.append({'node': list(a), 'g_m': g, 'priority_m': f})
        if a == goal:
            break
        for b, weight in graph.edges[a]:
            checks += 1
            candidate = g + weight
            if candidate < best.get(b, math.inf):
                best[b], parent[b] = candidate, a
                heapq.heappush(queue, (candidate + heuristic(b, goal, 8), candidate, *b))
                pushes += 1
    elapsed = time.perf_counter_ns() - stamp
    result = _result(start, goal, best, parent, expanded, pops, pushes, stale, checks)
    result['search_ns'] = elapsed
    return result


def dijkstra_search(start, goal, graph):
    """Independent uniform-cost loop, priority (g,x,y), no A* invocation."""
    best, parent = {start: 0.0}, {start: None}
    queue = [(0.0, *start)]
    expanded, pops, pushes, stale, checks = [], 0, 1, 0, 0
    stamp = time.perf_counter_ns()
    while queue:
        g, x, y = heapq.heappop(queue)
        pops += 1
        a = x, y
        if g != best.get(a):
            stale += 1
            continue
        expanded.append({'node': list(a), 'g_m': g, 'priority_m': g})
        if a == goal:
            break
        for b, weight in graph.edges[a]:
            checks += 1
            candidate = g + weight
            if candidate < best.get(b, math.inf):
                best[b], parent[b] = candidate, a
                heapq.heappush(queue, (candidate, *b))
                pushes += 1
    elapsed = time.perf_counter_ns() - stamp
    result = _result(start, goal, best, parent, expanded, pops, pushes, stale, checks)
    result['search_ns'] = elapsed
    return result


class SearchSession:
    def __init__(self, config, algorithm):
        if algorithm not in ('Astar', 'Dijkstra'):
            raise ValueError(algorithm)
        self.algorithm = algorithm
        self.rows, self.graphs = [], []
        self.time_s, self.reason = 0, 'LOCAL_U1_T03'
        self.observe(config, forbidden_nodes(config), 0)

    def observe(self, config, blocked, version):
        self.graph = ObservedGraph(config, blocked, version)
        self.graphs.append({'version': version, 'sha256': self.graph.sha256,
                            'blocked': [list(p) for p in sorted(blocked)],
                            'node_count': len(self.graph.edges), 'build_ns': self.graph.build_ns})

    def context(self, time_s, reason):
        self.time_s, self.reason = time_s, reason

    def __call__(self, start, goal, config, blocked):
        start, goal = tuple(start), tuple(goal)
        if any(v != int(v) or not math.isfinite(v) for p in (start, goal) for v in p):
            raise ValueError('Search endpoints must be integer grid coordinates')
        if set(map(tuple, blocked)) != self.graph.blocked or config['map'] != self.graph.config['map']:
            raise ValueError('Planner received a different observed graph')
        valid = start in self.graph.edges and goal in self.graph.edges
        if valid:
            fn = astar_search if self.algorithm == 'Astar' else dijkstra_search
            result = fn(start, goal, self.graph)
        else:
            result = _result(start, goal, {}, {}, [], 0, 0, 0, 0)
            result['status'] = 'INVALID_ENDPOINT'
            result['search_ns'] = 0
        row = {**result, 'query_id': len(self.rows) + 1, 'algorithm': self.algorithm,
               'time_s': self.time_s, 'reason': self.reason, 'start': list(start), 'goal': list(goal),
               'graph_version': self.graph.version, 'graph_sha256': self.graph.sha256,
               'heuristic': 'octile' if self.algorithm == 'Astar' else 'zero',
               'timing_scope': 'search priority-queue loop with in-memory expansion tracing; excludes graph build, endpoint checks, path reconstruction, serialization, IO and drawing',
               'work_definition': 'non-stale priority-queue pops, including goal; not unique-node count'}
        self.rows.append(row)
        if result['path'] is None:
            return None
        return {**result, 'adjacency': 8, 'search_query_id': row['query_id']}
