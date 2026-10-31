"""D* Lite Figure 3, Koenig/Likhachev AAAI 2002, positive octile graph.

https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
No A* call: g/rhs, logical queue and km persist within each owner/goal cache.
"""
import heapq
import math
import time

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.planning.grid_planner import heuristic
from usv_uuc_cooperation.planning.x2_search import ObservedGraph


def finite(value):
    return value if math.isfinite(value) else None


def predecessors(graph):
    output = {}
    for a, edges in graph.edges.items():
        for b, _ in edges:
            output.setdefault(b, set()).add(a)
    return output


class DStarLite:
    def __init__(self, graph, start, goal):
        self.graph, self.start, self.last_start, self.goal = graph, start, start, goal
        self.km = 0.0
        m = graph.config['map']
        self.nodes = [(x, y) for x in range(m['x_range'][0], m['x_range'][1] + 1)
                      for y in range(m['y_range'][0], m['y_range'][1] + 1)]
        self.g = dict.fromkeys(self.nodes, math.inf)
        self.rhs = dict.fromkeys(self.nodes, math.inf)
        self.rhs[goal] = 0.0
        self.heap, self.active, self.serial = [], {}, 0
        self.pred = predecessors(graph)
        self.push(goal)

    def key(self, node):
        low = min(self.g[node], self.rhs[node])
        return low + heuristic(self.start, node, 8) + self.km, low

    def push(self, node):
        self.serial += 1
        key = self.key(node)
        self.active[node] = key, self.serial
        heapq.heappush(self.heap, (*key, *node, self.serial))

    def clean_top(self, work):
        while self.heap:
            a, b, x, y, serial = self.heap[0]
            if self.active.get((x, y)) == ((a, b), serial):
                return a, b
            heapq.heappop(self.heap)
            work['stale_pops'] += 1
        return math.inf, math.inf

    def update_vertex(self, node, work):
        old = self.rhs[node]
        if node != self.goal:
            edges = self.graph.edges.get(node, ())
            work['edge_checks'] += len(edges)
            self.rhs[node] = min((weight + self.g[b] for b, weight in edges), default=math.inf)
        self.active.pop(node, None)
        if self.g[node] != self.rhs[node]:
            self.push(node)
            work['queue_insertions'] += 1
        work['update_vertex_calls'] += 1
        work['rhs_updates'].append({'node': list(node), 'g_m': finite(self.g[node]),
            'rhs_before_m': finite(old), 'rhs_after_m': finite(self.rhs[node]),
            'inconsistent': self.g[node] != self.rhs[node]})

    def snapshot(self):
        return {'start': list(self.start), 'goal': list(self.goal), 'km_m': self.km,
                'graph_version': self.graph.version, 'graph_sha256': self.graph.sha256,
                'g_rhs': [[*p, finite(self.g[p]), finite(self.rhs[p])] for p in self.nodes],
                'queue': [[*p, *key] for p, (key, _) in sorted(self.active.items())]}

    def solve(self, start, graph):
        before = self.snapshot()
        old_start, old_graph = self.start, self.graph
        self.km += heuristic(self.last_start, start, 8)
        self.start = self.last_start = start
        stamp = time.perf_counter_ns()
        changed, changed_edges = [], []
        if old_graph.sha256 != graph.sha256:
            for a in self.nodes:
                old, new = dict(old_graph.edges.get(a, ())), dict(graph.edges.get(a, ()))
                delta = [(b, old.get(b, math.inf), new.get(b, math.inf))
                         for b in sorted(old.keys() | new.keys())
                         if old.get(b, math.inf) != new.get(b, math.inf)]
                if delta:
                    changed.append(a)
                    changed_edges.extend({'from': list(a), 'to': list(b),
                                          'old_cost_m': finite(c), 'new_cost_m': finite(d)} for b, c, d in delta)
        self.graph = graph
        if old_graph.sha256 != graph.sha256:
            self.pred = predecessors(graph)
        graph_diff_ns = time.perf_counter_ns() - stamp
        work = {'stale_pops': 0, 'queue_insertions': 0, 'update_vertex_calls': 0,
                'edge_checks': 0, 'rhs_updates': [], 'expansion_trace': []}
        stamp = time.perf_counter_ns()
        for node in changed:
            self.update_vertex(node, work)
        seed_ns = time.perf_counter_ns() - stamp
        after_edge_updates = self.snapshot()
        stamp = time.perf_counter_ns()
        while self.clean_top(work) < self.key(start) or self.rhs[start] != self.g[start]:
            if not self.heap:
                raise RuntimeError('Inconsistent start with empty logical queue')
            k1, k2, x, y, serial = heapq.heappop(self.heap)
            node, old_key = (x, y), (k1, k2)
            assert self.active.pop(node) == (old_key, serial)
            new_key = self.key(node)
            item = {'node': list(node), 'old_key': [finite(v) for v in old_key],
                    'new_key': [finite(v) for v in new_key],
                    'g_before_m': finite(self.g[node]), 'rhs_before_m': finite(self.rhs[node])}
            if old_key < new_key:
                self.push(node)
                work['queue_insertions'] += 1
                item['action'] = 'REKEY'
            elif self.g[node] > self.rhs[node]:
                self.g[node] = self.rhs[node]
                item['action'] = 'SET_G_TO_RHS'
                for parent in sorted(self.pred.get(node, ())):
                    self.update_vertex(parent, work)
            else:
                self.g[node] = math.inf
                item['action'] = 'INVALIDATE_G'
                for parent in sorted(self.pred.get(node, set()) | {node}):
                    self.update_vertex(parent, work)
            item.update(g_after_m=finite(self.g[node]), rhs_after_m=finite(self.rhs[node]))
            work['expansion_trace'].append(item)
        loop_ns = time.perf_counter_ns() - stamp
        assert self.g[start] == self.rhs[start]
        path, node = None, start
        if math.isfinite(self.g[start]):
            path, seen = [list(start)], {start}
            while node != self.goal:
                options = [(w + self.g[b], b) for b, w in graph.edges[node]]
                value, node = min(options)
                if not math.isfinite(value) or node in seen:
                    raise RuntimeError('Invalid D* Lite cost descent')
                path.append(list(node))
                seen.add(node)
        after = self.snapshot()
        trace = work['expansion_trace']
        return {**work, 'status': 'REACHABLE' if path is not None else 'UNREACHABLE',
                'path': path, 'distance_m': math.fsum(math.dist(a, b) for a, b in zip(path, path[1:])) if path else None,
                'search_cost_m': finite(self.g[start]), 'search_ns': seed_ns + loop_ns,
                'graph_diff_ns': graph_diff_ns, 'expanded_nodes': len(trace),
                'unique_expanded_nodes': len({tuple(r['node']) for r in trace}),
                'heap_pops': len(trace) + work['stale_pops'],
                'changed_edges': changed_edges, 'changed_tails': [list(p) for p in changed],
                'start_moved': start != old_start, 'graph_changed': old_graph.sha256 != graph.sha256,
                'before_state': before, 'after_edge_updates': after_edge_updates, 'after_state': after,
                'km_increment_m': heuristic(old_start, start, 8),
                'state_identity_preserved': True}


class DStarSession:
    """Explicit per-owner/goal caches also support nonexecuted connector candidates.

New goals never inherit another goal's g/rhs. Existing exact-goal caches may
reactivate, synchronize to the current graph and adjust km for their own start.
"""
    algorithm = 'DstarLite'

    def __init__(self, config):
        self.rows, self.graphs, self.states, self.last_goal = [], [], {}, {}
        self.time_s, self.reason = 0, 'LOCAL_INCREMENTAL'
        self.owner, self.task, self.query_role = 'LOCAL', None, 'FIXED_TARGET_SEGMENT'
        self.next_state_id = 0
        self.observe({'map': config['map']}, forbidden_nodes(config), 0)

    def observe(self, config, blocked, version):
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('D* Lite observation must contain current physical data only')
        self.graph = ObservedGraph({'map': config['map']}, blocked, version)
        self.graphs.append({'version': version, 'sha256': self.graph.sha256,
                            'blocked': [list(p) for p in sorted(blocked)],
                            'node_count': len(self.graph.edges), 'build_ns': self.graph.build_ns})

    def context(self, time_s, reason):
        self.time_s, self.reason = time_s, reason

    def route_context(self, owner, task, query_role):
        self.owner, self.task, self.query_role = owner, task, query_role

    def __call__(self, start, goal, config, blocked):
        start, goal = tuple(start), tuple(goal)
        if any(not math.isfinite(v) or v != int(v) for p in (start, goal) for v in p):
            raise ValueError('D* Lite endpoints must be integer grid coordinates')
        if config['map'] != self.graph.config['map'] or set(map(tuple, blocked)) != self.graph.blocked:
            raise ValueError('D* Lite query graph differs from current observation')
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('D* Lite query must not receive future tables')
        valid = start in self.graph.edges and goal in self.graph.edges
        key = self.owner, goal
        prior_goal = self.last_goal.get(self.owner)
        initial_ns = 0
        if valid:
            new = key not in self.states
            if new:
                stamp = time.perf_counter_ns()
                self.next_state_id += 1
                self.states[key] = self.next_state_id, DStarLite(self.graph, start, goal)
                initial_ns = time.perf_counter_ns() - stamp
            state_id, state = self.states[key]
            result = state.solve(start, self.graph)
            mode = ('INITIALIZE' if prior_goal is None else 'GOAL_RESET') if new else (
                'SAME_GOAL_REPAIR' if result['graph_changed'] else 'START_MOVE' if result['start_moved'] else 'REUSE')
            self.last_goal[self.owner] = goal
        else:
            state_id, mode = None, 'INVALID_ENDPOINT'
            result = {'status': mode, 'path': None, 'distance_m': None, 'search_cost_m': None,
                      'search_ns': 0, 'graph_diff_ns': 0, 'expanded_nodes': 0, 'unique_expanded_nodes': 0,
                      'heap_pops': 0, 'stale_pops': 0, 'edge_checks': 0, 'queue_insertions': 0,
                      'update_vertex_calls': 0, 'rhs_updates': [], 'expansion_trace': []}
        row = {**result, 'query_id': len(self.rows) + 1, 'algorithm': self.algorithm,
               'time_s': self.time_s, 'reason': self.reason, 'start': list(start), 'goal': list(goal),
               'owner': self.owner, 'task_id': self.task, 'query_role': self.query_role,
               'mode': mode, 'state_id': state_id, 'state_initialization_ns': initial_ns,
               'previous_query_goal': list(prior_goal) if prior_goal is not None else None,
               'cache_reactivated': valid and not new and prior_goal is not None and prior_goal != goal,
               'graph_version': self.graph.version, 'graph_sha256': self.graph.sha256,
               'heuristic': 'octile_to_current_start_plus_km',
               'timing_scope': 'changed-tail UpdateVertex and ComputeShortestPath with tracing; excludes graph build/diff, state snapshots, path extraction, reference A*, IO, drawing; state initialization separately counted',
               'work_definition': 'logical queue pops (including REKEY), not A* expanded-node semantics; UpdateVertex calls and stale physical pops separately counted'}
        self.rows.append(row)
        if row['path'] is None:
            return None
        return {**result, 'adjacency': 8, 'search_query_id': row['query_id']}
