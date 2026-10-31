"""Original D*: Stentz CMU-RI-TR-93-20 §2.1/2.2, h/p and backpointers.

The 1993 printed CLOSED update assigns p after overwriting h. Here INSERT
preserves the prior h before changing it, consistent with the definition of p
and the Basic D* INSERT in CMU-RI-TR-94-37 §2.3 (not Focussed D*).
No g/rhs, km, A* or Dijkstra is used to produce paths. The adapter accepts
only the project's symmetric positive-weight graph; deleted arcs cost infinity.
"""
import heapq
import math
import time

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.planning.x2_search import ObservedGraph

EPS = 1e-10
REFERENCE = 'Stentz 1993 CMU-RI-TR-93-20 §2.1–2.2; prior-h-preserving INSERT cross-checked against 1994 CMU-RI-TR-94-37 Basic D* §2.3'


def finite(value):
    return value if math.isfinite(value) else None


def less(a, b):
    return a < b - EPS


def equal(a, b):
    return a == b or math.isclose(a, b, rel_tol=0, abs_tol=EPS)


class OriginalDStar:
    def __init__(self, graph, start, goal):
        self.graph, self.start, self.goal = graph, start, goal
        m = graph.config['map']
        self.nodes = [(x, y) for x in range(m['x_range'][0], m['x_range'][1] + 1)
                      for y in range(m['y_range'][0], m['y_range'][1] + 1)]
        domain = set(self.nodes)
        self.neighbors = {a: sorted((a[0] + dx, a[1] + dy)
            for dx in (-1, 0, 1) for dy in (-1, 0, 1)
            if (dx or dy) and (a[0] + dx, a[1] + dy) in domain) for a in self.nodes}
        self.h = dict.fromkeys(self.nodes, math.inf)
        self.p = dict.fromkeys(self.nodes, math.inf)
        self.tag = dict.fromkeys(self.nodes, 'NEW')
        self.back = dict.fromkeys(self.nodes, None)
        self.heap, self.active, self.serial = [], {}, 0
        self.costs = self.graph_costs(graph)
        self.insert(goal, 0.0, None, 'INITIAL_GOAL')

    @staticmethod
    def graph_costs(graph):
        costs = {a: dict(edges) for a, edges in graph.edges.items()}
        for a, edges in costs.items():
            for b, w in edges.items():
                if not math.isfinite(w) or w <= 0 or costs.get(b, {}).get(a) != w:
                    raise ValueError('Original D* adapter requires symmetric finite positive arcs')
        return costs

    def cost(self, a, b):
        return self.costs.get(a, {}).get(b, math.inf)

    def insert(self, node, new_h, work, reason):
        old_h, old_p, old_tag = self.h[node], self.p[node], self.tag[node]
        if old_tag == 'NEW':
            self.p[node] = new_h
        elif old_tag == 'OPEN':
            self.p[node] = min(old_p, old_h)
        else:
            # Preserve the previous path cost, before h is replaced.
            self.p[node] = old_h
        self.h[node], self.tag[node] = new_h, 'OPEN'
        key = min(self.h[node], self.p[node])
        self.serial += 1
        self.active[node] = key, self.serial
        heapq.heappush(self.heap, (key, *node, self.serial))
        if work is not None:
            work['queue_insertions'] += 1
            work['updates'].append({'node': list(node), 'reason': reason,
                'tag_before': old_tag, 'h_before_m': finite(old_h), 'p_before_m': finite(old_p),
                'h_after_m': finite(new_h), 'p_after_m': finite(self.p[node]),
                'key_m': finite(key), 'backpointer': list(self.back[node]) if self.back[node] else None})

    def peek(self, work):
        while self.heap:
            key, x, y, serial = self.heap[0]
            if self.active.get((x, y)) == (key, serial):
                return key
            heapq.heappop(self.heap)
            work['stale_pops'] += 1
        return math.inf

    def snapshot(self):
        return {'start': list(self.start), 'goal': list(self.goal),
            'graph_version': self.graph.version, 'graph_sha256': self.graph.sha256,
            'h_p_tag_back': [[*a, finite(self.h[a]), finite(self.p[a]), self.tag[a],
                            list(self.back[a]) if self.back[a] is not None else None] for a in self.nodes],
            'queue': [[*a, finite(key)] for a, (key, _) in sorted(self.active.items())]}

    def process_state(self, work):
        old_key = self.peek(work)
        if not math.isfinite(old_key):
            return
        key, x, y, serial = heapq.heappop(self.heap)
        a = x, y
        assert self.active.pop(a) == (key, serial)
        self.tag[a] = 'CLOSED'
        item = {'node': list(a), 'key_m': key, 'h_before_m': finite(self.h[a]),
                'p_before_m': finite(self.p[a]),
                'category': 'RAISE' if less(self.p[a], self.h[a]) else 'LOWER',
                'first_update': len(work['updates']), 'self_reductions': []}
        # 1993 PROCESS-STATE L6–L11.
        for b in self.neighbors[a]:
            work['edge_checks'] += 1
            candidate = self.h[b] + self.cost(b, a)
            if self.tag[b] == 'CLOSED' and not less(old_key, self.h[b]) and less(candidate, self.h[a]):
                self.back[a], self.h[a] = b, candidate
                item['self_reductions'].append({'via': list(b), 'h_m': finite(candidate)})
        # 1993 L13–L57; updates to h/p are centralized in INSERT.
        for b in self.neighbors[a]:
            work['edge_checks'] += 1
            candidate = self.h[a] + self.cost(a, b)
            if self.tag[b] == 'NEW':
                self.back[b] = a
                self.insert(b, candidate, work, 'DISCOVER')
            elif self.back[b] == a and not equal(self.h[b], candidate):
                self.insert(b, candidate, work, 'BACKPOINTER_DEPENDENCY')
            elif self.back[b] != a and less(candidate, self.h[b]):
                if not less(self.p[a], self.h[a]):
                    self.back[b] = a
                    self.insert(b, candidate, work, 'LOWER_ALTERNATIVE')
                else:
                    self.insert(a, self.h[a], work, 'POSTPONE_RAISE_SOURCE')
            elif (self.back[b] != a and less(self.h[b] + self.cost(b, a), self.h[a])
                  and self.tag[b] == 'CLOSED' and less(old_key, self.h[b])):
                self.insert(b, self.h[b], work, 'POSTPONE_BETTER_NEIGHBOR')
        item.update(h_after_m=finite(self.h[a]), p_after_m=finite(self.p[a]),
                    last_update=len(work['updates']))
        work['expansion_trace'].append(item)

    def solve(self, start, graph):
        before = self.snapshot()
        old_start, old_graph = self.start, self.graph
        work = {'queue_insertions': 0, 'stale_pops': 0, 'edge_checks': 0,
                'updates': [], 'expansion_trace': []}
        stamp = time.perf_counter_ns()
        new_costs = self.graph_costs(graph) if old_graph.sha256 != graph.sha256 else self.costs
        changes = []
        if old_graph.sha256 != graph.sha256:
            for a in self.nodes:
                old, new = self.costs.get(a, {}), new_costs.get(a, {})
                for b in sorted(old.keys() | new.keys()):
                    c, d = old.get(b, math.inf), new.get(b, math.inf)
                    if c != d:
                        changes.append({'from': list(a), 'to': list(b),
                                        'old_cost_m': finite(c), 'new_cost_m': finite(d)})
        self.graph, self.start, self.costs = graph, start, new_costs
        diff_ns = time.perf_counter_ns() - stamp
        stamp = time.perf_counter_ns()
        # Report c(X,Y) is the arc Y -> X. For our a -> b arc, X=b.
        # All changed arcs are applied as one observation before propagation.
        seeds = sorted({tuple(row['to']) for row in changes})
        for node in seeds:
            if self.tag[node] == 'CLOSED':
                self.insert(node, self.h[node], work, 'MODIFY_COST_HEAD')
        seed_ns = time.perf_counter_ns() - stamp
        seeded = self.snapshot()
        stamp = time.perf_counter_ns()
        while math.isfinite(self.peek(work)) and (
                self.tag[start] != 'CLOSED' or less(self.peek(work), self.h[start])):
            self.process_state(work)
            if len(work['expansion_trace']) > 100000:
                raise RuntimeError('Original D* propagation did not terminate')
        search_ns = seed_ns + time.perf_counter_ns() - stamp
        path = None
        if self.tag[start] == 'CLOSED' and math.isfinite(self.h[start]):
            path, seen, node = [list(start)], {start}, start
            while node != self.goal:
                parent = self.back[node]
                if (parent is None or parent in seen or not math.isfinite(self.cost(node, parent))
                        or not equal(self.h[node], self.cost(node, parent) + self.h[parent])):
                    raise RuntimeError('Original D* has an invalid or stale backpointer chain')
                path.append(list(parent))
                seen.add(parent)
                node = parent
        after = self.snapshot()
        trace = work['expansion_trace']
        return {**work, 'status': 'REACHABLE' if path is not None else 'UNREACHABLE',
            'path': path, 'distance_m': math.fsum(self.cost(tuple(a), tuple(b)) for a, b in zip(path, path[1:])) if path else None,
            'search_cost_m': finite(self.h[start]) if path else None,
            'search_ns': search_ns, 'graph_diff_ns': diff_ns,
            'expanded_nodes': len(trace), 'unique_expanded_nodes': len({tuple(e['node']) for e in trace}),
            'heap_pops': len(trace) + work['stale_pops'],
            'changed_edges': changes, 'changed_heads': [list(a) for a in seeds],
            'start_moved': start != old_start, 'graph_changed': old_graph.sha256 != graph.sha256,
            'before_state': before, 'after_edge_updates': seeded, 'after_state': after,
            'state_identity_preserved': True}


class OriginalDStarSession:
    algorithm = 'Dstar'

    def __init__(self, config):
        self.rows, self.graphs, self.states, self.last_goal = [], [], {}, {}
        self.time_s, self.reason = 0, 'LOCAL_INCREMENTAL'
        self.owner, self.task, self.query_role = 'LOCAL', None, 'FIXED_TARGET_SEGMENT'
        self.next_state_id = 0
        self.observe({'map': config['map']}, forbidden_nodes(config), 0)

    def observe(self, config, blocked, version):
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('Original D* receives current physical data only')
        self.graph = ObservedGraph({'map': config['map']}, blocked, version)
        self.graphs.append({'version': version, 'sha256': self.graph.sha256,
            'blocked': [list(a) for a in sorted(blocked)], 'node_count': len(self.graph.edges),
            'build_ns': self.graph.build_ns})

    def context(self, time_s, reason):
        self.time_s, self.reason = time_s, reason

    def route_context(self, owner, task, query_role):
        self.owner, self.task, self.query_role = owner, task, query_role

    def __call__(self, start, goal, config, blocked):
        start, goal = tuple(start), tuple(goal)
        if any(not math.isfinite(v) or v != int(v) for a in (start, goal) for v in a):
            raise ValueError('Original D* endpoints must be integer grid coordinates')
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('Original D* query must not receive future tables')
        if config['map'] != self.graph.config['map'] or set(map(tuple, blocked)) != self.graph.blocked:
            raise ValueError('Original D* query differs from current observation')
        prior = self.last_goal.get(self.owner)
        valid = start in self.graph.edges and goal in self.graph.edges
        key, initial_ns, new = (self.owner, goal), 0, False
        if valid:
            new = key not in self.states
            if new:
                stamp = time.perf_counter_ns()
                self.next_state_id += 1
                self.states[key] = self.next_state_id, OriginalDStar(self.graph, start, goal)
                initial_ns = time.perf_counter_ns() - stamp
            state_id, state = self.states[key]
            result = state.solve(start, self.graph)
            mode = ('INITIALIZE' if prior is None else 'GOAL_RESET') if new else (
                'SAME_GOAL_REPAIR' if result['graph_changed'] else 'START_MOVE' if result['start_moved'] else 'REUSE')
            self.last_goal[self.owner] = goal
        else:
            state_id, mode = None, 'INVALID_ENDPOINT'
            result = {'status': mode, 'path': None, 'distance_m': None, 'search_cost_m': None,
                'search_ns': 0, 'graph_diff_ns': 0, 'expanded_nodes': 0, 'unique_expanded_nodes': 0,
                'heap_pops': 0, 'stale_pops': 0, 'queue_insertions': 0, 'edge_checks': 0,
                'updates': [], 'expansion_trace': []}
        row = {**result, 'query_id': len(self.rows) + 1, 'algorithm': self.algorithm,
            'time_s': self.time_s, 'reason': self.reason, 'start': list(start), 'goal': list(goal),
            'owner': self.owner, 'task_id': self.task, 'query_role': self.query_role,
            'state_id': state_id, 'mode': mode, 'state_initialization_ns': initial_ns,
            'previous_query_goal': list(prior) if prior is not None else None,
            'cache_reactivated': valid and not new and prior is not None and prior != goal,
            'graph_version': self.graph.version, 'graph_sha256': self.graph.sha256,
            'reference': REFERENCE, 'heuristic': 'none; scalar min(h,p) toward fixed goal',
            'work_definition': 'PROCESS-STATE logical pops; RAISE/LOWER categories and insertion reasons logged; not equivalent to A* or D* Lite pops',
            'timing_scope': 'MODIFY-COST queue seeding and PROCESS-STATE with trace; graph build/diff, state snapshots, extraction, offline references, IO and drawing excluded; initialization separate'}
        self.rows.append(row)
        return {**result, 'adjacency': 8, 'search_query_id': row['query_id']} if row['path'] is not None else None
