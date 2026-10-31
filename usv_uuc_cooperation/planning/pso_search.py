"""Waypoint PSO with a deterministic no-search decoder and feasible memories."""
import copy
import hashlib
import math
import random
import time

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.planning.x2_search import ObservedGraph


def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def decode(vector, start, goal, graph):
    anchors = [tuple(int(math.floor(v + .5)) for v in vector[i:i + 2])
               for i in range(0, len(vector), 2)] + [goal]
    path, indices, node = [start], {start: 0}, start
    for end in anchors:
        while node != end:
            node = (node[0] + ((end[0] > node[0]) - (end[0] < node[0])),
                    node[1] + ((end[1] > node[1]) - (end[1] < node[1])))
            if node in indices:
                keep = indices[node] + 1
                for old in path[keep:]:
                    del indices[old]
                del path[keep:]
            else:
                indices[node] = len(path)
                path.append(node)
    invalid = sum((a, b) not in graph.arc_costs for a, b in zip(path, path[1:]))
    distance = math.fsum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
    return {'feasible': invalid == 0, 'invalid_edges': invalid, 'distance_m': distance,
            'path': [list(p) for p in path]}


def remember_feasible(previous, vector, candidate):
    if candidate['feasible'] and (previous is None or candidate['distance_m'] < previous['distance_m'] - 1e-10):
        return {**candidate, 'vector': list(vector)}
    return previous


def initial_vectors(start, goal, graph, policy, rng):
    k, m = policy['waypoints'], graph.config['map']
    sx, sy = start
    gx, gy = goal
    xmin, xmax = m['x_range']
    ymin, ymax = m['y_range']
    templates = [[(gx, sy)], [(sx, gy)], [(sx, ymin), (gx, ymin)], [(sx, ymax), (gx, ymax)],
                 [(xmin, sy), (xmin, gy)], [(xmax, sy), (xmax, gy)]]
    for x in (xmin, xmax):
        for y in (ymin, ymax):
            templates.extend([[(x, sy), (x, y), (gx, y)], [(sx, y), (x, y), (x, gy)]])
    vectors = [[coordinate for j in range(1, k + 1) for coordinate in
                (sx + (gx - sx) * j / (k + 1), sy + (gy - sy) * j / (k + 1))]]
    for anchors in templates:
        vectors.append([v for p in anchors + [goal] * (k - len(anchors)) for v in p])
    bounds = [m['x_range'], m['y_range']] * k
    while len(vectors) < policy['swarm_size']:
        vectors.append([rng.uniform(lo, hi) for lo, hi in bounds])
    return vectors[:policy['swarm_size']]


def optimize(start, goal, graph, policy, seed):
    stamp = time.perf_counter_ns()
    rng = random.Random(seed)
    bounds = [graph.config['map']['x_range'], graph.config['map']['y_range']] * policy['waypoints']
    limits = [(hi - lo) * policy['velocity_fraction'] for lo, hi in bounds]
    positions = initial_vectors(start, goal, graph, policy, rng)
    velocities = [[rng.uniform(-v, v) for v in limits] for _ in positions]
    personal = [None] * len(positions)
    best, guide, history, samples = None, None, [], []
    evaluations, restart_count = 0, 0
    lower_bound = octile(start, goal)
    for iteration in range(policy['max_iterations'] + 1):
        current = [decode(x, start, goal, graph) for x in positions]
        evaluations += len(current)
        for i, candidate in enumerate(current):
            personal[i] = remember_feasible(personal[i], positions[i], candidate)
            best = remember_feasible(best, positions[i], candidate)
            rank = (candidate['invalid_edges'], candidate['distance_m'])
            if guide is None or rank < guide['rank']:
                guide = {'rank': rank, 'vector': list(positions[i])}
        history.append({'iteration': iteration, 'evaluations': evaluations,
            'best_feasible_m': best['distance_m'] if best else None,
            'personal_best_m': [p['distance_m'] if p else None for p in personal],
            'current_feasible_particles': sum(c['feasible'] for c in current),
            'best_infeasible_guide_rank': list(guide['rank'])})
        if iteration == policy['max_iterations'] or (best and best['distance_m'] <= lower_bound + 1e-10):
            break
        leader = best['vector'] if best else guide['vector']
        restart = set()
        if (iteration + 1) % policy['restart_every'] == 0:
            count = max(1, int(len(positions) * policy['restart_fraction']))
            worst = sorted(range(len(current)), key=lambda i: (not current[i]['feasible'],
                current[i]['invalid_edges'], current[i]['distance_m'], i), reverse=True)
            restart = set(worst[:count])
        for i, x in enumerate(positions):
            if i in restart:
                positions[i] = [rng.uniform(lo, hi) for lo, hi in bounds]
                velocities[i] = [0.] * len(bounds)
                restart_count += 1
                continue
            own = personal[i]['vector'] if personal[i] else list(x)
            sample = {'iteration': iteration, 'particle': i, 'position': list(x), 'velocity': list(velocities[i]),
                      'personal_target': list(own), 'social_target': list(leader), 'r1': [], 'r2': []} if i == 0 else None
            for d, (lo, hi) in enumerate(bounds):
                r1, r2 = rng.random(), rng.random()
                v = (policy['inertia'] * velocities[i][d] + policy['cognitive'] * r1 * (own[d] - x[d])
                     + policy['social'] * r2 * (leader[d] - x[d]))
                velocities[i][d] = min(limits[d], max(-limits[d], v))
                x[d] = min(hi, max(lo, x[d] + velocities[i][d]))
                if sample is not None:
                    sample['r1'].append(r1)
                    sample['r2'].append(r2)
            if sample is not None:
                sample.update(next_position=list(x), next_velocity=list(velocities[i]))
                samples.append(sample)
    elapsed = time.perf_counter_ns() - stamp
    return {'status': 'REACHABLE' if best else 'NO_FEASIBLE_PATH_WITHIN_BUDGET',
        'path': best['path'] if best else None, 'distance_m': best['distance_m'] if best else None,
        'search_cost_m': best['distance_m'] if best else None, 'best_vector': best['vector'] if best else None,
        'search_ns': elapsed, 'evaluations': evaluations, 'iterations': history[-1]['iteration'],
        'restart_count': restart_count, 'history': history, 'update_samples': samples,
        'analytic_lower_bound_m': lower_bound, 'stopped_at_lower_bound': bool(best and best['distance_m'] <= lower_bound + 1e-10),
        'infeasible_not_proof_of_unreachability': best is None}


class PSOSession:
    algorithm = 'PSO'

    def __init__(self, config, policy, main_seed=None):
        self.policy = copy.deepcopy(policy)
        self.main_seed = policy['main_seed'] if main_seed is None else main_seed
        self.rows, self.graphs, self.cache = [], [], {}
        self.time_s, self.reason = 0, 'INITIAL'
        self.owner, self.task, self.query_role = None, None, None
        self.observe({'map': config['map']}, forbidden_nodes(config), 0)

    def observe(self, config, blocked, version):
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('PSO observation must contain current physical data only')
        self.graph = ObservedGraph({'map': config['map']}, blocked, version)
        self.graph.arc_costs = {(a, b): w for a, edges in self.graph.edges.items() for b, w in edges}
        self.graphs.append({'version': version, 'sha256': self.graph.sha256,
            'blocked': [list(p) for p in sorted(blocked)], 'build_ns': self.graph.build_ns})

    def context(self, time_s, reason):
        self.time_s, self.reason = time_s, reason

    def route_context(self, owner, task, query_role):
        self.owner, self.task, self.query_role = owner, task, query_role

    def __call__(self, start, goal, config, blocked):
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('PSO query must not receive future tables')
        if config['map'] != self.graph.config['map'] or set(map(tuple, blocked)) != self.graph.blocked:
            raise ValueError('PSO query differs from current observed graph')
        if any(not math.isfinite(v) or v != int(v) for p in (start, goal) for v in p):
            raise ValueError('PSO endpoints must be integer grid coordinates')
        start, goal = tuple(map(int, start)), tuple(map(int, goal))
        key = self.graph.sha256, start, goal
        seed = int(hashlib.sha256(repr((self.main_seed, key)).encode()).hexdigest()[:16], 16)
        query_id = len(self.rows) + 1
        hit = key in self.cache
        if start not in self.graph.edges or goal not in self.graph.edges:
            origin = None
            result = {'status': 'INVALID_ENDPOINT', 'path': None, 'distance_m': None, 'search_cost_m': None,
                'best_vector': None, 'search_ns': 0, 'evaluations': 0, 'iterations': 0, 'restart_count': 0,
                'history': [], 'update_samples': [], 'analytic_lower_bound_m': octile(start, goal), 'stopped_at_lower_bound': False}
        elif hit:
            origin, saved = self.cache[key]
            result = {**copy.deepcopy(saved), 'search_ns': 0, 'evaluations': 0, 'iterations': 0,
                      'restart_count': 0, 'history': [], 'update_samples': []}
        else:
            origin = query_id
            result = optimize(start, goal, self.graph, self.policy, seed)
            self.cache[key] = origin, copy.deepcopy(result)
        row = {**result, 'query_id': query_id, 'algorithm': self.algorithm, 'start': list(start), 'goal': list(goal),
            'graph_version': self.graph.version, 'graph_sha256': self.graph.sha256,
            'time_s': self.time_s, 'reason': self.reason, 'owner': self.owner, 'task_id': self.task,
            'query_role': self.query_role, 'mode': 'CACHE_REUSE' if hit else 'OPTIMIZE',
            'cache_hit': hit, 'optimizer_origin_query_id': origin, 'main_seed': self.main_seed, 'derived_seed': seed,
            'evaluation_budget': self.policy['max_evaluations'], 'policy_version': self.policy['version'],
            'work_definition': 'decoded candidate route fitness evaluations; cache hits have zero evaluations',
            'timing_scope': 'initialization, PSO updates/decoding/evaluation and trace; graph build, offline reference, serialization, IO/drawing excluded',
            'fallback_search_calls': 0}
        self.rows.append(row)
        return {**result, 'adjacency': 8, 'search_query_id': query_id} if row['path'] is not None else None
