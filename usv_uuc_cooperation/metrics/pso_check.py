"""Offline exact references and independent PSO trace/execution checks."""
import hashlib
import math

from usv_uuc_cooperation.metrics.x2_check import certify_graph, reference_distances, observed_cells, check_execution
from usv_uuc_cooperation.planning.x2_search import ObservedGraph


def independent_decode(vector, start, goal):
    anchors = [tuple(math.floor(x + .5) for x in vector[i:i + 2]) for i in range(0, len(vector), 2)] + [tuple(goal)]
    walk, here = [tuple(start)], tuple(start)
    for end in anchors:
        dx, dy = end[0] - here[0], end[1] - here[1]
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        origin = here
        for step in range(1, max(abs(dx), abs(dy)) + 1):
            here = (origin[0] + sx * min(step, abs(dx)), origin[1] + sy * min(step, abs(dy)))
            if here in walk:
                walk = walk[:walk.index(here) + 1]
            else:
                walk.append(here)
    return [list(p) for p in walk]


def check_queries(config, rows, graph_records, policy):
    graphs, references, gaps = {}, {}, []
    for item in graph_records:
        graph = ObservedGraph({'map': config['map']}, item['blocked'], item['version'])
        assert graph.sha256 == item['sha256']
        certify_graph(graph, tuple(config['tasks_m']['T03']))
        graphs[item['version']] = graph
    for number, row in enumerate(rows, 1):
        assert row['query_id'] == number and row['algorithm'] == 'PSO'
        blocked, version = observed_cells(config, row['time_s'])
        graph = graphs[version]
        assert version == row['graph_version'] and blocked == graph.blocked
        assert row['graph_sha256'] == graph.sha256 and row['fallback_search_calls'] == 0
        key = graph.sha256, tuple(row['start']), tuple(row['goal'])
        seed = int(hashlib.sha256(repr((row['main_seed'], key)).encode()).hexdigest()[:16], 16)
        assert seed == row['derived_seed']
        refkey = version, tuple(row['goal'])
        if refkey not in references:
            references[refkey] = reference_distances(graph, refkey[1])
        exact = references[refkey].get(tuple(row['start']))
        assert 0 <= row['evaluations'] <= policy['max_evaluations'] == policy['swarm_size'] * (policy['max_iterations'] + 1)
        assert row['search_ns'] >= 0
        if row['cache_hit']:
            prior = rows[row['optimizer_origin_query_id'] - 1]
            assert prior['query_id'] < number and not prior['cache_hit']
            for field in ('start', 'goal', 'graph_sha256', 'path', 'distance_m', 'best_vector', 'status'):
                assert prior[field] == row[field]
            assert row['evaluations'] == 0 and row['history'] == [] and row['update_samples'] == []
        elif row['status'] != 'INVALID_ENDPOINT':
            history = row['history']
            assert [h['iteration'] for h in history] == list(range(row['iterations'] + 1))
            assert row['evaluations'] == policy['swarm_size'] * len(history)
            previous = [None] * (policy['swarm_size'] + 1)
            for h in history:
                assert h['evaluations'] == policy['swarm_size'] * (h['iteration'] + 1)
                current = h['personal_best_m'] + [h['best_feasible_m']]
                assert len(current) == len(previous)
                for old, new in zip(previous, current):
                    if old is not None:
                        assert new is not None and new <= old + 1e-10
                previous = current
            assert history[-1]['best_feasible_m'] == row['distance_m']
            assert row['iterations'] == policy['max_iterations'] or row['stopped_at_lower_bound']
            for sample in row['update_samples']:
                for d, (lo, hi) in enumerate([config['map']['x_range'], config['map']['y_range']] * policy['waypoints']):
                    x, v = sample['position'][d], sample['velocity'][d]
                    predicted = policy['inertia'] * v + policy['cognitive'] * sample['r1'][d] * (sample['personal_target'][d] - x) + policy['social'] * sample['r2'][d] * (sample['social_target'][d] - x)
                    limit = (hi - lo) * policy['velocity_fraction']
                    predicted = max(-limit, min(limit, predicted))
                    assert math.isclose(predicted, sample['next_velocity'][d], abs_tol=1e-10)
                    assert math.isclose(max(lo, min(hi, x + predicted)), sample['next_position'][d], abs_tol=1e-10)
        if row['path'] is not None:
            assert row['status'] == 'REACHABLE' and exact is not None
            vector, path = row['best_vector'], row['path']
            assert len(vector) == policy['waypoints'] * 2
            assert all(lo <= v <= hi for v, (lo, hi) in zip(vector, [config['map']['x_range'], config['map']['y_range']] * policy['waypoints']))
            assert independent_decode(vector, row['start'], row['goal']) == path
            assert path[0] == row['start'] and path[-1] == row['goal'] and len(path) == len(set(map(tuple, path)))
            assert all(tuple(b) in dict(graph.edges[tuple(a)]) for a, b in zip(path, path[1:]))
            cost = math.fsum(math.dist(a, b) for a, b in zip(path, path[1:]))
            assert math.isclose(cost, row['distance_m'], abs_tol=1e-9) and cost >= exact - 1e-9
            gaps.append({'query_id': number, 'exact_m': exact, 'found_m': cost, 'gap_m': max(0., cost - exact)})
        else:
            assert row['distance_m'] is None and row['status'] in ('INVALID_ENDPOINT', 'NO_FEASIBLE_PATH_WITHIN_BUDGET')
        dx, dy = (abs(row['start'][i] - row['goal'][i]) for i in (0, 1))
        lower = max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        assert math.isclose(lower, row['analytic_lower_bound_m'], abs_tol=1e-9)
        if row['stopped_at_lower_bound']:
            assert row['path'] is not None and math.isclose(row['distance_m'], lower, abs_tol=1e-9)
    return {'status': 'PASS', 'queries': len(rows), 'unique_optimizations': sum(not r['cache_hit'] and r['evaluations'] > 0 for r in rows),
        'failed_queries': sum(r['path'] is None for r in rows), 'fitness_evaluations': sum(r['evaluations'] for r in rows),
        'independent_graph_geometry_decoding_and_exact_cost_gap': 'PASS', 'monotone_feasible_memory_and_sampled_velocity_formula': 'PASS',
        'fixed_seed_budget_cache_and_analytic_stop': 'PASS', 'feasible_gaps': gaps,
        'reference_scope': 'offline complete Dijkstra distances only; never supplied to production optimizer'}


def check_pso(result, rows, graph_records, policy):
    search = check_queries(result['config'], rows, graph_records, policy)
    execution = check_execution(result, rows)
    execution['planned_prefix_budget_actual_origin_and_communication'] = execution.pop('full_remaining_route_budget_actual_origin_and_communication')
    held_count = 0
    for d in result['replans']:
        snap = result['snapshots'][d['time_s']]
        held = d['planning_holds']
        for boat in result['config']['uuc_ids']:
            owned = {t for t, item in snap['tasks'].items() if item['boat_id'] == boat and item['status'] != 'completed'}
            queue = d['output_queues'][boat]
            if snap['boats'][boat]['failure']:
                assert not queue
                continue
            assert len(queue) == len(set(queue)) and set(queue) == owned
            if boat in held:
                h = held[boat]
                assert h['held_task_ids'][0] == h['first_task_id']
                assert queue == h['executable_task_ids'] + h['held_task_ids']
                assert [l['task_id'] for l in d['legs'] if l['boat_id'] == boat] == h['executable_task_ids']
                assert not any(a['boat_id'] == boat and a['old_owner'] != boat for a in d['assignments'])
                assert not h['global_infeasibility_proved']
                held_count += 1
            assert d['projected_total_NEU'][boat] <= snap['boats'][boat]['budget_NEU'] + 1e-9
    return {'status': 'PASS', 'search': search, 'execution': execution,
        'held_tail_ownership_and_executable_prefix_budget': 'PASS', 'boat_hold_decisions': held_count,
        'completion_is_an_observation_not_an_acceptance_requirement': True}
