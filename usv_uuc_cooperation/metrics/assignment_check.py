"""Independent saved-result audit: graph distances, float routes and assignments."""
import heapq
from itertools import product
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes, unsafe_reason


def dijkstra(start, goal, config, mode, blocked):
    start, goal = tuple(start), tuple(goal)
    if unsafe_reason(start, start, config, blocked) or unsafe_reason(goal, goal, config, blocked):
        return None
    distances, heap = {start: 0.0}, [(0.0, start)]
    while heap:
        distance, node = heapq.heappop(heap)
        if distance > distances[node]:
            continue
        if node == goal:
            return distance
        for dx, dy in product((-1, 0, 1), repeat=2):
            if (dx == dy == 0) or (mode == 4 and abs(dx) + abs(dy) != 1):
                continue
            end = (node[0] + dx, node[1] + dy)
            if dx and dy and ((node[0] + dx, node[1]) in blocked or (node[0], node[1] + dy) in blocked):
                continue
            if unsafe_reason(node, end, config, blocked):
                continue
            candidate = distance + math.dist(node, end)
            if candidate < distances.get(end, math.inf):
                distances[end] = candidate
                heapq.heappush(heap, (candidate, end))
    return None


def check_result(result):
    config, reports = result['config'], {}
    points = {**config['starts_m'], **config['tasks_m']}
    blocked = forbidden_nodes(config)
    for key, group in result['groups'].items():
        mode, problem = int(key), group['problem']
        assert problem['adjacency'] == mode
        assert problem['scale'] == config['scale']
        assert problem['boats'] == config['boats'] and problem['tasks'] == config['tasks']
        assert problem['capacity'] == config['capacity']
        assert problem['per_coefficient_error_bound_m'] == .5 / config['scale']
        assert problem['selected_sum_error_bound_m'] == len(config['tasks']) * .5 / config['scale']
        shortest_errors = []
        for name, route in problem['routes'].items():
            a, b = name.split('->')
            reference = dijkstra(points[a], points[b], config, mode, blocked)
            assert (route is None) == (reference is None)
            if route is None:
                continue
            assert route['adjacency'] == mode
            assert route['path'][0] == points[a] and route['path'][-1] == points[b]
            length = math.fsum(math.dist(x, y) for x, y in zip(route['path'], route['path'][1:]))
            for x, y in zip(route['path'], route['path'][1:]):
                dx, dy = abs(x[0]-y[0]), abs(x[1]-y[1])
                assert max(dx, dy) == 1 and (mode == 8 or dx + dy == 1)
                assert unsafe_reason(x, y, config, blocked) is None
                if dx and dy:
                    assert (x[0], y[1]) not in blocked and (y[0], x[1]) not in blocked
            assert math.isclose(length, route['distance_m'], abs_tol=1e-9, rel_tol=0)
            assert math.isclose(length, reference, abs_tol=1e-9, rel_tol=0)
            shortest_errors.append(abs(length-reference))
        errors = []
        for i, boat in enumerate(config['boats']):
            for j, task in enumerate(config['tasks']):
                route = problem['routes'][f'{boat}->{task}']
                assert problem['reachable'][i][j] == (route is not None)
                if route is None:
                    assert problem['cost_m'][i][j] is None and problem['cost_int'][i][j] == 0
                    continue
                c, ci = problem['cost_m'][i][j], problem['cost_int'][i][j]
                assert math.isclose(c, route['distance_m'], abs_tol=1e-9, rel_tol=0)
                assert isinstance(ci, int) and ci == math.floor(c * problem['scale'] + .5)
                error = ci / problem['scale'] - c
                assert abs(error) <= .5 / problem['scale'] + 1e-12
                assert math.isclose(error, problem['coefficient_error_m'][i][j], abs_tol=1e-12, rel_tol=0)
                errors.append(abs(error))
        report = {'status': 'PASS', 'dijkstra_route_count': len(problem['routes']),
                  'max_shortest_path_error_m': max(shortest_errors, default=0),
                  'max_coefficient_error_m': max(errors, default=0)}
        solver, route = group['solver'], group['route_budget']
        if solver is not None:
            feasible = []
            for owners in product(range(len(config['boats'])), repeat=len(config['tasks'])):
                if any(owners.count(i) > cap for i, cap in enumerate(config['capacity'])):
                    continue
                if not all(problem['reachable'][i][j] for j, i in enumerate(owners)):
                    continue
                feasible.append({'owners': list(owners),
                                 'integer': sum(problem['cost_int'][i][j] for j, i in enumerate(owners)),
                                 'float_m': math.fsum(problem['cost_m'][i][j] for j, i in enumerate(owners))})
            if solver['assignment'] is None:
                assert route is None and solver['matrix'] is None
                if solver['status'] == 'INFEASIBLE':
                    assert not feasible
                report['solver_status'] = solver['status']
                reports[key] = report
                continue
            assert solver['status'] in ('OPTIMAL', 'FEASIBLE')
            owners = [next(i for i, b in enumerate(config['boats']) if t in solver['assignment'][b]) for t in config['tasks']]
            selected = next(v for v in feasible if v['owners'] == owners)
            expected_matrix = [[int(owner == i) for owner in owners] for i in range(len(config['boats']))]
            assert solver['matrix'] == expected_matrix
            assert solver['objective_int'] == selected['integer']
            assert solver['scaled_objective_m'] == selected['integer'] / problem['scale']
            assert math.isclose(solver['float_start_task_sum_m'], selected['float_m'], abs_tol=1e-9, rel_tol=0)
            error = selected['integer'] / problem['scale'] - selected['float_m']
            assert math.isclose(error, solver['selected_sum_error_m'], abs_tol=1e-12, rel_tol=0)
            assert abs(error) <= len(config['tasks']) * .5 / problem['scale'] + 1e-12
            float_best = min(v['float_m'] for v in feasible)
            if solver['status'] == 'OPTIMAL':
                assert selected['integer'] == min(v['integer'] for v in feasible)
                assert selected['float_m'] - float_best <= len(config['tasks']) / problem['scale'] + 1e-9
            report.update(solver_status=solver['status'], feasible_assignments=feasible,
                          scaled_optimum_int=min(v['integer'] for v in feasible),
                          float_optimum_m=float_best, selected_float_regret_m=selected['float_m']-float_best,
                          selected_sum_error_m=error)
        assert route['adjacency'] == mode
        if result['phase'] == 'fixed':
            assert route['queues'] == config['fixed_queues']
        total, budget_flags = [], []
        flat = [t for q in route['queues'].values() for t in q]
        assert sorted(flat) == sorted(config['tasks'])
        for i, boat in enumerate(config['boats']):
            row = route['boats'][boat]; queue = route['queues'][boat]
            assert row['adjacency'] == mode and row['queue'] == queue
            assert len(queue) <= config['capacity'][i]
            if solver is not None:
                assert sorted(queue) == sorted(solver['assignment'][boat])
                tail, remaining = boat, set(queue)
                for task in queue:
                    candidates = [(problem['routes'][f'{tail}->{t}']['distance_m'], t) for t in remaining
                                  if problem['routes'][f'{tail}->{t}'] is not None]
                    assert task == min(candidates)[1]
                    remaining.remove(task); tail = task
            expected_path = [config['starts_m'][boat]]
            assert len(row['legs']) == len(queue)
            for a, b, leg in zip([boat, *queue], queue, row['legs']):
                assert leg == {'from': a, 'to': b, **problem['routes'][f'{a}->{b}']}
                expected_path.extend(leg['path'][1:])
            assert row['path'] == expected_path
            length = math.fsum(math.dist(a, b) for a, b in zip(expected_path, expected_path[1:]))
            assert math.isclose(length, row['route_distance_m'], abs_tol=1e-9, rel_tol=0)
            energy = config['energy_coefficient_NEU_per_m'] * length
            assert math.isclose(energy, row['predicted_energy_NEU'], abs_tol=1e-9, rel_tol=0)
            assert row['budget_NEU'] == config['budget_NEU'][boat]
            assert math.isclose(row['predicted_remaining_NEU'], row['budget_NEU']-energy, abs_tol=1e-9, rel_tol=0)
            flag = energy <= row['budget_NEU'] + 1e-9
            assert row['budget_ok'] == flag
            budget_flags.append(flag); total.append(length)
        assert math.isclose(math.fsum(total), route['route_total_m'], abs_tol=1e-9, rel_tol=0)
        assert route['all_budget_ok'] == all(budget_flags)
        reports[key] = report
    assert set(result['groups']) == {'4', '8'}
    if result['phase'] == 'fixed':
        assert result['groups']['4']['route_budget']['queues'] == result['groups']['8']['route_budget']['queues']
    return {'status': 'PASS', 'phase': result['phase'], 'groups': reports,
            'scope': 'saved static cost/route/budget audit; not learner acceptance or actual execution'}
