"""Independent checks of saved initial costs and integrated 4/8 logs."""
import copy
from decimal import Decimal, ROUND_HALF_UP
import heapq
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.metrics.integrated_check import safe_segment, check_result as p4_check


def dijkstra(start, config, blocked):
    mode = config['map']['adjacency']
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    if mode == 8:
        directions += [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    start = tuple(start)
    best, queue = {start: 0.0}, [(0.0, start)]
    while queue:
        g, node = heapq.heappop(queue)
        if g != best[node]:
            continue
        x, y = node
        for dx, dy in directions:
            end = x + dx, y + dy
            if dx and dy and ((x + dx, y) in blocked or (x, y + dy) in blocked):
                continue
            if not safe_segment(node, end, config, blocked):
                continue
            total = g + math.hypot(dx, dy)
            if total < best.get(end, math.inf):
                best[end] = total
                heapq.heappush(queue, (total, end))
    return best


def grid_path_check(path, mode, config, blocked):
    for a, b in zip(path, path[1:]):
        assert all(v == int(v) for p in (a, b) for v in p)
        dx, dy = b[0] - a[0], b[1] - a[1]
        assert max(abs(dx), abs(dy)) == 1
        assert mode == 8 or dx == 0 or dy == 0
        if dx and dy:
            assert (a[0] + dx, a[1]) not in blocked and (a[0], a[1] + dy) not in blocked
        assert safe_segment(a, b, config, blocked)


def check_initial(config, problem, solved):
    blocked, mode = forbidden_nodes(config), config['map']['adjacency']
    assert solved['has_feasible_solution'] and solved['variables_loaded']
    assert problem['adjacency'] == solved['adjacency'] == mode
    assert problem['boats'] == config['uuc_ids'] and problem['tasks'] == sorted(config['tasks_m'])
    assert problem['map'] == config['map'] and problem['tasks_m'] == config['tasks_m']
    assert problem['capacity'] == [config['capacity_per_uuc']] * len(config['uuc_ids'])
    n, scale = len(problem['tasks']), problem['scale']
    matrix = solved['matrix']
    assert len(matrix) == len(problem['boats']) and all(len(row) == n for row in matrix)
    for i, boat in enumerate(problem['boats']):
        best = dijkstra(config['starts_m'][boat], config, blocked)
        for j, task in enumerate(problem['tasks']):
            distance = best.get(tuple(config['tasks_m'][task]))
            assert problem['reachable'][i][j] == (distance is not None)
            assert min(abs(matrix[i][j]), abs(matrix[i][j] - 1)) < 1e-7
            assert matrix[i][j] <= int(problem['reachable'][i][j]) + 1e-7
            route = problem['start_task_routes'][f'{boat}->{task}']
            if distance is None:
                assert route is None
                continue
            assert math.isclose(problem['cost_m'][i][j], distance, abs_tol=1e-9)
            assert route['path'][0] == config['starts_m'][boat] and route['path'][-1] == config['tasks_m'][task]
            assert route['adjacency'] == mode
            grid_path_check(route['path'], mode, config, blocked)
            length = math.fsum(math.dist(a, b) for a, b in zip(route['path'], route['path'][1:]))
            assert math.isclose(length, distance, abs_tol=1e-9)
            expected = int((Decimal(str(problem['cost_m'][i][j])) * scale).quantize(Decimal('1'), rounding=ROUND_HALF_UP))
            assert expected == problem['cost_int'][i][j]
            assert abs(expected / scale - distance) <= .5 / scale + 1e-9
        assert sum(matrix[i]) <= problem['capacity'][i] + 1e-7
    assert all(abs(sum(row[j] for row in matrix) - 1) < 1e-7 for j in range(n))
    integer = sum(x * c for row, costs in zip(matrix, problem['cost_int']) for x, c in zip(row, costs))
    actual = math.fsum(c for row, costs in zip(matrix, problem['cost_m']) for x, c in zip(row, costs) if x > .5)
    assert math.isclose(integer, solved['objective_int'], abs_tol=1e-7)
    assert math.isclose(actual, solved['float_start_task_sum_m'], abs_tol=1e-9)
    assert math.isclose(integer / scale, solved['scaled_objective_m'], abs_tol=1e-9)
    assert abs(integer / scale - actual) <= n * .5 / scale + 1e-9
    assert solved['assignment'] == {b: [t for t, x in zip(problem['tasks'], row) if x > .5]
                                    for b, row in zip(problem['boats'], matrix)}
    if solved['status'] == 'OPTIMAL':
        assert solved['termination_condition'] == 'optimal'
        assert math.isclose(integer, solved['objective_bound_int'], abs_tol=1e-7)
    return {'status': 'PASS', 'adjacency': mode, 'independent_Dijkstra_pairs': len(problem['boats']) * n,
            'binary_coverage_capacity_reach_scaled_and_float_objective': 'PASS',
            'float_start_task_sum_m': actual, 'rounding_error_m': integer / scale - actual,
            'scope': 'integer proxy model only; no full-route or dynamic optimality claim'}


def check_result(result):
    report = p4_check(result)
    config, mode = result['config'], result['config']['map']['adjacency']
    blocked = forbidden_nodes(config)
    assert result['strategy'] == config['x1']['strategy'] == 'DYNAMIC_REPLAN'
    assert result.get('adjacency', mode) == mode
    assert not ({'events', 'current_schedule'} & set(result['planner_config_keys']))
    distances = {b: 0.0 for b in config['uuc_ids'] + ['USV']}
    peak, diagonals = 0.0, 0
    for row in result['steps']:
        b, t = row['boat_id'], row['time_s']
        length = math.dist(row['start_m'], row['end_m'])
        distances[b] += length
        expected_current = next(c['vector_m_per_s'] for c in config['current_schedule'] if c['start_s'] <= t < c['end_s'])
        assert row['current_m_per_s'] == expected_current
        assert row['next_time_s'] == t + config['dt_s']
        peak = max(peak, math.hypot(*row['water_m_per_s']))
        assert math.isclose(distances[b], result['snapshots'][t + 1]['boats'][b]['distance_m'], abs_tol=1e-8)
        coefficient = config['energy']['usv_coefficient_NEU_per_m' if b == 'USV' else 'uuc_coefficient_NEU_per_m']
        predicted = row['energy_before_NEU'] + coefficient * math.dist(row['start_m'], row['candidate_m'])
        assert math.isclose(predicted, row['predicted_total_NEU'], abs_tol=1e-8)
        assert row['budget_check_passed'] == (predicted <= row['budget_NEU'] + 1e-9)
        if row['action'] == 'move' and row['end_m'] == row['waypoint_m']:
            assert row['plan_count_after'] == row['plan_count_before'] - 1
    for decision in result['replans']:
        t = decision['time_s']
        assert decision['observed_task_ids'] == sorted(result['snapshots'][t]['tasks'])
        for b, state in decision.get('input_state', {}).get('boats', {}).items():
            assert state['position_m'] == result['snapshots'][t]['boats'][b]['position_m']
            assert state['energy_NEU'] == result['snapshots'][t]['boats'][b]['energy_NEU']
        first = set()
        for leg in decision['legs']:
            assert leg['adjacency'] == mode
            path, grid = leg['path'], leg['grid_path']
            assert path == [path[0]] + (grid if path[0] != grid[0] else grid[1:])
            assert math.isclose(leg['connector_m'], math.dist(path[0], grid[0]), abs_tol=1e-9)
            if leg['boat_id'] not in first:
                assert path[0] == result['snapshots'][t]['boats'][leg['boat_id']]['position_m']
                first.add(leg['boat_id'])
            assert path[-1] == result['snapshots'][t]['tasks'][leg['task_id']]['position_m']
            grid_path_check(grid, mode, config, blocked)
            length = math.fsum(math.dist(a, b) for a, b in zip(grid, grid[1:]))
            assert math.isclose(length, leg['grid_distance_m'], abs_tol=1e-8)
            assert math.isclose(length + leg['connector_m'], leg['distance_m'], abs_tol=1e-8)
            diagonals += sum(a[0] != b[0] and a[1] != b[1] for a, b in zip(grid, grid[1:]))
        for b, projected in decision['projected_total_NEU'].items():
            actual = result['snapshots'][t]['boats'][b]['energy_NEU']
            estimate = actual + config['energy']['uuc_coefficient_NEU_per_m'] * math.fsum(l['distance_m'] for l in decision['legs'] if l['boat_id'] == b)
            assert math.isclose(projected, estimate, abs_tol=1e-8)
    summary = result['summary']
    for b, distance in distances.items():
        assert math.isclose(summary['boat_totals'][b]['distance_m'], distance, abs_tol=1e-8)
    total = math.fsum(distances.values())
    assert math.isclose(summary['actual_total_distance_m'], total, abs_tol=1e-8)
    rows = result['links']
    assert len(rows) == len(config['uuc_ids']) * config['cutoff_time_s']
    seen = set()
    for row in rows:
        key = row['time_s'], row['boat_id']; assert key not in seen; seen.add(key)
        snap = result['snapshots'][row['time_s']]
        distance = math.dist(snap['boats'][row['boat_id']]['position_m'], snap['boats']['USV']['position_m'])
        assert math.isclose(distance, row['distance_m'], abs_tol=1e-9)
        assert row['connected'] == (distance <= config['communication']['radius_m'])
    assert summary['communication']['connected_boat_seconds'] == sum(r['connected'] for r in rows)
    assert math.isclose(summary['communication']['retention_rate'], sum(r['connected'] for r in rows) / len(rows), abs_tol=1e-9)
    return {**report, 'adjacency': mode, 'planned_grid_diagonals_across_replans': diagonals,
            'actual_total_distance_m': total, 'actual_per_boat_distance_m': distances,
            'water_speed_peak_m_per_s': peak, 'full_remaining_route_budget_actual_origin_and_communication': 'PASS',
            'note': 'planned diagonal count includes replanned legs; not an executed-edge count'}


def check_pair(groups):
    normalized = []
    for mode in ('4', '8'):
        result = groups[mode]
        c = copy.deepcopy(result['config'])
        assert c['map'].pop('adjacency') == int(mode)
        c['map'].pop('movement')
        normalized.append(c)
        assert [s['time_s'] for s in result['snapshots']] == list(range(c['cutoff_time_s'] + 1))
        assert [(e['event_id'], e['time_s']) for e in result['event_log']] == [(e['event_id'], e['time_s']) for e in sorted(c['events'], key=lambda e: (e['time_s'], 0 if e['type'] == 'UUV_FAILURE' else 1, e['event_id']))]
    assert normalized[0] == normalized[1]
    return {'status': 'PASS', 'same_physical_config_strategy_currents_events_budgets_and_times': 'PASS',
            'changed': 'adjacency and required costs/heuristic; resulting assignment/order/response may differ',
            'initial_queues_equal': groups['4']['initial_queues'] == groups['8']['initial_queues'],
            'independent_pair_count': 1, 'main_execution_groups': 2}
