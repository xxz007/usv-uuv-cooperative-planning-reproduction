"""X1 static cost propagation; the accepted P4 entry remains unchanged."""
from decimal import Decimal, ROUND_HALF_UP
from itertools import permutations
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.planning.grid_planner import plan
from usv_uuc_cooperation.scheduling.ortools_scheduler import ortools_reference


def scaled_cost(value, scale):
    if not math.isfinite(value) or value < 0 or not isinstance(scale, int) or scale <= 0:
        raise ValueError("Expected a nonnegative finite cost and positive integer scale")
    return int((Decimal(str(value)) * scale).quantize(Decimal('1'), rounding=ROUND_HALF_UP))


def build_problem(config, adjacency):
    blocked = forbidden_nodes(config)
    points = {**config['starts_m'], **config['tasks_m']}
    routes = {f'{a}->{b}': plan(points[a], points[b], config, blocked, adjacency=adjacency)
              for a, b in permutations(points, 2) if b in config['tasks_m']}
    boats, tasks, scale = config['boats'], config['tasks'], config['scale']
    distances = [[routes[f'{b}->{t}']['distance_m'] if routes[f'{b}->{t}'] else None
                  for t in tasks] for b in boats]
    coefficients = [[scaled_cost(c, scale) if c is not None else 0 for c in row] for row in distances]
    errors = [[i / scale - c if c is not None else None for i, c in zip(ints, row)]
              for ints, row in zip(coefficients, distances)]
    return {'adjacency': adjacency, 'boats': boats, 'tasks': tasks,
            'capacity': config['capacity'], 'cost_m': distances, 'cost_int': coefficients,
            'reachable': [[c is not None for c in row] for row in distances],
            'scale': scale, 'rounding': 'Decimal(str(cost_m))*scale; ROUND_HALF_UP',
            'coefficient_error_m': errors, 'per_coefficient_error_bound_m': .5 / scale,
            'selected_sum_error_bound_m': len(tasks) * .5 / scale,
            'routes': routes, 'scope': 'minimum sum of start-to-task distances; not full-route/makespan/energy optimum; budget checked after assignment'}


def solve_assignment(problem):
    result = ortools_reference(problem)
    result['assignment'] = None
    result['scaled_objective_m'] = None
    result['float_start_task_sum_m'] = None
    if result['matrix'] is None:
        return result
    matrix = result['matrix']
    tasks, boats = problem['tasks'], problem['boats']
    if (any(sum(row[j] for row in matrix) != 1 for j in range(len(tasks)))
            or any(sum(row) > cap for row, cap in zip(matrix, problem['capacity']))
            or any(x not in (0, 1) or (x and not r) for row, flags in zip(matrix, problem['reachable']) for x, r in zip(row, flags))):
        raise ValueError('Invalid CP-SAT assignment')
    integer = sum(x * c for row, costs in zip(matrix, problem['cost_int']) for x, c in zip(row, costs))
    if integer != result['objective_int']:
        raise ValueError('CP-SAT objective differs from independent integer sum')
    actual = math.fsum(c for row, costs in zip(matrix, problem['cost_m']) for x, c in zip(row, costs) if x)
    result.update(assignment={b: [t for t, x in zip(tasks, row) if x] for b, row in zip(boats, matrix)},
                  objective_int=integer, scaled_objective_m=integer / problem['scale'],
                  float_start_task_sum_m=actual, selected_sum_error_m=integer / problem['scale'] - actual,
                  optimality_scope='scaled start-to-task assignment model only')
    return result


def nearest_order(boat, assigned, problem):
    tail, remaining, order = boat, set(assigned), []
    while remaining:
        candidates = [(problem['routes'][f'{tail}->{t}']['distance_m'], t) for t in sorted(remaining)
                      if problem['routes'][f'{tail}->{t}'] is not None]
        if not candidates:
            raise ValueError(f'No reachable remaining task for {boat}')
        _, task = min(candidates)
        order.append(task)
        remaining.remove(task)
        tail = task
    return order


def route_budget(config, problem, queues):
    if set(queues) != set(config['boats']):
        raise ValueError('Queue boats differ from config')
    scheduled = [t for q in queues.values() for t in q]
    if sorted(scheduled) != sorted(config['tasks']) or any(len(queues[b]) > cap for b, cap in zip(config['boats'], config['capacity'])):
        raise ValueError('Queues must cover each task once within capacity')
    rows = {}
    for boat, queue in queues.items():
        full, legs = [config['starts_m'][boat]], []
        for a, b in zip([boat, *queue], queue):
            leg = problem['routes'][f'{a}->{b}']
            if leg is None:
                raise ValueError(f'Unreachable route leg: {a}->{b}')
            if leg['adjacency'] != problem['adjacency']:
                raise ValueError('Route adjacency differs from cost model')
            legs.append({'from': a, 'to': b, **leg})
            full.extend(leg['path'][1:])
        length = math.fsum(math.dist(a, b) for a, b in zip(full, full[1:]))
        if not math.isclose(length, math.fsum(l['distance_m'] for l in legs), abs_tol=1e-9):
            raise ValueError('Full route distance differs from leg sum')
        energy = length * config['energy_coefficient_NEU_per_m']
        budget = config['budget_NEU'][boat]
        rows[boat] = {'adjacency': problem['adjacency'], 'queue': queue, 'legs': legs, 'path': full,
                      'route_distance_m': length, 'predicted_energy_NEU': energy,
                      'budget_NEU': budget, 'predicted_remaining_NEU': budget - energy,
                      'budget_ok': energy <= budget + 1e-9}
    return {'adjacency': problem['adjacency'], 'queues': queues, 'boats': rows,
            'route_total_m': math.fsum(r['route_distance_m'] for r in rows.values()),
            'all_budget_ok': all(r['budget_ok'] for r in rows.values()),
            'scope': 'static UUV geometric routes, no return; E=cL prediction, not actual execution; budget is post-solve validation'}
