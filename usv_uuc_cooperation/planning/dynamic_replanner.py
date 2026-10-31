"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P4/P4-L3/源码/p4_l3_dynamic.py
"""
import copy
import time
from usv_uuc_cooperation.planning.astar import astar



def snapshot(states, tasks, accepted, queues):
    return {'boats': copy.deepcopy(states), 'tasks': copy.deepcopy(tasks),
            'accepted_task_ids': {b: sorted(v) for b, v in accepted.items()},
            'queues': copy.deepcopy(queues)}


def replan(config, blocked, states, tasks, accepted, t, reason, fixed=None, previous_queues=None):
    """Only current known tasks/state enter here; event schedule is not an argument."""
    stamp = time.perf_counter()
    ids = config['uuc_ids']
    coefficient = config['energy']['uuc_coefficient_NEU_per_m']
    queues, plans = {b: [] for b in ids}, {b: [] for b in ids}
    tails = {b: list(states[b]['position_m']) for b in ids}
    projected = {b: states[b]['energy_NEU'] for b in ids}
    remaining = {k for k, v in tasks.items() if v['status'] != 'completed'}
    inputs = snapshot(states, tasks, accepted, previous_queues if previous_queues is not None else queues)
    assignments, candidate_checks = [], []

    def append(b, task, route, progress=0):
        item = tasks[task]
        old_owner = item['boat_id']
        if progress == 0:
            item.update(boat_id=b, status='pending', arrival_s=None, service_progress_s=0)
        accepted[b].add(task)
        queues[b].append(task)
        for a, end in zip(route['path'], route['path'][1:]):
            plans[b].append({'action': 'move', 'task_id': task, 'end': list(end)})
        plans[b].extend({'action': 'service', 'task_id': task, 'end': list(item['position_m'])}
                        for _ in range(config['service_s_per_task'] - progress))
        tails[b] = list(item['position_m'])
        projected[b] += coefficient * route['distance_m']
        assignments.append({'task_id': task, 'boat_id': b, 'old_owner': old_owner,
                            'from_m': list(route['path'][0]), 'route_distance_m': route['distance_m'],
                            'projected_total_NEU': projected[b], 'locked_service': progress > 0})
        remaining.remove(task)

    # Continuous service is protected when an arrival triggers replanning.
    for task in sorted(remaining.copy()):
        item = tasks[task]
        if item['service_progress_s'] > 0:
            b = item['boat_id']
            assert not states[b]['failure']
            assert tails[b] == item['position_m']
            append(b, task, {'path': [tails[b]], 'distance_m': 0}, item['service_progress_s'])
    if fixed is not None:
        for b, sequence in fixed.items():
            for task in sequence:
                route = astar(tails[b], tasks[task]['position_m'], config['map'], blocked)
                assert route is not None
                assert len(accepted[b] | {task}) <= config['capacity_per_uuc']
                assert projected[b] + coefficient * route['distance_m'] <= states[b]['budget_NEU'] + 1e-10
                append(b, task, route)
    else:
        while remaining:
            candidates = []
            for b in ids:
                if states[b]['failure']:
                    continue
                for task in sorted(remaining):
                    capacity_ok = len(accepted[b] | {task}) <= config['capacity_per_uuc']
                    route = astar(tails[b], tasks[task]['position_m'], config['map'], blocked)
                    predicted = projected[b] + coefficient * route['distance_m'] if route else None
                    budget_ok = predicted is not None and predicted <= states[b]['budget_NEU'] + 1e-10
                    candidate_checks.append({'boat_id': b, 'task_id': task, 'from_m': list(tails[b]),
                                             'projected_energy_before_NEU': projected[b],
                                             'route_distance_m': route['distance_m'] if route else None,
                                             'predicted_total_NEU': predicted,
                                             'budget_NEU': states[b]['budget_NEU'],
                                             'capacity_ok': capacity_ok, 'budget_ok': budget_ok,
                                             'reachable': route is not None})
                    if capacity_ok and route and budget_ok:
                        candidates.append((route['distance_m'], b, task, route))
            if not candidates:
                break
            _, b, task, route = min(candidates, key=lambda v: v[:3])
            append(b, task, route)
    for task in remaining:
        tasks[task].update(boat_id=None, status='unassigned', arrival_s=None, service_progress_s=0)
    upper = (sum(v['status'] == 'completed' and states[v['boat_id']]['failure'] is not None
                 for v in tasks.values())
             + config['capacity_per_uuc'] * sum(not states[b]['failure'] for b in ids))
    failure = None
    if remaining:
        proved = len(tasks) > upper
        failure = {'reason': 'TOTAL_CAPACITY_UPPER_BOUND' if proved else 'HEURISTIC_NO_FEASIBLE_ASSIGNMENT',
                   'global_infeasibility_proved': proved, 'known_demand': len(tasks),
                   'optimistic_completion_upper_bound': upper, 'unassigned_tasks': sorted(remaining),
                   'scope': 'current observed mission/state, fixed per-boat quotas; no future resources assumed'}
    return queues, plans, {'time_s': t, 'reason': reason, 'input_state': inputs,
                          'assignments': assignments, 'candidate_checks': candidate_checks,
                          'output_queues': copy.deepcopy(queues),
                          'projected_total_NEU': projected, 'planning_failure': failure,
                          'wall_time_s': time.perf_counter() - stamp,
                          'timing_scope': 'current-state allocation and A* plan construction; excludes execution/output'}

