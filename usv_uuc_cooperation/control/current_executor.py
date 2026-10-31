"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P4/P4-L2/源码/p4_l2_current_sim.py
"""
import math
from usv_uuc_cooperation.env.acoustic_comm import communication_graph
from usv_uuc_cooperation.metrics.evaluator import metrics
from usv_uuc_cooperation.env.obstacle import unsafe_reason



def waypoint_candidate(start, goal, current, limit, exact_zero_endpoint=True):
    distance = math.dist(start, goal)
    if not distance:
        return list(start)
    if current == [0, 0] or current == (0, 0):
        factor = min(distance, limit)/distance
        if exact_zero_endpoint and distance <= limit:
            return list(goal)
        return [a+factor*(b-a) for a,b in zip(start,goal)]
    direction = [(goal[k]-start[k])/distance for k in (0,1)]
    parallel = sum(current[k]*direction[k] for k in (0,1))
    perpendicular2 = max(0.0, sum(c*c for c in current)-parallel*parallel)
    ground_limit = parallel+math.sqrt(limit*limit-perpendicular2)
    if distance <= ground_limit:
        return list(goal)
    return [start[k]+ground_limit*direction[k] for k in (0,1)]


def simulate(config, sequences, plans, blocked, budgets, currents):
    assert config['dt_s'] == 1 and all(config['speed_m_per_s'][b] == 1 for b in sequences)
    horizon = config['cutoff_time_s']
    assert horizon == config['observation_duration_s'] and isinstance(horizon, int)
    assert len(currents) == horizon
    for current in currents:
        if len(current) != 2 or any(not math.isfinite(x) for x in current) or math.hypot(*current) > .2 + 1e-12:
            raise ValueError('Unsupported current: use finite 2D flow with norm <=0.2m/s')
    ids = list(sequences)
    states = {b: {'position_m': list(config['starts_m'][b]), 'distance_m': 0.0, 'energy_NEU': 0.0,
                  'failure': None} for b in ids + ['USV']}
    tasks = {task: {'boat_id': b, 'status': 'pending', 'arrival_s': None, 'service_progress_s': 0,
                    'completion_s': None, 'deadline_s': config['deadline_s_per_task'],
                    'deadline_met': False, 'failure_reason': None}
             for b, sequence in sequences.items() for task in sequence}
    assert len(tasks) == sum(len(s) for s in sequences.values()), 'Duplicate assignment'
    assert all(len(s) <= config['capacity_per_uuc'] for s in sequences.values())
    cursor = {b: 0 for b in ids}
    controls = []
    previous = {b: True for b in ids}
    ages = {b: 0 for b in ids}
    steps, links, snapshots = [], [], []
    for t in range(horizon + 1):
        positions = {b: list(states[b]['position_m']) for b in ids}
        usv = list(states['USV']['position_m'])
        graph, distances = communication_graph(usv, positions, config['communication']['radius_m'])
        connected = {b: graph.has_edge('USV', b) for b in ids}
        snapshots.append({'time_s': t, 'positions_m': {**positions, 'USV': usv}, 'connected': connected,
                          'completed_tasks': sorted(k for k, v in tasks.items() if v['status'] == 'completed')})
        if t == horizon:
            break
        for b in ids:
            ages[b] = 0 if connected[b] or previous[b] else ages[b] + 1
            links.append({'time_s': t, 'interval_end_s': t + 1, 'boat_id': b,
                          'x_m': positions[b][0], 'y_m': positions[b][1],
                          'usv_x_m': usv[0], 'usv_y_m': usv[1], 'distance_m': distances[b],
                          'connected': connected[b], 'outage_age_s': ages[b]})
        current = currents[t]
        lost = [b for b in ids if not connected[b]]
        target_boat = min(lost, key=lambda b: (-ages[b], distances[b], b)) if lost else ''
        target = positions[target_boat] if target_boat else usv
        d = math.dist(usv, target)
        candidate = waypoint_candidate(usv, target, current, config['speed_m_per_s']['USV'], exact_zero_endpoint=False)
        # Candidate relay target is captured from t actual states, before any UUC moves.
        for b in ids + ['USV']:
            state = states[b]
            start = list(state['position_m'])
            task, arrival, selected, tx, ty = '', False, '', '', ''
            index_before = cursor.get(b)
            waypoint = None
            if state['failure']:
                action, end = 'failed_hold', start
            elif b == 'USV':
                action, end = 'relay_move' if d else 'relay_hold', candidate
                selected, tx, ty = target_boat, target[0], target[1]
            elif cursor[b] < len(plans[b]):
                op = plans[b][cursor[b]]
                action, task = op['action'], op['task']
                waypoint = list(op['end'])
                if action == 'move':
                    end = waypoint_candidate(start, waypoint, current, config['speed_m_per_s'][b])
                    arrival = op['arrived'] and end == waypoint
                elif action == 'service':
                    assert start == waypoint == config['tasks_m'][task], 'Service requires actual waypoint arrival'
                    end = start
                else:
                    raise ValueError(f'Unsupported waypoint action: {action}')
            else:
                action, end = 'finished_hold', start
            coefficient = config['energy']['usv_coefficient_NEU_per_m' if b == 'USV' else 'uuc_coefficient_NEU_per_m']
            planned = math.dist(start, end)
            before = state['energy_NEU']
            predicted = before + coefficient * planned
            safe_error = unsafe_reason(start, end, config, blocked)
            budget_ok = predicted <= budgets[b] + 1e-10
            reason, event = '', ''
            if safe_error:
                reason, action = safe_error, 'safety_rejected'
            elif not budget_ok:
                reason, action = 'insufficient_movement_budget', 'budget_rejected'
                state['failure'] = {'reason': reason, 'decision_time_s': t, 'task': task,
                                    'energy_before_NEU': before, 'required_total_NEU': predicted, 'budget_NEU': budgets[b]}
            else:
                candidate_water = [end[k] - start[k] - current[k] for k in (0, 1)]
                assert math.hypot(*candidate_water) <= config['speed_m_per_s'][b] + 1e-10
                state['position_m'] = list(end)
                if task:
                    task_state = tasks[task]
                    if arrival:
                        task_state.update(status='arrived', arrival_s=t + 1)
                        event = 'arrived'
                    if action == 'service':
                        if task_state['arrival_s'] is None:
                            assert start == config['tasks_m'][task]
                            task_state['arrival_s'] = t
                        assert start == end == config['tasks_m'][task]
                        task_state['service_progress_s'] += 1
                        task_state['status'] = 'servicing'
                        if task_state['service_progress_s'] == config['service_s_per_task']:
                            task_state.update(status='completed', completion_s=t + 1,
                                              deadline_met=t + 1 <= task_state['deadline_s'])
                            event = 'completed'
                if b in cursor and task and (action == 'service' or end == waypoint):
                    cursor[b] += 1
            actual_end = state['position_m']
            actual = math.dist(start, actual_end)
            velocity = [actual_end[k] - start[k] for k in (0, 1)]
            water = [velocity[k] - current[k] for k in (0, 1)]
            candidate_water = [end[k] - start[k] - current[k] for k in (0, 1)]
            assert math.hypot(*water) <= config['speed_m_per_s'][b] + 1e-10
            assert all(math.isclose(actual_end[k], start[k]+water[k]+current[k], abs_tol=1e-10) for k in (0, 1))
            controls.append({'time_s': t, 'next_time_s': t + 1, 'boat_id': b,
                             'current_x_m_per_s': current[0], 'current_y_m_per_s': current[1],
                             'water_x_m_per_s': water[0], 'water_y_m_per_s': water[1],
                             'water_speed_m_per_s': math.hypot(*water),
                             'water_relative_distance_m': math.hypot(*water),
                             'candidate_water_x_m_per_s': candidate_water[0],
                             'candidate_water_y_m_per_s': candidate_water[1],
                             'candidate_water_speed_m_per_s': math.hypot(*candidate_water), 'water_limit_m_per_s': config['speed_m_per_s'][b],
                             'ground_x_m_per_s': velocity[0], 'ground_y_m_per_s': velocity[1],
                             'ground_speed_m_per_s': actual,
                             'plan_index_before': index_before, 'plan_index_after': cursor.get(b),
                             'waypoint_x_m': waypoint[0] if waypoint else None,
                             'waypoint_y_m': waypoint[1] if waypoint else None})
            state['distance_m'] += actual
            state['energy_NEU'] += coefficient * actual
            assert unsafe_reason(start, actual_end, config, blocked) is None
            assert before <= state['energy_NEU'] <= budgets[b] + 1e-10
            steps.append({'time_s': t, 'next_time_s': t + 1, 'boat_id': b, 'action': action, 'task_id': task,
                          'event_at_end': event, 'start_x_m': start[0], 'start_y_m': start[1],
                          'candidate_x_m': end[0], 'candidate_y_m': end[1],
                          'end_x_m': actual_end[0], 'end_y_m': actual_end[1],
                          'planned_distance_m': planned, 'actual_distance_m': actual,
                          'cumulative_distance_m': state['distance_m'], 'coefficient_NEU_per_m': coefficient,
                          'energy_before_NEU': before, 'predicted_energy_NEU': predicted,
                          'delta_energy_NEU': coefficient * actual, 'cumulative_energy_NEU': state['energy_NEU'],
                          'budget_NEU': budgets[b], 'budget_check_passed': budget_ok,
                          'remaining_NEU': max(0.0, budgets[b] - state['energy_NEU']),
                          'service_progress_s': tasks[task]['service_progress_s'] if task else '',
                          'target_boat': selected, 'target_x_m': tx, 'target_y_m': ty, 'rejection_reason': reason})
        previous = connected
    for task, item in tasks.items():
        if item['status'] != 'completed':
            failure = states[item['boat_id']]['failure']
            item.update(status='failed' if failure else 'unfinished',
                        failure_reason=failure['reason'] if failure else 'cutoff_before_service_completion')
    per_boat = {b: metrics([row for row in links if row['boat_id'] == b]) for b in ids}
    observed = sum(v['observed_boat_seconds'] for v in per_boat.values())
    connected_seconds = sum(v['connected_boat_seconds'] for v in per_boat.values())
    completed = [v['completion_s'] for v in tasks.values() if v['status'] == 'completed']
    summary = {'task_count': len(tasks), 'completed_task_count': len(completed),
               'completion_rate': len(completed) / len(tasks),
               'team_completion_time_s': max(completed) if len(completed) == len(tasks) else None,
               'cutoff_time_s': horizon, 'deadline_met_task_count': sum(v['deadline_met'] for v in tasks.values()),
               'boat_totals': {b: {**state, 'budget_NEU': budgets[b], 'remaining_NEU': max(0.0, budgets[b] - state['energy_NEU'])}
                               for b, state in states.items()},
               'communication': {'observed_boat_seconds': observed, 'connected_boat_seconds': connected_seconds,
                                 'retention_rate': connected_seconds / observed,
                                 'outage_event_sum': sum(v['outage_event_count'] for v in per_boat.values()),
                                 'longest_single_boat_outage_s': max(v['longest_outage_s'] for v in per_boat.values()),
                                 'per_boat': per_boat},
               'constraint_violations': {'assignment': 0, 'capacity': 0, 'actual_obstacle_collision': 0,
                                         'speed': 0, 'actual_budget_exceedance': 0,
                                         'deadline_missed_or_unfinished': sum(not v['deadline_met'] for v in tasks.values())},
               'rejected_safety_candidates': sum(row['action'] == 'safety_rejected' for row in steps),
               'rejected_budget_candidates': sum(row['action'] == 'budget_rejected' for row in steps),
               'boat_collision_modeled': False,
               'status': 'TASKS_COMPLETED' if len(completed) == len(tasks) else 'TASKS_INCOMPLETE'}
    return summary, tasks, steps, links, snapshots, controls

