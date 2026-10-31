"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P4/P4-L3/源码/p4_l3_dynamic.py
"""
import math
from usv_uuc_cooperation.env.acoustic_comm import communication_graph
from usv_uuc_cooperation.metrics.evaluator import metrics
from usv_uuc_cooperation.env.obstacle import unsafe_reason
import copy
import time
from usv_uuc_cooperation.planning.astar import astar
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.control.current_executor import waypoint_candidate
from usv_uuc_cooperation.planning.dynamic_replanner import snapshot, replan



def simulate(config, initial_sequences, events, budgets):
    config = copy.deepcopy(config)
    assert config['dt_s'] == 1 and config['cutoff_time_s'] == config['observation_duration_s']
    assert all(config['speed_m_per_s'][b] == 1 for b in config['uuc_ids'])
    ids, horizon = config['uuc_ids'], config['cutoff_time_s']
    assert all(e['type'] in ('TASK_ARRIVAL', 'UUV_FAILURE') and isinstance(e['time_s'], int)
               and 0 <= e['time_s'] < horizon for e in events)
    assert len({e['event_id'] for e in events}) == len(events)
    blocked = forbidden_nodes(config)
    states = {b: {'position_m': list(config['starts_m'][b]), 'energy_NEU': 0.0,
                  'distance_m': 0.0, 'budget_NEU': budgets[b], 'remaining_NEU': budgets[b],
                  'failure': None} for b in ids + ['USV']}
    accepted = {b: set() for b in ids}

    def new_task(position, released):
        assert unsafe_reason(position, position, config, blocked) is None
        return {'position_m': list(position), 'release_s': released, 'boat_id': None,
                'status': 'pending', 'arrival_s': None, 'service_progress_s': 0,
                'completion_s': None, 'deadline_s': config['deadline_s_per_task']}

    tasks = {k: new_task(v, 0) for k, v in config['tasks_m'].items()}
    assert sorted(k for seq in initial_sequences.values() for k in seq) == sorted(tasks)
    queues, plans, initial = replan(config, blocked, states, tasks, accepted, 0, 'INITIAL_FIXED', initial_sequences)
    replans, event_log, history, steps, links = [initial], [], [], [], []
    observed = set()
    previous, ages = {b: True for b in ids}, {b: 0 for b in ids}
    for t in range(horizon + 1):
        # The previous tick has already committed its movement/service ending at t.
        before_events = snapshot(states, tasks, accepted, queues)
        due = sorted((e for e in events if e['time_s'] == t),
                     key=lambda e: (0 if e['type'] == 'UUV_FAILURE' else 1, e['event_id']))
        if due:
            for e in due:
                assert e['event_id'] not in observed
                observed.add(e['event_id'])
                detail = {}
                if e['type'] == 'TASK_ARRIVAL':
                    assert e['task_id'] not in tasks
                    tasks[e['task_id']] = new_task(e['position_m'], t)
                else:
                    b = e['boat_id']
                    assert b in ids and not states[b]['failure']
                    states[b]['failure'] = {'reason': 'single_uuv_failure', 'time_s': t}
                    interrupted = []
                    for task, item in tasks.items():
                        if item['boat_id'] == b and item['status'] != 'completed':
                            interrupted.append({'task_id': task, 'previous_status': item['status'],
                                                'previous_service_progress_s': item['service_progress_s'],
                                                'reset_service_progress_s': 0})
                            item.update(boat_id=None, status='pending', arrival_s=None, service_progress_s=0)
                    detail = {'interrupted_tasks': interrupted, 'position_m': list(states[b]['position_m']),
                              'retained_energy_NEU': states[b]['energy_NEU']}
                event_log.append({**e, **detail, 'observed_at_s': t, 'processed_at_s': t})
            queues, plans, decision = replan(config, blocked, states, tasks, accepted, t,
                                             '+'.join(e['event_id'] for e in due), previous_queues=queues)
            replans.append(decision)
        positions = {b: list(states[b]['position_m']) for b in ids}
        usv = list(states['USV']['position_m'])
        graph, distances = communication_graph(usv, positions, config['communication']['radius_m'])
        connected = {b: graph.has_edge('USV', b) for b in ids}
        history.append({'time_s': t, 'before_events': before_events,
                        'after_events': snapshot(states, tasks, accepted, queues), 'connected': connected})
        if t == horizon:
            break
        for b in ids:
            ages[b] = 0 if connected[b] or previous[b] else ages[b] + 1
            links.append({'time_s': t, 'interval_end_s': t + 1, 'boat_id': b,
                          'position_m': positions[b], 'usv_position_m': usv,
                          'distance_m': distances[b], 'connected': connected[b], 'outage_age_s': ages[b]})
        lost = [b for b in ids if not connected[b]]
        target_boat = min(lost, key=lambda b: (-ages[b], distances[b], b)) if lost else ''
        target = positions[target_boat] if target_boat else usv
        candidate_usv = waypoint_candidate(usv, target, [0, 0], config['speed_m_per_s']['USV'], False)
        for b in ids + ['USV']:
            state = states[b]
            start, task = list(state['position_m']), ''
            if state['failure']:
                action, candidate = 'failed_hold', start
            elif b == 'USV':
                candidate = candidate_usv
                action = 'relay_move' if candidate != start else 'relay_hold'
            elif plans[b]:
                op = plans[b][0]
                action, task, candidate = op['action'], op['task_id'], op['end']
            else:
                action, candidate = 'finished_hold', start
            coefficient = config['energy']['usv_coefficient_NEU_per_m' if b == 'USV' else 'uuc_coefficient_NEU_per_m']
            predicted = state['energy_NEU'] + coefficient * math.dist(start, candidate)
            safe_error = unsafe_reason(start, candidate, config, blocked)
            budget_ok = predicted <= state['budget_NEU'] + 1e-10
            event_end, rejection = '', ''
            before_energy = state['energy_NEU']
            if safe_error:
                action, rejection, end = 'safety_rejected', safe_error, start
            elif not budget_ok:
                action, rejection, end = 'budget_rejected', 'insufficient_movement_budget', start
                state['failure'] = {'reason': rejection, 'time_s': t}
            else:
                end = list(candidate)
                if task:
                    item = tasks[task]
                    assert item['status'] != 'completed' and item['boat_id'] == b
                    if action == 'move' and end == item['position_m']:
                        item.update(status='arrived', arrival_s=t + 1)
                        event_end = 'arrived'
                    elif action == 'service':
                        assert start == end == item['position_m']
                        if item['arrival_s'] is None:
                            item['arrival_s'] = t
                        item['service_progress_s'] += 1
                        item['status'] = 'servicing'
                        if item['service_progress_s'] == config['service_s_per_task']:
                            item.update(status='completed', completion_s=t + 1)
                            event_end = 'completed'
                            assert queues[b].pop(0) == task
                    plans[b].pop(0)
            length = math.dist(start, end)
            assert length <= config['speed_m_per_s'][b] + 1e-10
            assert unsafe_reason(start, end, config, blocked) is None
            state.update(position_m=end, distance_m=state['distance_m'] + length,
                         energy_NEU=before_energy + coefficient * length)
            state['remaining_NEU'] = max(0.0, state['budget_NEU'] - state['energy_NEU'])
            assert state['energy_NEU'] <= state['budget_NEU'] + 1e-10
            steps.append({'time_s': t, 'next_time_s': t + 1, 'boat_id': b, 'action': action,
                          'task_id': task, 'start_m': start, 'candidate_m': list(candidate), 'end_m': end,
                          'actual_distance_m': length, 'energy_before_NEU': before_energy,
                          'predicted_total_NEU': predicted, 'delta_energy_NEU': coefficient * length,
                          'energy_NEU': state['energy_NEU'], 'remaining_NEU': state['remaining_NEU'],
                          'budget_NEU': state['budget_NEU'], 'budget_check_passed': budget_ok,
                          'rejection_reason': rejection, 'event_at_end': event_end,
                          'service_progress_s': tasks[task]['service_progress_s'] if task else None,
                          'relay_target': target_boat if b == 'USV' else ''})
        previous = connected
    completed = [v['completion_s'] for v in tasks.values() if v['status'] == 'completed']
    per_boat = {b: metrics([row for row in links if row['boat_id'] == b]) for b in ids}
    failure = replans[-1]['planning_failure']
    summary = {'status': 'SUCCESS' if len(completed) == len(tasks) else 'TASKS_INCOMPLETE',
               'completed_task_count': len(completed), 'known_task_count': len(tasks),
               'team_completion_time_s': max(completed) if len(completed) == len(tasks) else None,
               'cutoff_time_s': horizon, 'replanning_count': len(replans) - 1,
               'replanning_wall_time_s': sum(v['wall_time_s'] for v in replans[1:]),
               'initial_planning_wall_time_s': initial['wall_time_s'], 'planning_failure': failure,
               'total_energy_NEU': sum(s['energy_NEU'] for s in states.values()),
               'boat_totals': states, 'accepted_task_ids': {b: sorted(v) for b, v in accepted.items()},
               'communication': {'observed_boat_seconds': len(ids) * horizon,
                                 'connected_boat_seconds': sum(v['connected_boat_seconds'] for v in per_boat.values()),
                                 'retention_rate': sum(v['connected_boat_seconds'] for v in per_boat.values()) / (len(ids) * horizon),
                                 'per_boat': per_boat},
               'capacity_rule': 'distinct tasks ever accepted by each boat; completion/reassignment does not refund a slot',
               'current_mode': 'ZERO; inherited water-speed definition', 'optimality_claim': None,
               'fault_service_policy': 'unfinished service resets to zero; receiver restarts full continuous service',
               'boat_collision_modeled': False}
    return {'config': config, 'initial_sequences': initial_sequences, 'events': events,
            'budgets_NEU': budgets, 'summary': summary, 'tasks': tasks, 'steps': steps,
            'links': links, 'history': history, 'event_log': event_log, 'replans': replans}

