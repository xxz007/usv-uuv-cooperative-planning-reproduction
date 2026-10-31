"""Saved scan order, unchanged dispatch policy and complete execution checks."""
import math

from usv_uuc_cooperation.metrics.x2_check import check_execution, check_searches
from usv_uuc_cooperation.metrics.integrated_check import safe_segment


def independent_order(task_ids, tasks, config, protected=None):
    # Reconstruct rows separately from the production key function.
    by_row = {}
    m = config['map']
    for task in task_ids:
        if task == protected:
            continue
        x, y = tasks[task]['position_m']
        row = int(round((y - m['y_range'][0]) / m['spacing_m']))
        by_row.setdefault(row, []).append((x, task))
    output = [protected] if protected is not None else []
    for row in sorted(by_row):
        xs = sorted({x for x, _ in by_row[row]}, reverse=bool(row % 2))
        for x in xs:
            output.extend(sorted(task for xx, task in by_row[row] if xx == x))
    return output


def check_scan(result, searches, graph_records):
    c = result['config']
    initial_tasks = result['snapshots'][0]['tasks']
    expected_initial = {b: independent_order(c['x2']['initial_assignment'][b], initial_tasks, c)
                        for b in c['uuc_ids']}
    searches_check = check_searches(c, searches, graph_records)
    execution = check_execution(result, searches, expected_initial_queues=expected_initial)
    execution['declared_scan_initial_queues'] = execution.pop('initial_frozen_queues')
    by_id = {row['query_id']: row for row in searches}
    ordered_checks, dispatch_count = [], 0
    for d in result['replans']:
        snap = result['snapshots'][d['time_s']]
        tasks, states = snap['tasks'], snap['boats']
        scan = d['scan_ordering']
        assert d['observed_task_ids'] == sorted(tasks)
        for b in c['uuc_ids']:
            queue = d['output_queues'][b]
            owned = [t for t, item in tasks.items() if item['boat_id'] == b and item['status'] != 'completed']
            if states[b]['failure']:
                assert queue == []
                continue
            assert len(queue) == len(set(queue)) and set(queue) == set(owned)
            assert all(tasks[t]['release_s'] <= d['time_s'] for t in queue)
            heads = [t for t in owned if tasks[t]['status'] in ('arrived', 'servicing') or tasks[t]['service_progress_s'] > 0]
            assert len(heads) <= 1
            protected = heads[0] if heads else None
            assert scan['protected_heads'][b] == protected
            assert queue == independent_order(owned, tasks, c, protected)
        reference = scan['shared_dispatch_reference'] or d
        # Check provisional dispatch paths too, not just final executable legs.
        blocked = set(map(tuple, snap['observed_blocked_cells']))
        for leg in reference['legs']:
            row = by_id[leg['search_query_id']]
            assert row['algorithm'] == 'Astar' and row['time_s'] == d['time_s']
            assert row['path'] == leg['grid_path'] and row['goal'] == leg['path'][-1]
            assert all(safe_segment(a, b, c, blocked) for a, b in zip(leg['path'], leg['path'][1:]))
            assert math.isclose(math.fsum(math.dist(a, b) for a, b in zip(leg['path'], leg['path'][1:])), leg['distance_m'], abs_tol=1e-9)
        pending = {t for t, owner in scan['pre_dispatch_owners'].items()
                   if tasks[t]['status'] != 'completed' and (owner is None or states[owner]['failure'])}
        prior = d.get('input_state', {}).get('accepted_task_ids', c['x2']['initial_assignment'])
        accepted = {b: set(prior[b]) for b in c['uuc_ids']}
        chosen = [a for a in reference['assignments'] if a['old_owner'] != a['boat_id']]
        candidates, selected = reference['candidate_checks'], 0
        healthy = [b for b in c['uuc_ids'] if not states[b]['failure']]
        while candidates:
            width = len(healthy) * len(pending)
            assert width > 0
            batch, candidates = candidates[:width], candidates[width:]
            assert len(batch) == width
            assert {(x['boat_id'], x['task_id']) for x in batch} == {(b, t) for b in healthy for t in pending}
            eligible = []
            for candidate in batch:
                b, task = candidate['boat_id'], candidate['task_id']
                cap = len(accepted[b] | {task}) <= c['capacity_per_uuc']
                distance = candidate['route_distance_m']
                predicted = candidate['projected_before_NEU'] + c['energy']['uuc_coefficient_NEU_per_m'] * distance if distance is not None else None
                budget = predicted is not None and predicted <= states[b]['budget_NEU'] + 1e-9
                assert candidate['capacity_ok'] == cap and candidate['budget_ok'] == budget
                assert candidate['predicted_total_NEU'] == predicted
                if cap and budget:
                    eligible.append((distance, b, task))
            if not eligible:
                assert not candidates
                break
            _, boat, task = min(eligible)
            assert chosen[selected]['boat_id'] == boat and chosen[selected]['task_id'] == task
            accepted[boat].add(task)
            pending.remove(task)
            selected += 1
        assert selected == len(chosen)
        assert pending == set(d['unassigned_tasks'])
        dispatch_count += selected
        assert scan['final_scan_queues'] == d['output_queues']
        ordered_checks.append({'time_s': d['time_s'], 'status': 'PASS', 'queues': d['output_queues'],
                               'protected_heads': scan['protected_heads'], 'post_dispatch_resort': scan['post_dispatch_resort']})
    # The frozen assignment is identical; the explicitly changed factor is queue order.
    assert all(set(expected_initial[b]) == set(c['x2']['initial_assignment'][b]) for b in c['uuc_ids'])
    return {'status': 'PASS', 'search': searches_check, 'execution': execution,
        'initial_assignment_same_scan_order_independently_reconstructed': 'PASS',
        'released_only_and_arrived_servicing_head_protected': 'PASS',
        'shared_tail_greedy_capacity_budget_dispatch_replayed': 'PASS',
        'final_resorted_full_route_budget': 'PASS', 'dynamic_assignments_checked': dispatch_count,
        'ordering_checks': ordered_checks, 'changed_factor': 'task ordering strategy, Astar connector unchanged'}
