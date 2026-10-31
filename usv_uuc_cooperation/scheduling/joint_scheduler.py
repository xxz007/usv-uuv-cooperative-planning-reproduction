"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L4/源码/p3_l4_joint.py
"""
from usv_uuc_cooperation.planning.astar import astar



def schedule(config, blocked):
    sequences = {b: [] for b in config['uuc_ids']}
    tails = {b: config['starts_m'][b] for b in sequences}
    remaining = set(config['tasks_m'])
    decisions = []
    for round_index in range(config['capacity_per_uuc']):
        for boat in config['uuc_ids']:
            if not remaining:
                break
            candidates = []
            for task in sorted(remaining):
                route = astar(tails[boat], config['tasks_m'][task], config['map'], blocked)
                if route is not None:
                    candidates.append((route['distance_m'], task))
            if not candidates:
                return None, decisions, {'reason': 'heuristic_no_reachable_candidate', 'boat_id': boat,
                                         'unassigned_tasks': sorted(remaining), 'global_infeasibility_proved': False}
            distance, selected = min(candidates)
            decisions.append({'round': round_index + 1, 'boat_id': boat, 'tail_m': list(tails[boat]),
                              'selected_task': selected, 'incremental_astar_distance_m': distance,
                              'candidates': [{'task_id': task, 'distance_m': d} for d, task in sorted(candidates)],
                              'tie_break': 'distance ascending then task ID'})
            sequences[boat].append(selected)
            tails[boat] = config['tasks_m'][selected]
            remaining.remove(selected)
    if remaining:
        return None, decisions, {'reason': 'capacity_exhausted', 'unassigned_tasks': sorted(remaining),
                                 'global_infeasibility_proved': True}
    return sequences, decisions, None

