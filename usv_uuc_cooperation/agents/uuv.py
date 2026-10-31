"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L4/源码/p3_l4_sim.py
"""
from usv_uuc_cooperation.planning.astar import astar



def task_plan(boat, sequence, config, blocked):
    point = list(config['starts_m'][boat])
    plan, legs = [], []
    for task in sequence:
        route = astar(point, config['tasks_m'][task], config['map'], blocked)
        if route is None:
            return None, {'reason': 'unreachable', 'boat': boat, 'task': task}
        legs.append({'task': task, **route})
        for a, b in zip(route['path'], route['path'][1:]):
            plan.append({'action': 'move', 'start': a, 'end': b, 'task': task,
                         'arrived': b == config['tasks_m'][task]})
        point = list(config['tasks_m'][task])
        for _ in range(config['service_s_per_task']):
            plan.append({'action': 'service', 'start': point, 'end': point, 'task': task, 'arrived': False})
    return plan, legs

