"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L4/源码/p3_l4_map.py
"""
import math
import random
from usv_uuc_cooperation.planning.astar import astar
from usv_uuc_cooperation.env.obstacle import forbidden_nodes, unsafe_reason



def generated_tasks(config, blocked):
    fixed = config['task_generation']['fixed_first_points_m']
    starts = {tuple(p) for p in config['starts_m'].values()}
    xr, yr = config['task_generation']['candidate_x_range'], config['task_generation']['candidate_y_range']
    pool = [(x, y) for x in range(xr[0], xr[1] + 1) for y in range(yr[0], yr[1] + 1)
            if (x, y) not in blocked and (x, y) not in starts and [x, y] not in fixed]
    points = fixed + [list(p) for p in random.Random(config['random_seed']).sample(pool, 20 - len(fixed))]
    return {f'T{i:02d}': p for i, p in enumerate(points, 1)}


def audit(config):
    blocked = forbidden_nodes(config)
    assert config['map']['spacing_m'] == 1, 'Existing A* assumes 1m grid'
    assert config['tasks_m'] == generated_tasks(config, blocked), 'Frozen task coordinates changed'
    assert len(config['uuc_ids']) == 5 and len(config['tasks_m']) == 20
    assert len({tuple(p) for p in config['tasks_m'].values()}) == 20
    assert config['capacity_per_uuc'] * len(config['uuc_ids']) >= len(config['tasks_m'])
    points = {**config['starts_m'], **config['tasks_m']}
    assert all(unsafe_reason(p, p, config, blocked) is None for p in points.values()), 'Unsafe point'
    pairs = []
    for boat, start in config['starts_m'].items():
        for task, goal in config['tasks_m'].items():
            route = astar(start, goal, config['map'], blocked)
            assert route is not None, f'Unreachable {boat} to {task}'
            path = route['path']
            assert all(unsafe_reason(a, b, config, blocked) is None for a, b in zip(path, path[1:])), 'Unsafe edge'
            assert route['distance_m'] == sum(math.dist(a, b) for a, b in zip(path, path[1:]))
            pairs.append({'start_id': boat, 'task_id': task, 'distance_m': route['distance_m']})
    # Undirected grid: all points reachable from U1 implies all task-to-task pairs reachable.
    return {'status': 'PREFLIGHT_PASS', 'scope': 'map, endpoints and reachability only; not integrated simulation',
            'safe_points': len(points), 'forbidden_node_count': len(blocked),
            'start_task_paths_checked': len(pairs), 'pairs': pairs,
            'blocked_nodes': [list(p) for p in sorted(blocked)],
            'tasks_m': config['tasks_m'], 'boat_collision_modeled': False}

