"""Targeted original-D* checks; no full 120s simulation or learner acceptance."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from usv_uuc_cooperation.planning.dstar_original import OriginalDStar, OriginalDStarSession
from usv_uuc_cooperation.experiments.x2_original_dstar import local_segment
from usv_uuc_cooperation.experiments.x2_comparison import load_config
from usv_uuc_cooperation.metrics.original_dstar_check import check_original_dstar
from usv_uuc_cooperation.metrics.x2_check import reference_distances
from usv_uuc_cooperation.control.x2_executor import simulate


def graph(nodes, undirected, version, ranges):
    edges = {a: [] for a in nodes}
    for (a, b), w in sorted(undirected.items()):
        edges[a].append((b, w))
        edges[b].append((a, w))
    return SimpleNamespace(config={'map': {'x_range': ranges[0], 'y_range': ranges[1]}},
        edges=edges, version=version, sha256=hashlib.sha256(repr((edges, version)).encode()).hexdigest())


def check_cost(solver, start, g):
    out = solver.solve(start, g)
    expected = reference_distances(g, solver.goal).get(start)
    assert (out['path'] is not None) == (expected is not None)
    if expected is not None:
        assert math.isclose(out['distance_m'], expected, rel_tol=0, abs_tol=1e-9)
        path = list(map(tuple, out['path']))
        assert path[0] == start and path[-1] == solver.goal and len(set(path)) == len(path)
        assert all(b in dict(g.edges[a]) for a, b in zip(path, path[1:]))
    return out


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    def save(name, data):
        (output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    report = {'status': 'IN_PROGRESS', 'teacher_full_120s_runs': 0, 'learner_runs': 0,
              'solver_runs': 0, 'visual_renders': 0}
    try:
        nodes = [(0, 0), (1, 0), (1, 1), (2, 0)]
        s, a, b, goal = nodes
        edges = {(s, a): 2., (s, b): 3., (b, goal): 4., (a, goal): 2.}
        g = graph(nodes, edges, 0, ([0, 2], [0, 1]))
        solver = OriginalDStar(g, s, goal)
        toy = [check_cost(solver, s, g)]
        closed = dict(edges)
        del closed[(a, goal)]
        toy.append(check_cost(solver, s, graph(nodes, closed, 1, ([0, 2], [0, 1]))))
        assert [r['distance_m'] for r in toy] == [4, 7]
        assert solver.h[a] == 9 and solver.h[s] == 7
        assert any(u['reason'] == 'BACKPOINTER_DEPENDENCY' and u['node'] == list(s) for u in toy[-1]['updates'])
        toy.append(check_cost(solver, s, graph(nodes, edges, 2, ([0, 2], [0, 1]))))
        assert toy[-1]['distance_m'] == 4
        save('四节点_封路恢复传播.json', toy)
        report['closed_node_and_upstream_dependency'] = 'PASS: 4 -> 7 -> 4; A=9 after closure'
        line_nodes = [(i, 0) for i in range(5)]
        line = {(line_nodes[i], line_nodes[i + 1]): 1. for i in range(4)}
        g = graph(line_nodes, line, 0, ([0, 4], [0, 0]))
        solver = OriginalDStar(g, line_nodes[0], line_nodes[-1])
        bridge = [check_cost(solver, line_nodes[0], g)]
        cut = dict(line)
        del cut[(line_nodes[2], line_nodes[3])]
        bridge.append(check_cost(solver, line_nodes[0], graph(line_nodes, cut, 1, ([0, 4], [0, 0]))))
        assert bridge[-1]['status'] == 'UNREACHABLE'
        bridge.append(check_cost(solver, line_nodes[0], graph(line_nodes, line, 2, ([0, 4], [0, 0]))))
        assert bridge[-1]['distance_m'] == 4
        save('桥接_不可达恢复.json', bridge)
        report['unreachable_then_recovery_same_state'] = 'PASS'
        rng = random.Random(20261026)
        grid = [(x, y) for x in range(8) for y in range(8)]
        edges = {(p, q): rng.choice([1., math.sqrt(2), 2., 5.]) for p in grid for q in grid
                 if p < q and abs(p[0] - q[0]) + abs(p[1] - q[1]) == 1}
        all_edges = sorted(edges)
        g = graph(grid, edges, 0, ([0, 7], [0, 7]))
        solver = OriginalDStar(g, (0, 0), (7, 7))
        probes = []
        for version in range(25):
            if version:
                for edge in rng.sample(all_edges, 5):
                    if rng.random() < .3:
                        edges.pop(edge, None)
                    else:
                        edges[edge] = rng.choice([1., math.sqrt(2), 2., 5.])
            g = graph(grid, edges, version, ([0, 7], [0, 7]))
            for start in [(0, 0), (3, 2), (6, 5)]:
                row = check_cost(solver, start, g)
                probes.append({'version': version, 'start': start, 'distance_m': row['distance_m'],
                               'logical_pops': row['expanded_nodes'], 'status': 'PASS'})
        save('权重增减删除恢复_移动起点.json', probes)
        report['weighted_symmetric_changes_and_moving_start'] = {'status': 'PASS', 'queries': len(probes), 'seed': 20261026}
        c = load_config()
        local = local_segment(c)
        local_check = check_original_dstar(c, local.rows, local.graphs)
        save('X2局部查询.json', local.rows)
        save('X2局部图版本.json', local.graphs)
        save('X2局部独立核查.json', local_check)
        report['same_L2_local_input_order_and_costs'] = {'status': 'PASS',
            'costs_m': [r['distance_m'] for r in local.rows],
            'state_id_and_modes': [[r['state_id'], r['mode']] for r in local.rows]}
        old_state_id = local.rows[1]['state_id']
        local.context(18, 'TEACHER_CACHE_RETURN')
        local.route_context('LOCAL_T08_T19', 'T19', 'FIXED_TARGET_SEGMENT')
        local([11, 2], c['tasks_m']['T19'], {'map': c['map']}, local.graph.blocked)
        assert local.rows[-1]['state_id'] == old_state_id and local.rows[-1]['cache_reactivated']
        check_original_dstar(c, local.rows, local.graphs)
        report['new_goal_isolation_and_old_cache_reactivation'] = 'PASS'
        rejected = 0
        for invoke in [lambda: local.observe(c, local.graph.blocked, 2),
                       lambda: local([11, 2], c['tasks_m']['T19'], c, local.graph.blocked)]:
            try:
                invoke()
            except ValueError:
                rejected += 1
        assert rejected == 2
        report['future_table_rejection'] = 'PASS'
        short = copy.deepcopy(c)
        short['cutoff_time_s'] = 8
        planner = OriginalDStarSession(short)
        result = simulate(short, short['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner)
        check_original_dstar(short, planner.rows, planner.graphs)
        assert len(result['snapshots']) == 9 and len(result['steps']) == 48
        assert result['initial_queues'] == c['x2']['initial_queues']
        assert any(r['time_s'] == 6 and r['mode'] == 'SAME_GOAL_REPAIR' for r in planner.rows)
        save('教师8秒_结果.json', result)
        save('教师8秒_搜索记录.json', planner.rows)
        save('教师8秒_图版本.json', planner.graphs)
        report['teacher_8s_connectivity'] = {'status': 'PASS', 'runs': 1, 'snapshots': 9,
            'actual_steps': 48, 'queries': len(planner.rows), 'scope': 'initial queues + t6 update; not a full fleet acceptance'}
        report['status'] = 'PASS'
        save('核查摘要.json', report)
        return report
    except Exception as exc:
        report.update(status='FAILED', error_class=type(exc).__name__, error=str(exc))
        save('核查摘要.json', report)
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.output), ensure_ascii=False))
