"""Bounded PSO mechanism, representation, failure handling and wiring checks."""
import argparse
import ast
import copy
import json
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from usv_uuc_cooperation.config import PROJECT_ROOT, read_json
from usv_uuc_cooperation.experiments.x2_comparison import load_config
from usv_uuc_cooperation.experiments.x2_pso import local_statistics
from usv_uuc_cooperation.control.x2_executor import simulate
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.planning.pso_search import PSOSession, remember_feasible, decode
from usv_uuc_cooperation.planning.pso_routes import pso_rebuild
from usv_uuc_cooperation.planning.dstar_original import OriginalDStarSession
from usv_uuc_cooperation.planning.x2_search import ObservedGraph, astar_search
from usv_uuc_cooperation.metrics.pso_check import check_pso, check_queries, independent_decode


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    def save(name, data):
        (output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    report = {'status': 'IN_PROGRESS', 'teacher_full_120s_runs': 0, 'learner_runs': 0, 'solver_runs': 0}
    try:
        c = load_config()
        policy = read_json(PROJECT_ROOT / 'usv_uuc_cooperation/configs/X2-PSO-V01.json')
        previous = {'feasible': True, 'distance_m': 16., 'vector': [1., 2.]}
        assert remember_feasible(previous, [2., 3.], {'feasible': True, 'distance_m': 18.}) is previous
        assert remember_feasible(previous, [2., 3.], {'feasible': False, 'distance_m': 12.}) is previous
        assert remember_feasible(previous, [2., 3.], {'feasible': True, 'distance_m': 15.})['distance_m'] == 15.
        report['pbest_16_to_18_invalid_and_strict_improvement'] = 'PASS'
        graph = ObservedGraph({'map': c['map']}, forbidden_nodes(c), 0)
        graph.arc_costs = {(a, b): w for a, edges in graph.edges.items() for b, w in edges}
        # Offline reference paths test representability; never seed the optimizer.
        representations = []
        for boat, queue in c['x2']['initial_queues'].items():
            start = c['starts_m'][boat]
            for task in queue:
                goal = c['tasks_m'][task]
                path = astar_search(tuple(start), tuple(goal), graph)['path']
                anchors, i = [], 0
                while i < len(path) - 1:
                    end = max(j for j in range(i + 1, len(path)) if independent_decode([], path[i], path[j]) == path[i:j + 1])
                    anchors.append(path[end]); i = end
                middle = anchors[:-1]
                assert len(middle) <= policy['waypoints'], (boat, task, len(middle))
                vector = [v for p in middle + [goal] * (policy['waypoints'] - len(middle)) for v in p]
                decoded = decode(vector, tuple(start), tuple(goal), graph)
                assert decoded['feasible'] and decoded['path'] == path
                representations.append({'boat': boat, 'task': task, 'intermediate_anchors_needed': len(middle), 'distance_m': decoded['distance_m']})
                start = goal
        save('仅离线表示范围核查.json', {'status': 'PASS', 'scope': '20 initial retained legs, not every possible route', 'legs': representations,
            'reference_paths_used_in_production': False})
        report['twenty_initial_exact_paths_representable_with_eight_anchors'] = 'PASS'
        local, traces = local_statistics(c, policy)
        save('固定五种子局部统计.json', local)
        save('固定五种子局部追踪.json', traces)
        report['five_seed_local'] = local
        planner = PSOSession(c, policy)
        physical = {'map': c['map']}
        start, goal = c['starts_m']['U1'], c['tasks_m']['T03']
        r1 = planner(start, goal, physical, forbidden_nodes(c))
        r2 = planner(start, goal, physical, forbidden_nodes(c))
        assert r1['path'] == r2['path'] and planner.rows[-1]['evaluations'] == 0
        repeat = PSOSession(c, policy)
        repeat(start, goal, physical, forbidden_nodes(c))
        for key in ('path', 'distance_m', 'best_vector', 'history', 'update_samples', 'evaluations'):
            assert planner.rows[0][key] == repeat.rows[0][key]
        planner.observe(physical, forbidden_nodes(c) | {(12, 3)}, 1)
        planner.context(6, 'MAP_UPDATE')
        planner(start, goal, physical, forbidden_nodes(c) | {(12, 3)})
        assert not planner.rows[-1]['cache_hit'] and planner.rows[-1]['derived_seed'] != planner.rows[0]['derived_seed']
        check_queries(c, planner.rows, planner.graphs, policy)
        try:
            planner(start, goal, c, forbidden_nodes(c))
            raise AssertionError('Future tables accepted')
        except ValueError:
            pass
        report['reproducible_seed_exact_cache_graph_identity_and_future_rejection'] = 'PASS'
        # Isolate arrived-head protection, held-tail ownership and prefix budget.
        small = {'map': {**c['map'], 'x_range': [0, 8], 'y_range': [0, 6], 'circles': [], 'safety_buffer_m': 0},
            'uuc_ids': ['U1'], 'starts_m': {'U1': [2, 2]}, 'tasks_m': {'A': [2, 2], 'B': [6, 2], 'N': [3, 2]},
            'energy': {'uuc_coefficient_NEU_per_m': 2}, 'capacity_per_uuc': 6, 'service_s_per_task': 2}
        state = {'U1': {'position_m': [2, 2], 'energy_NEU': 3, 'budget_NEU': 200, 'failure': None}}
        class HeadOnly:
            def __call__(self, a, b, config, blocked):
                return {'path': [a], 'distance_m': 0., 'search_query_id': 1} if a == b else None
        holds = []
        for progress, status in [(0, 'arrived'), (1, 'servicing')]:
            tasks = {t: {'position_m': p, 'status': 'pending', 'release_s': 0, 'boat_id': 'U1' if t != 'N' else None,
                'service_progress_s': 0, 'arrival_s': None} for t, p in small['tasks_m'].items()}
            tasks['A'].update(status=status, service_progress_s=progress, arrival_s=1)
            queues, plans, d = pso_rebuild(small, set(), state, tasks, {'U1': {'A', 'B'}}, {'U1': ['A', 'B']}, 1, 'TEST', True, HeadOnly())
            assert queues['U1'] == ['A', 'B'] and all(p['action'] == 'service' and p['task_id'] == 'A' for p in plans['U1'])
            assert len(plans['U1']) == 2 - progress and tasks['B']['boat_id'] == 'U1'
            assert tasks['N']['boat_id'] is None and state['U1']['failure'] is None
            assert d['planning_holds']['U1']['held_task_ids'] == ['B'] and d['candidate_checks'] == []
            queues['U1'].pop(0)
            assert d['output_queues']['U1'] == ['A', 'B']
            holds.append(d)
        tasks['A'].update(status='completed', service_progress_s=2)
        low = copy.deepcopy(state); low['U1']['budget_NEU'] = 4
        queues, plans, d = pso_rebuild(small, set(), low, tasks, {'U1': {'A', 'B'}}, {'U1': ['B']}, 6, 'MAP_UPDATE', False, PSOSession(small, policy))
        assert d['planning_holds']['U1']['reason'] == 'PREFIX_ENERGY_BUDGET' and not plans['U1']
        queues, plans, recovered = pso_rebuild(small, set(), state, tasks, {'U1': {'A', 'B'}}, {'U1': ['B']}, 11, 'TASK_ARRIVAL', False, PSOSession(small, policy))
        assert not recovered['planning_holds'] and plans['U1'] and tasks['B']['boat_id'] == 'U1'
        save('失败保留与下次事件恢复核查.json', {'status': 'PASS', 'holds': holds, 'budget_hold': d, 'ordinary_event_recovery': recovered})
        report['held_tail_arrived_service_budget_no_new_admission_and_event_retry'] = 'PASS'
        old_path = PROJECT_ROOT / '课时资料/X2/验收/原始Dstar_教师核查_V02/教师8秒_结果.json'
        old = read_json(old_path)
        baseline = simulate(old['config'], old['config']['x2']['initial_assignment'], 'DYNAMIC_REPLAN', OriginalDStarSession(old['config']))
        for key in ('steps', 'snapshots', 'summary', 'tasks', 'event_log', 'initial_queues'):
            assert baseline[key] == old[key], key
        report['default_rebuild_eight_second_physical_regression'] = 'PASS'
        source = PROJECT_ROOT / 'usv_uuc_cooperation/planning/pso_search.py'
        tree = ast.parse(source.read_text())
        assert not any(isinstance(n, ast.Name) and n.id in {'astar', 'astar_search', 'dijkstra_search', 'plan', 'reference_distances'} for n in ast.walk(tree))
        short = copy.deepcopy(c)
        short['cutoff_time_s'] = short['observation_duration_s'] = 26
        planner = PSOSession(short, policy)
        with patch('usv_uuc_cooperation.planning.x2_search.astar_search', side_effect=AssertionError('Hidden Astar')), patch('usv_uuc_cooperation.planning.x2_search.dijkstra_search', side_effect=AssertionError('Hidden Dijkstra')), patch('usv_uuc_cooperation.planning.continuous_routes.astar', side_effect=AssertionError('Hidden connector Astar')):
            result = simulate(short, short['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner, rebuild_routes=pso_rebuild)
        result['summary']['actual_total_distance_m'] = sum(b['distance_m'] for b in result['summary']['boat_totals'].values())
        result.update(algorithm='PSO', adjacency=8)
        save('教师26秒_结果.json', result)
        save('教师26秒_搜索记录.json', planner.rows)
        save('教师26秒_图版本.json', planner.graphs)
        checks = check_pso(read_json(output / '教师26秒_结果.json'), read_json(output / '教师26秒_搜索记录.json'), read_json(output / '教师26秒_图版本.json'), policy)
        assert [d['time_s'] for d in result['replans']] == [0, 6, 11, 18, 24]
        save('独立保存产物核查.json', checks)
        report.update(status='PASS', short_wiring={'seconds': 26, 'snapshots': len(result['snapshots']), 'steps': len(result['steps']),
            'queries': len(planner.rows), 'holds': checks['boat_hold_decisions'], 'failed_queries': checks['search']['failed_queries']},
            no_hidden_graph_search='PASS', full_fleet_acceptance='PENDING_LEARNER_RUN')
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
