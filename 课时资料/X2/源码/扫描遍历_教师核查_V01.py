"""Scan ordering/service protection and focused shared-executor regressions."""
import argparse
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from usv_uuc_cooperation.config import PROJECT_ROOT
from usv_uuc_cooperation.experiments.x2_comparison import load_config
from usv_uuc_cooperation.control.x2_executor import simulate
from usv_uuc_cooperation.planning.scan_routes import scan_order, scan_rebuild, ScanAStarSession
from usv_uuc_cooperation.planning.dstar_original import OriginalDStarSession
from usv_uuc_cooperation.metrics.scan_check import check_scan


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    def save(name, data):
        (output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    report = {'status': 'IN_PROGRESS', 'teacher_full_120s_runs': 0, 'learner_runs': 0,
              'solver_runs': 0, 'plot_runs': 0}
    try:
        c = load_config()
        tasks = {t: {'position_m': p, 'status': 'pending', 'release_s': 0, 'boat_id': 'U1',
                     'service_progress_s': 0, 'arrival_s': None, 'completion_s': None}
                 for t, p in {'A': [2, 2], 'B': [6, 2], 'C': [2, 5], 'D': [6, 5]}.items()}
        assert scan_order(['A', 'D', 'B', 'C'], tasks, c, 0) == ['A', 'B', 'D', 'C']
        assert scan_order(['A', 'D', 'B', 'C'], tasks, c, 0, 'D') == ['D', 'A', 'B', 'C']
        twin = {t: {**tasks['D'], 'position_m': [6, 5]} for t in ['Z', 'AA']}
        assert scan_order(['Z', 'AA'], twin, c, 0) == ['AA', 'Z']
        future = copy.deepcopy(tasks)
        future['D']['release_s'] = 11
        try:
            scan_order(['D'], future, c, 0)
            raise AssertionError('Unreleased task was accepted')
        except ValueError:
            pass
        small = {'map': {**c['map'], 'x_range': [0, 8], 'y_range': [0, 6], 'circles': [], 'safety_buffer_m': 0},
            'uuc_ids': ['U1'], 'starts_m': {'U1': [6, 5]}, 'tasks_m': {t: v['position_m'] for t, v in tasks.items()},
            'energy': {'uuc_coefficient_NEU_per_m': 2}, 'capacity_per_uuc': 6, 'service_s_per_task': 2}
        states = {'U1': {'position_m': [6, 5], 'energy_NEU': 3, 'budget_NEU': 200, 'failure': None}}
        for progress, status in [(0, 'arrived'), (1, 'servicing')]:
            local = copy.deepcopy(tasks)
            local['D'].update(status=status, arrival_s=1, service_progress_s=progress)
            planner = ScanAStarSession(small)
            queues, plans, decision = scan_rebuild(small, set(), states, local, {'U1': set(tasks)},
                {'U1': ['D', 'C', 'A', 'B']}, 1, 'TEACHER_PROTECTION', False, planner)
            assert queues['U1'] == ['D', 'A', 'B', 'C']
            assert local['D']['service_progress_s'] == progress and local['D']['arrival_s'] == 1
            assert plans['U1'][0]['action'] == 'service' and plans['U1'][0]['task_id'] == 'D'
        report['snake_rows_ties_release_and_arrived_servicing_head'] = 'PASS'
        save('局部排序与服务保护.json', {'status': 'PASS', 'example': ['A', 'B', 'D', 'C'],
            'protected_D': ['D', 'A', 'B', 'C'], 'arrived_zero_progress_and_servicing_checked': True})
        # Default callback path must preserve the saved V12 physical behavior.
        old_path = PROJECT_ROOT / '课时资料/X2/验收/原始Dstar_教师核查_V02/教师8秒_结果.json'
        old = json.loads(old_path.read_text())
        baseline = simulate(old['config'], old['config']['x2']['initial_assignment'], 'DYNAMIC_REPLAN', OriginalDStarSession(old['config']))
        for key in ['steps', 'snapshots', 'summary', 'tasks', 'event_log', 'initial_queues']:
            assert baseline[key] == old[key], key
        report['default_executor_8s_vs_saved_V12_physical_outputs'] = 'PASS'
        report['baseline_regression_short_runs'] = 1
        save('基线8秒回归核查.json', {'status': 'PASS', 'source': str(old_path),
            'equal_fields': ['steps', 'snapshots', 'summary', 'tasks', 'event_log', 'initial_queues'],
            'scope': 'only default rebuild callback addition; no full baseline simulation'})
        short = copy.deepcopy(c)
        short['cutoff_time_s'] = short['observation_duration_s'] = 26
        planner = ScanAStarSession(short)
        result = simulate(short, short['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner, rebuild_routes=scan_rebuild)
        result['summary']['actual_total_distance_m'] = sum(b['distance_m'] for b in result['summary']['boat_totals'].values())
        result.update(algorithm='GridScanAstar', connector_algorithm='Astar')
        save('教师26秒_结果.json', result)
        save('教师26秒_搜索记录.json', planner.rows)
        save('教师26秒_图版本.json', planner.graphs)
        checks = check_scan(result, planner.rows, planner.graphs)
        assert len(result['snapshots']) == 27 and len(result['steps']) == 156
        assert [d['time_s'] for d in result['replans']] == [0, 6, 11, 18, 24]
        save('独立核查.json', checks)
        report['scan_26s_dynamic_wiring'] = {'status': 'PASS', 'short_runs': 1, 'snapshots': 27,
            'actual_steps': 156, 'queries': len(planner.rows), 'decision_times': [0, 6, 11, 18, 24],
            'scope': 'initial ordering, map updates, arrivals, fault handoff, service protection and complete remaining budget; not full execution acceptance'}
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
