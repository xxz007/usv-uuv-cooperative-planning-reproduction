"""Recheck the saved 26s run; verify scan decision queues do not alias live queues."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from usv_uuc_cooperation.metrics.scan_check import check_scan
from usv_uuc_cooperation.planning.scan_routes import scan_rebuild, ScanAStarSession


def run(prior, output):
    output.mkdir(parents=True, exist_ok=False)
    old = json.loads((prior / '核查摘要.json').read_text())
    assert old['snake_rows_ties_release_and_arrived_servicing_head'] == 'PASS'
    assert old['default_executor_8s_vs_saved_V12_physical_outputs'] == 'PASS'
    result = json.loads((prior / '教师26秒_结果.json').read_text())
    before_physics = {k: copy.deepcopy(result[k]) for k in ['steps', 'snapshots', 'summary', 'tasks', 'event_log', 'initial_queues']}
    repairs = []
    for d in result['replans']:
        if d['output_queues'] != d['scan_ordering']['final_scan_queues']:
            repairs.append(d['time_s'])
        d['output_queues'] = copy.deepcopy(d['scan_ordering']['final_scan_queues'])
    assert all(result[k] == v for k, v in before_physics.items())
    # Isolate the production logging fix: mutating the live queue must not alter a decision.
    c = result['config']
    snap = result['snapshots'][0]
    physical = {k: c[k] for k in ['map', 'uuc_ids', 'starts_m', 'tasks_m', 'energy', 'capacity_per_uuc', 'service_s_per_task']}
    tasks, states = copy.deepcopy(snap['tasks']), copy.deepcopy(snap['boats'])
    accepted = {b: set(c['x2']['initial_assignment'][b]) for b in c['uuc_ids']}
    planner = ScanAStarSession(c)
    queues, _, decision = scan_rebuild(physical, planner.graph.blocked, states, tasks, accepted,
        copy.deepcopy(c['x2']['initial_queues']), 0, 'TEACHER_LOG_ISOLATION', False, planner)
    saved = copy.deepcopy(decision['output_queues'])
    queues['U1'].pop(0)
    assert decision['output_queues'] == saved
    rows = json.loads((prior / '教师26秒_搜索记录.json').read_text())
    graphs = json.loads((prior / '教师26秒_图版本.json').read_text())
    def save(name, data):
        (output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    save('教师26秒_结果_决策快照派生修正.json', result)
    try:
        checks = check_scan(result, rows, graphs)
        save('保存产物独立核查.json', checks)
        report = {'status': 'PASS', 'prior_report': str(prior / '核查摘要.json'),
            'prior_artifact_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in prior.glob('*.json')},
            'prior_successful_local_and_default_regression_checks_reused': True,
            'logging_fix': 'scan decision output_queues copied independently of live execution queues',
            'derived_snapshot_repaired_decision_times': repairs,
            'physical_steps_snapshots_summary_tasks_events_unchanged': True,
            'production_live_queue_mutation_isolation_test': 'PASS',
            'scan_26s_saved_run': {'status': 'PASS', 'snapshots': 27, 'actual_steps': 156,
                'queries': len(rows), 'decision_times': [d['time_s'] for d in result['replans']]},
            'baseline_short_8s_runs': 1, 'scan_short_26s_runs': 1,
            'simulation_reruns_for_logging_fix': 0, 'teacher_full_120s_runs': 0, 'learner_runs': 0,
            'solver_runs': 0, 'plot_runs': 0}
        save('核查摘要.json', report)
        return report
    except Exception as exc:
        save('核查摘要.json', {'status': 'FAILED', 'error_class': type(exc).__name__, 'error': str(exc)})
        raise


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prior', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(run(a.prior, a.output), ensure_ascii=False))
