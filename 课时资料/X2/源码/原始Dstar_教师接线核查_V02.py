"""Reuse V01 successful probes; fix only the short-run observation-window setup."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from usv_uuc_cooperation.experiments.x2_comparison import load_config
from usv_uuc_cooperation.planning.dstar_original import OriginalDStarSession
from usv_uuc_cooperation.metrics.original_dstar_check import check_original_dstar
from usv_uuc_cooperation.control.x2_executor import simulate


def run(prior, output):
    old = json.loads((prior / '核查摘要.json').read_text())
    assert old['status'] == 'FAILED' and old['error_class'] == 'AssertionError'
    for field in ['closed_node_and_upstream_dependency', 'unreachable_then_recovery_same_state',
                  'weighted_symmetric_changes_and_moving_start', 'same_L2_local_input_order_and_costs',
                  'new_goal_isolation_and_old_cache_reactivation', 'future_table_rejection']:
        assert (old[field].get('status') == 'PASS') if isinstance(old[field], dict) else old[field].startswith('PASS')
    output.mkdir(parents=True, exist_ok=False)
    def save(name, data):
        (output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    c = load_config()
    check_original_dstar(c, json.loads((prior / 'X2局部查询.json').read_text()),
                        json.loads((prior / 'X2局部图版本.json').read_text()))
    short = copy.deepcopy(c)
    short['cutoff_time_s'] = short['observation_duration_s'] = 8
    planner = OriginalDStarSession(short)
    result = simulate(short, short['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner)
    check = check_original_dstar(short, planner.rows, planner.graphs)
    assert len(result['snapshots']) == 9 and len(result['steps']) == 48
    assert result['initial_queues'] == c['x2']['initial_queues']
    assert any(r['time_s'] == 6 and r['mode'] == 'SAME_GOAL_REPAIR' for r in planner.rows)
    for name, data in [('教师8秒_结果.json', result), ('教师8秒_搜索记录.json', planner.rows),
                       ('教师8秒_图版本.json', planner.graphs), ('搜索独立核查.json', check)]:
        save(name, data)
    report = {**old, 'status': 'PASS', 'error_class': None, 'error': None,
        'prior_report': str(prior / '核查摘要.json'),
        'prior_artifact_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in prior.glob('*.json')},
        'fix_scope': 'teacher harness: cutoff_time_s and observation_duration_s must both equal 8; production source unchanged',
        'saved_V01_local_queries_rechecked': 'PASS',
        'teacher_8s_connectivity': {'status': 'PASS', 'runs': 1, 'snapshots': 9,
            'actual_steps': 48, 'queries': len(planner.rows), 'scope': 'initial queues + t6 observation, no full 120s teacher simulation'}}
    save('核查摘要.json', report)
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prior', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(run(a.prior, a.output), ensure_ascii=False))
