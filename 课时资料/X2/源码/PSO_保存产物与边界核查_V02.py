"""Recheck saved wiring without simulation; test blocked decoding/search failures."""
import argparse
import copy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from usv_uuc_cooperation.config import PROJECT_ROOT, read_json
from usv_uuc_cooperation.metrics.pso_check import check_pso, check_queries, independent_decode
from usv_uuc_cooperation.planning.pso_search import PSOSession, decode


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    source = PROJECT_ROOT / '课时资料/X2/验收/PSO_教师核查_V01'
    result, rows, graphs = (read_json(source / f'教师26秒_{name}.json') for name in ('结果', '搜索记录', '图版本'))
    policy = read_json(PROJECT_ROOT / 'usv_uuc_cooperation/configs/X2-PSO-V01.json')
    checks = check_pso(result, rows, graphs, policy)
    map_config = copy.deepcopy(result['config']['map'])
    map_config.update(x_range=[0, 4], y_range=[0, 4], circles=[], safety_buffer_m=0, forbidden_cells=[[2, y] for y in range(5)])
    # forbidden_nodes uses explicit blocked_cells, supplied as current observation.
    c = {'map': map_config}
    wall = {(2, y) for y in range(5)}
    planner = PSOSession(c, policy)
    planner.observe(c, wall, 1)
    assert planner([0, 2], [4, 2], c, wall) is None
    failed = planner.rows[-1]
    assert failed['status'] == 'NO_FEASIBLE_PATH_WITHIN_BUDGET' and failed['evaluations'] == 2440
    assert all(h['best_feasible_m'] is None and all(p is None for p in h['personal_best_m']) for h in failed['history'])
    assert failed['distance_m'] is None and failed['infeasible_not_proof_of_unreachability']
    # Corner obstruction, half-up rounding and loop removal, checked independently.
    planner.observe(c, {(1, 0)}, 2)
    vector = [0., 0.] * 8
    corner = decode(vector, (0, 0), (1, 1), planner.graph)
    assert not corner['feasible'] and corner['invalid_edges'] == 1
    planner.observe(c, set(), 3)
    for vector in ([.5, .5] * 8, [3., 0., 0., 0.] + [4., 4.] * 6):
        candidate = decode(vector, (0, 0), (4, 4), planner.graph)
        assert candidate['path'] == independent_decode(vector, [0, 0], [4, 4])
        assert candidate['feasible'] and len(candidate['path']) == len(set(map(tuple, candidate['path'])))
    caught = []
    for label in ('illegal_path', 'false_cost', 'pbest_worsens', 'wrong_seed', 'budget_overrun'):
        modified = copy.deepcopy(rows)
        r = next(r for r in modified if r['history'] and len(r['history']) > 1 and r['path'])
        if label == 'illegal_path':
            r['path'].insert(1, [999, 999])
        elif label == 'false_cost':
            r['distance_m'] = 0.
        elif label == 'pbest_worsens':
            index = next(i for i, p in enumerate(r['history'][0]['personal_best_m']) if p is not None)
            r['history'][1]['personal_best_m'][index] = r['history'][0]['personal_best_m'][index] + 1
        elif label == 'wrong_seed':
            r['derived_seed'] += 1
        else:
            r['evaluations'] = 2441
        try:
            check_queries(result['config'], modified, graphs, policy)
        except AssertionError:
            caught.append(label)
        else:
            raise AssertionError(f'Checker accepted {label}')
    report = {'status': 'PASS', 'saved_short_wiring_recheck': checks, 'simulation_reruns': 0,
        'blocked_graph_failed_query': failed, 'corner_rounding_loop_decoder': 'PASS', 'invalid_evidence_rejected': caught,
        'scope': 'new isolated query and decoder edge cases; existing 26s physical simulation reused'}
    (output / '核查摘要.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return {'status': 'PASS', 'simulation_reruns': 0, 'negative_checks': caught, 'blocked_query_evaluations': failed['evaluations']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.output), ensure_ascii=False))
