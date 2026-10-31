"""Independently check selected saved data; no simulation or platform launch."""
from pathlib import Path
import argparse
import json
import math
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=root / 'results/x2_algorithms/reference')
    args = parser.parse_args()
    sys.path.insert(0, str(root))
    from usv_uuc_cooperation.main import assert_frozen_core
    from usv_uuc_cooperation.experiments.p4_integrated import verify_manifest
    from usv_uuc_cooperation.metrics.x2_check import check_searches, check_execution, check_pair
    assert_frozen_core()
    groups = {}
    for name in ('Astar', 'Dijkstra'):
        part = args.run / name
        result = json.loads((part / '结果.json').read_text())
        searches = json.loads((part / '搜索记录.json').read_text())
        graphs = json.loads((part / '图版本.json').read_text())
        check_searches(result['config'], searches, graphs)
        check_execution(result, searches)
        s = result['summary']
        assert s['completed_task_count'] == s['known_task_count'] == 22
        assert s['team_completion_time_s'] == 54
        assert math.isclose(s['actual_total_distance_m'], 122.59818219586397, abs_tol=1e-8)
        groups[name] = result
    check_pair(groups)
    p3 = root / 'results/p3_baseline/reference'
    verify_manifest(p3)
    for name, count, time, energy in (('SUCCESS', 20, 48, 380), ('RESTRICTED', 16, None, 314)):
        s = json.loads((p3 / f'{name}_结果.json').read_text())['summary']
        assert s['completed_task_count'] == count and s['team_completion_time_s'] == time
        assert math.isclose(s['total_all_boat_energy_NEU'], energy, abs_tol=1e-8)
    print('saved results PASS: Astar/Dijkstra and original P3 success/failure; no simulation')


if __name__ == '__main__':
    main()
