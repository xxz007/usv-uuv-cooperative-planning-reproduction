"""Recheck published saved data; no simulation, solver or platform required."""
from pathlib import Path
import json
import math
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    from usv_uuc_cooperation.main import assert_frozen_core
    from usv_uuc_cooperation.experiments.p4_integrated import verify_manifest
    from usv_uuc_cooperation.metrics.x2_check import check_searches, check_execution, check_pair
    assert_frozen_core()
    run = root / '输出文件/阶段结果/X2/X2-V01/runs/P6_clean_X2_Astar_Dijkstra_V01'
    verify_manifest(run)
    groups = {}
    for name in ('Astar', 'Dijkstra'):
        part = run / name
        result = json.loads((part / '结果.json').read_text())
        searches = json.loads((part / '搜索记录.json').read_text())
        graphs = json.loads((part / '图版本.json').read_text())
        check_searches(result['config'], searches, graphs)
        check_execution(result, searches)
        summary = result['summary']
        assert summary['completed_task_count'] == summary['known_task_count'] == 22
        assert summary['team_completion_time_s'] == 54
        assert math.isclose(summary['actual_total_distance_m'], 122.59818219586397, abs_tol=1e-8)
        groups[name] = result
    check_pair(groups)
    p3 = root / '输出文件/阶段结果/P3/P3-V01/runs/框架迁移核查_教师V01'
    verify_manifest(p3)
    for name, count, time, energy in (('SUCCESS', 20, 48, 380), ('RESTRICTED', 16, None, 314)):
        result = json.loads((p3 / f'{name}_结果.json').read_text())
        summary = result['summary']
        assert summary['completed_task_count'] == count
        assert summary['team_completion_time_s'] == time
        assert math.isclose(summary['total_all_boat_energy_NEU'], energy, abs_tol=1e-8)
    print('P6 saved data PASS: 2 X2 groups; original P3 success/failure; no new simulations')


if __name__ == '__main__':
    main()
