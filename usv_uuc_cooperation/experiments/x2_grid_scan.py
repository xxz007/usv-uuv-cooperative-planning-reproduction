"""One complete grid-scan task-order group with the existing A* connector."""
import json
import sys
from collections import Counter

from usv_uuc_cooperation.config import PACKAGE_ROOT, VERSIONS_ROOT, FRAMEWORK_VERSION, read_json, new_run_directory
from usv_uuc_cooperation.control.x2_executor import simulate
from usv_uuc_cooperation.experiments.x2_comparison import load_config, CONFIG_PATH
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest, write_csv
from usv_uuc_cooperation.metrics.scan_check import check_scan
from usv_uuc_cooperation.planning.scan_routes import ScanAStarSession, scan_rebuild

POLICY_PATH = PACKAGE_ROOT / 'configs/X2-Scan-V01.json'
GROUP = 'GridScanAstar'


def summary(config, result, checks, costs):
    text = ['# X2完整扩展课程｜网格扫描任务遍历＋A*连接', '',
        f'{FRAMEWORK_VERSION} · 共同输入X2-V01 · 排序策略X2-Scan-V01。', '',
        '## 本轮关注：访问顺序变了，单段连接仍为A*', '',
        '| 艇 | 原A*基线初始顺序 | 扫描后的初始顺序 |', '|---|---|---|']
    for boat in config['uuc_ids']:
        text.append(f"| {boat} | {'→'.join(config['x2']['initial_queues'][boat]) or '空'} | {'→'.join(result['initial_queues'][boat]) or '空'} |")
    text += ['', '只关注：U4扫描后初始顺序、连接器Astar、完整完成任务数。排序按y行升序，以map.y_range[0]为行号原点，偶数行x升序/奇数行x降序，同位置按任务ID。',
        '健康艇已经到达/正在服务的任务固定队首；新增/故障任务仍由共同贪心尾距、容量和预算规则接收，随后重排并重新核查完整路线预算，不做隐藏回退。', '',
        '## 完整六艇22点结果', '', '| 组 | 完成任务 | 六艇全窗实际总航程/m | 团队完成时间/s |', '|---|---:|---:|---:|']
    s = result['summary']
    text.append(f"| 网格扫描任务遍历＋A* | {s['completed_task_count']}/{s['known_task_count']} | {s['actual_total_distance_m']:.9f} | {s['team_completion_time_s'] if s['team_completion_time_s'] is not None else 'null'} |")
    text += ['', '主案例/服务/容量/预算/流场和t6禁行、t11两任务、t18撤销、t24 U4故障保持。总航程为六艇[0,120)实际位移，含故障前移动和完工后中继；未全完成时间为null。', '',
        '## 动态扫描顺序与服务保护（教师核查）', '']
    for row in checks['ordering_checks']:
        protected = {b: task for b, task in row['protected_heads'].items() if task is not None}
        text.append(f"- t{row['time_s']}：保护队首{protected or '无'}；新增接收后重排={row['post_dispatch_resort']}；顺序核查PASS。")
    text += ['', '[排序决策](GridScanAstar/排序与动态决策.json) · [完整核查](统一核查.json)', '',
        '## 计算与评价范围', '',
        f"全部A*查询{costs['queries']}次，有效扩展{costs['logical_pops']}次，搜索{costs['search_ns']/1e6:.6f}ms。查询角色{costs['query_roles']}。",
        '费用包括原队列预算/共同调度候选及接收后扫描重排需要的连接，不只计最终被选的航段。图构建、排序/调度与搜索计时分开，不能把排序层收益称为新的A*搜索改进。',
        '只服务已发布且已分配的任务点，不是区域全格覆盖；固定蛇形顺序不保证任务访问顺序全局最优。', '',
        '[运行来源](运行元数据.json) · [排序策略](排序策略.json)', '',
        '本人运行与解释按完整课程连续记录登记。效果图随后从保存日志生成，旧基线及原始D*不重跑；当前完整X2仍进行中。']
    return '\n'.join(text) + '\n'


def run(run_id):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    assert_frozen_core()
    config, policy = load_config(), read_json(POLICY_PATH)
    assert policy['version'] == 'X2-Scan-V01'
    folder = new_run_directory('X2', run_id)
    try:
        save(folder / '统一配置.json', config)
        save(folder / '排序策略.json', policy)
        planner = ScanAStarSession(config)
        result = simulate(config, config['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner,
                          rebuild_routes=scan_rebuild)
        result['summary']['actual_total_distance_m'] = sum(b['distance_m'] for b in result['summary']['boat_totals'].values())
        result.update(framework=FRAMEWORK_VERSION, adjacency=8, input_version='X2-V01',
                      algorithm=GROUP, connector_algorithm='Astar', ordering_policy=policy)
        part = folder / GROUP
        part.mkdir()
        save(part / '结果.json', result)
        save(part / '搜索记录.json', planner.rows)
        save(part / '图版本.json', planner.graphs)
        save(part / '指标.json', result['summary'])
        save(part / '排序与动态决策.json', result['replans'])
        save(part / '事件与重规划.json', {'events': result['event_log'], 'replans': result['replans']})
        with (part / '实际控制与能量.jsonl').open('w') as f:
            for row in result['steps']:
                f.write(json.dumps(row, ensure_ascii=False) + '\n')
        write_csv(part / '逐时通信.csv', result['links'])
        saved, rows = read_json(part / '结果.json'), read_json(part / '搜索记录.json')
        checks = check_scan(saved, rows, read_json(part / '图版本.json'))
        save(part / '核查.json', checks)
        costs = {'queries': len(rows), 'logical_pops': sum(r['expanded_nodes'] for r in rows),
            'search_ns': sum(r['search_ns'] for r in rows), 'graph_build_ns': sum(g['build_ns'] for g in planner.graphs),
            'query_roles': dict(Counter(r['query_role'] for r in rows)),
            'ordering_phases': dict(Counter(r['ordering_phase'] for r in rows))}
        save(folder / '搜索费用.json', {GROUP: costs})
        save(folder / '指标.json', {GROUP: saved['summary']})
        save(folder / '统一核查.json', {'status': 'PASS', 'groups': {GROUP: checks},
            'learner_acceptance': 'PENDING_LEARNER_RUN_AND_READOUT', 'visual_status': 'PENDING'})
        save(folder / '运行元数据.json', {'framework': FRAMEWORK_VERSION, 'course': 'X2', 'topic': 'grid_scan_task_order',
            'lesson': 'X2-Scan', 'algorithm': GROUP, 'connector_algorithm': 'Astar', 'input_version': 'X2-V01',
            'config_sha256': digest(CONFIG_PATH), 'ordering_policy_sha256': digest(POLICY_PATH),
            'source_manifest_sha256': digest(VERSIONS_ROOT / FRAMEWORK_VERSION / '源码配置_SHA256.txt'),
            'command': sys.argv, 'python': sys.executable, 'output': str(folder),
            'group_execution_count': 1, 'status': 'COMPLETE',
            'changed_factor': 'task order strategy; initial assignment and Astar connector unchanged',
            'role': 'invocation ownership confirmed separately in complete-course record', 'solver_runs': 0})
        (folder / '摘要.md').write_text(summary(config, saved, checks, costs))
        write_manifest(part)
        write_manifest(folder)
        print(f"扫描遍历＋A*完整保存核查PASS，完成{saved['summary']['completed_task_count']}/22", flush=True)
    except Exception as exc:
        save(folder / '失败.json', {'status': 'FAILED', 'error': str(exc), 'output': str(folder)})
        raise
    return folder
