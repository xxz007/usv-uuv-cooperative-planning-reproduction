"""One learner invocation: independent local searches and two real fleet runs."""
import copy
import json
from pathlib import Path
import sys

from usv_uuc_cooperation.config import PACKAGE_ROOT, PROJECT_ROOT, VERSIONS_ROOT, FRAMEWORK_VERSION, read_json, new_run_directory
from usv_uuc_cooperation.control.x2_executor import simulate
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest, write_csv
from usv_uuc_cooperation.planning.x2_search import SearchSession

CONFIG_PATH = PACKAGE_ROOT / 'configs/X2-V01.json'


def load_config():
    c = read_json(CONFIG_PATH)
    for name, sha in c['x2']['source_sha256'].items():
        if digest(PROJECT_ROOT / name) != sha:
            raise ValueError(f'Frozen initial evidence changed: {name}')
    original = read_json(PROJECT_ROOT / c['x1']['source_path'])
    if digest(PROJECT_ROOT / c['x1']['source_path']) != c['x1']['source_sha256']:
        raise ValueError('P4 source scenario changed')
    same = copy.deepcopy(c)
    same.pop('x1')
    same.pop('x2')
    same['config_version'] = original['config_version']
    same['map'].pop('adjacency')
    same['map']['movement'] = original['map']['movement']
    same['events'] = [e for e in same['events'] if e['type'] != 'MAP_UPDATE']
    assert same == original, 'X2 changed a shared physical/scheduling parameter'
    x = c['x2']
    assigned = [t for tasks in x['initial_assignment'].values() for t in tasks]
    assert sorted(assigned) == sorted(c['tasks_m']) and len(assigned) == 20
    assert all(set(x['initial_assignment'][b]) == set(x['initial_queues'][b]) for b in c['uuc_ids'])
    known_points = list(c['tasks_m'].values()) + [e['position_m'] for e in c['events'] if e['type'] == 'TASK_ARRIVAL']
    blocked = forbidden_nodes(c)
    for event in c['events']:
        if event['type'] == 'MAP_UPDATE' and event['add_cells']:
            for cell in event['add_cells']:
                assert tuple(cell) not in blocked and cell not in known_points
                for boat, point in c['starts_m'].items():
                    distance = sum(max(abs(point[k] - cell[k]) - .5, 0) ** 2 for k in (0, 1)) ** .5
                    assert distance > event['time_s'] * (c['speed_m_per_s'][boat] + .2)
    return c


def search_totals(rows):
    actual = [r for r in rows if r['reason'] != 'LOCAL_U1_T03']
    return {'queries': len(actual), 'expanded_nodes': sum(r['expanded_nodes'] for r in actual),
            'search_ns': sum(r['search_ns'] for r in actual),
            'scope': 'all execution-planning queries, including candidate connectors and dispatch/budget checks; local demonstration excluded'}


def summary(config, groups, local, totals):
    lines = ['# X2-L1｜A*与Dijkstra实际对照', '',
             f'{FRAMEWORK_VERSION} · 输入X2-V01 · 同八邻接图、固定X1初始归属/顺序、共同动态策略。', '',
             '## 本轮关注的三个字段（U1→T03，图版本0）', '',
             '| 算法 | distance_m 最短路/m | expanded_nodes 有效扩展 | search_ms 搜索/ms |',
             '|---|---:|---:|---:|']
    for name, r in local.items():
        lines.append(f"| {name} | {r['distance_m']:.9f} | {r['expanded_nodes']} | {r['search_ns'] / 1e6:.6f} |")
    lines += ['', f"起点{config['starts_m']['U1']}，目标T03={config['tasks_m']['T03']}。两组路径坐标是否完全相同：{local['Astar']['path'] == local['Dijkstra']['path']}。",
              '有效扩展指非过期堆弹出，包含目标；独立列出唯一节点数，不将有效弹出数冒称唯一节点数。',
              '计时只含搜索堆循环及内存扩展记录；不含图构建、起终点检查、路径恢复、文件IO或绘图。单次小图计时不能证明普遍快慢。', '',
              '## 完整1USV＋5UUV＋22点执行（教师核对）', '',
              '| 算法 | 完成数 | 实际总航程/m | 团队完成时间/s |', '|---|---:|---:|---:|']
    for name, result in groups.items():
        s = result['summary']
        lines.append(f"| {name} | {s['completed_task_count']}/{s['known_task_count']} | {s['actual_total_distance_m']:.9f} | {s['team_completion_time_s'] if s['team_completion_time_s'] is not None else 'null'} |")
    lines += ['', '全窗航程含六艇[0,120)实际位移、故障前移动和完工后中继；未全完成时间为null。',
              't6观察(12,3)临时禁行，t11发布两任务，t18撤销临时禁行，t24观察U4故障。未来表只由执行环境读取，规划仅接收当前图/位置/已发布任务。', '',
              '## 全部执行规划费用（不是只统计被选中的路线）', '',
              '| 算法 | 搜索查询数 | 有效扩展合计 | 搜索累计/ms | 图构建合计/ms（排除于搜索） |', '|---|---:|---:|---:|---:|']
    for name, t in totals.items():
        graph_ms = sum(g['build_ns'] for g in groups[name]['observed_graphs']) / 1e6
        lines.append(f"| {name} | {t['queries']} | {t['expanded_nodes']} | {t['search_ns']/1e6:.6f} | {graph_ms:.6f} |")
    lines += ['', '图构建在搜索前完成，两组采用相同规则；累计查询含未选中连接候选、余程预算和调度候选查询，局部U1→T03演示不计入累计。',
              '两算法可能选择不同的等成本路线，进而改变实际位置及后续事件响应；系统时间不由有效扩展数直接决定。', '',
              '[局部搜索路径与状态](局部搜索对照.json) · [完整核查](统一核查.json) · [输入](统一配置.json) · [运行来源](运行元数据.json)', '',
              '图由教师从本次保存的完整日志生成，无需再次运行模拟。机械PASS不代替本人运行/解释或图的视觉检查。']
    return '\n'.join(lines) + '\n'


def run_pair(run_id):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    from usv_uuc_cooperation.metrics.x2_check import check_searches, check_execution, check_pair
    assert_frozen_core()
    config = load_config()
    folder = new_run_directory('X2', run_id)
    try:
        save(folder / '统一配置.json', config)
        groups, local, totals, checks = {}, {}, {}, {}
        for name in ('Astar', 'Dijkstra'):
            print(f'{name}：局部搜索与完整六艇实际执行', flush=True)
            part = folder / name
            part.mkdir()
            planner = SearchSession(config, name)
            planner(config['starts_m']['U1'], config['tasks_m']['T03'], config, forbidden_nodes(config))
            local[name] = copy.deepcopy(planner.rows[-1])
            result = simulate(config, config['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner)
            result['summary']['actual_total_distance_m'] = sum(b['distance_m'] for b in result['summary']['boat_totals'].values())
            result.update(framework=FRAMEWORK_VERSION, adjacency=8, input_version='X2-V01')
            save(part / '结果.json', result)
            save(part / '搜索记录.json', planner.rows)
            save(part / '图版本.json', planner.graphs)
            save(part / '指标.json', result['summary'])
            save(part / '事件与重规划.json', {'events': result['event_log'], 'replans': result['replans']})
            with (part / '实际控制与能量.jsonl').open('w') as stream:
                for row in result['steps']:
                    stream.write(json.dumps(row, ensure_ascii=False) + '\n')
            write_csv(part / '逐时通信.csv', result['links'])
            serialized, searches = read_json(part / '结果.json'), read_json(part / '搜索记录.json')
            checks[name] = {'search': check_searches(config, searches, read_json(part / '图版本.json')),
                            'execution': check_execution(serialized, searches)}
            totals[name] = search_totals(searches)
            save(part / '核查.json', checks[name])
            write_manifest(part)
            groups[name] = serialized
            print(f"{name}保存产物核查PASS，完成{result['summary']['completed_task_count']}/22", flush=True)
        assert abs(local['Astar']['distance_m'] - local['Dijkstra']['distance_m']) < 1e-9
        save(folder / '局部搜索对照.json', local)
        save(folder / '统一核查.json', {'status': 'PASS', 'groups': checks, 'pair': check_pair(groups),
                                     'learner_acceptance': 'PENDING_LEARNER_RUN_CONFIRMATION_AND_EXPLANATION', 'visual_check': 'PENDING'})
        save(folder / '搜索费用.json', totals)
        save(folder / '指标.json', {n: g['summary'] for n, g in groups.items()})
        (folder / '摘要.md').write_text(summary(config, groups, local, totals), encoding='utf-8')
        save(folder / '运行元数据.json', {'framework': FRAMEWORK_VERSION, 'lesson': 'X2-L1', 'input_version': 'X2-V01',
            'config_sha256': digest(CONFIG_PATH), 'source_manifest_sha256': digest(VERSIONS_ROOT / FRAMEWORK_VERSION / '源码配置_SHA256.txt'),
            'command': sys.argv, 'python': sys.executable, 'output': str(folder), 'group_execution_count': 2,
            'initial_assignment': 'frozen X1-L4 eight-neighbor solution reused; solver not rerun', 'status': 'COMPLETE',
            'role': 'invocation role confirmed in lesson record; run-id alone does not prove learner execution'})
        write_manifest(folder)
    except Exception as exc:
        save(folder / '失败.json', {'status': 'FAILED', 'error': str(exc), 'output': str(folder)})
        raise
    return folder
