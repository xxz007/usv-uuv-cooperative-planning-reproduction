"""Original D* topic within the complete X2 extension: local repair + one fleet."""
from collections import Counter
import json
import sys

from usv_uuc_cooperation.config import (FRAMEWORK_VERSION, VERSIONS_ROOT, read_json, new_run_directory)
from usv_uuc_cooperation.control.x2_executor import simulate
from usv_uuc_cooperation.experiments.x2_comparison import load_config, CONFIG_PATH
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest, write_csv
from usv_uuc_cooperation.metrics.x2_check import observed_cells, check_execution
from usv_uuc_cooperation.metrics.original_dstar_check import check_original_dstar
from usv_uuc_cooperation.planning.dstar_original import OriginalDStarSession, REFERENCE


def local_segment(config):
    physical = {'map': config['map']}
    planner = OriginalDStarSession(config)
    planner.context(0, 'LOCAL_U1_T03')
    planner(config['starts_m']['U1'], config['tasks_m']['T03'], physical, planner.graph.blocked)
    planner.route_context('LOCAL_T08_T19', 'T19', 'FIXED_TARGET_SEGMENT')
    start, goal = config['tasks_m']['T08'], config['tasks_m']['T19']
    planner.context(0, 'LOCAL_SEGMENT_INITIAL')
    planner(start, goal, physical, planner.graph.blocked)
    for t in (6, 18):
        if t == 18:
            # Accepted D* Lite local query start, independent of tie choices here.
            start = [11, 2]
        cells, version = observed_cells(config, t)
        planner.observe(physical, cells, version)
        planner.context(t, 'LOCAL_SEGMENT_MAP_OBSERVATION')
        planner(start, goal, physical, cells)
    planner.context(18, 'LOCAL_NEW_GOAL')
    planner.route_context('LOCAL_T08_T19', 'T15', 'NEW_TARGET_DEMONSTRATION')
    planner(start, config['tasks_m']['T15'], physical, planner.graph.blocked)
    return planner


def totals(rows):
    return {'queries': len(rows), 'modes': dict(Counter(r['mode'] for r in rows)),
        'logical_pops': sum(r['expanded_nodes'] for r in rows),
        'raise_pops': sum(e['category'] == 'RAISE' for r in rows for e in r['expansion_trace']),
        'lower_pops': sum(e['category'] == 'LOWER' for r in rows for e in r['expansion_trace']),
        'queue_insertions': sum(r['queue_insertions'] for r in rows),
        'search_ns': sum(r['search_ns'] for r in rows),
        'state_initialization_ns': sum(r['state_initialization_ns'] for r in rows),
        'graph_diff_ns': sum(r['graph_diff_ns'] for r in rows),
        'scope': 'all fleet queries including candidate connectors/dispatch/budget lookahead; local demonstration excluded'}


def summary(local, result, costs):
    lines = ['# X2完整扩展课程｜原始D*运行摘要', '',
        f'{FRAMEWORK_VERSION} · 输入X2-V01 · 一组完整原始D*执行；课程仍进行中。', '',
        '## 这轮查看三个字段', '',
        'state_id/mode、h(start)路径代价、RAISE/LOWER处理次数。以下为局部查询，不是艇的实际运动。', '',
        '| 操作 | state_id / mode | h(start) / m | RAISE / LOWER |', '|---|---|---:|---|']
    for r in local.rows[1:]:
        counts = Counter(e['category'] for e in r['expansion_trace'])
        cost = f"{r['search_cost_m']:.9f}" if r['search_cost_m'] is not None else 'null'
        lines.append(f"| t{r['time_s']}，{r['start']}→{r['goal']} | {r['state_id']} / {r['mode']} | {cost} | {counts['RAISE']} / {counts['LOWER']} |")
    repair = local.rows[2]
    dependencies = [u for u in repair['updates'] if u['reason'] == 'BACKPOINTER_DEPENDENCY']
    lines += ['', f"t6共变化{len(repair['changed_edges'])}条有向边，{len(dependencies)}次回指针依赖更新；保留状态{repair['state_id']}，未清空h/p重新搜索。",
        '请结合t6前后h(start)和回指针依赖更新，解释为什么封路会改变其他节点的代价。不需要逐项抄日志。', '',
        '[局部状态与传播日志](局部增量状态.json) · [局部核查](局部核查.json)', '',
        '## 完整六艇22点实际执行', '',
        '| 算法 | 完成任务 | 全窗实际总航程/m | 团队完成时间/s |', '|---|---:|---:|---:|']
    s = result['summary']
    lines.append(f"| 原始D* | {s['completed_task_count']}/{s['known_task_count']} | {s['actual_total_distance_m']:.9f} | {s['team_completion_time_s'] if s['team_completion_time_s'] is not None else 'null'} |")
    lines += ['', '六艇[0,120)全窗航程包含故障前移动及完工后USV中继；20初始+t11两任务、t24 U4故障、t6禁行/t18撤销与原物理规则保持。', '',
        '## 初始化、修复与工作量', '',
        f"全部查询{costs['queries']}次，模式{costs['modes']}；PROCESS-STATE弹出{costs['logical_pops']}次（RAISE {costs['raise_pops']}、LOWER {costs['lower_pops']}）。",
        f"初始化{costs['state_initialization_ns']/1e6:.6f}ms；更新/修复{costs['search_ns']/1e6:.6f}ms；图差分{costs['graph_diff_ns']/1e6:.6f}ms，分列。",
        '每个owner/目标独立缓存，首次新目标显式初始化，旧目标缓存返回先同步当前图；移动起点不需要D* Lite的km。',
        'PROCESS-STATE弹出和D* Lite逻辑弹出、A*有效扩展定义不同，不据单次结果推断普遍快慢。计时包含内存追踪，排除图构建/差分、状态快照、路径提取、独立核查、IO/绘图。', '',
        '## 原始版本与实现适配', '', REFERENCE, '',
        'h/p、回指针、OPEN与PROCESS-STATE按1993原始机制；CLOSED节点INSERT先保留旧h再赋新h，与p的定义及作者1994 Basic D*插入规则一致，未采用Focussed D*或D* Lite。',
        '当前仅适配共同的对称1/√2图；禁行边以∞保留拓扑依赖，恢复时传播降低代价；独立A*和全图参照仅用于保存后的核查。', '',
        '[完整核查](统一核查.json) · [运行来源](运行元数据.json)', '',
        '本人运行与解释需课程记录确认；程序PASS不等于本人/完整课程验收。效果由教师从已保存日志集中生成，不为出图重跑模拟。']
    return '\n'.join(lines) + '\n'


def run(run_id):
    from usv_uuc_cooperation.main import save, assert_frozen_core
    assert_frozen_core()
    config = load_config()
    folder = new_run_directory('X2', run_id)
    try:
        save(folder / '统一配置.json', config)
        local = local_segment(config)
        save(folder / '局部增量状态.json', local.rows)
        local_check = check_original_dstar(config, local.rows, local.graphs)
        save(folder / '局部核查.json', local_check)
        print('原始D*局部传播/移动起点/新目标与独立最短路核查PASS', flush=True)
        planner = OriginalDStarSession(config)
        result = simulate(config, config['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner)
        result['summary']['actual_total_distance_m'] = sum(b['distance_m'] for b in result['summary']['boat_totals'].values())
        result.update(framework=FRAMEWORK_VERSION, adjacency=8, input_version='X2-V01')
        part = folder / 'Dstar'
        part.mkdir()
        save(part / '结果.json', result)
        save(part / '搜索记录.json', planner.rows)
        save(part / '图版本.json', planner.graphs)
        save(part / '指标.json', result['summary'])
        save(part / '事件与重规划.json', {'events': result['event_log'], 'replans': result['replans']})
        with (part / '实际控制与能量.jsonl').open('w') as f:
            for row in result['steps']:
                f.write(json.dumps(row, ensure_ascii=False) + '\n')
        write_csv(part / '逐时通信.csv', result['links'])
        saved, searches = read_json(part / '结果.json'), read_json(part / '搜索记录.json')
        checks = {'search': check_original_dstar(config, searches, read_json(part / '图版本.json')),
                  'execution': check_execution(saved, searches)}
        save(part / '核查.json', checks)
        costs = totals(searches)
        save(folder / '搜索费用.json', {'Dstar': costs})
        save(folder / '指标.json', {'Dstar': saved['summary']})
        save(folder / '统一核查.json', {'status': 'PASS', 'local': local_check, 'groups': {'Dstar': checks},
            'learner_acceptance': 'PENDING_LEARNER_RUN_CONFIRMATION_AND_EXPLANATION', 'visual_status': 'PENDING'})
        save(folder / '运行元数据.json', {'framework': FRAMEWORK_VERSION, 'course': 'X2', 'topic': 'original_dstar',
            'lesson': 'X2-Dstar', 'input_version': 'X2-V01', 'config_sha256': digest(CONFIG_PATH),
            'source_manifest_sha256': digest(VERSIONS_ROOT / FRAMEWORK_VERSION / '源码配置_SHA256.txt'),
            'command': sys.argv, 'python': sys.executable, 'output': str(folder),
            'group_execution_count': 1, 'status': 'COMPLETE', 'reference': REFERENCE,
            'initial_assignment': 'accepted X1 eight-neighbor initial solution reused; solver runs 0',
            'role': 'invocation ownership confirmed separately in complete-course record'})
        (folder / '摘要.md').write_text(summary(local, saved, costs))
        write_manifest(part)
        write_manifest(folder)
        print(f"原始D*完整保存核查PASS，完成{saved['summary']['completed_task_count']}/22", flush=True)
    except Exception as exc:
        save(folder / '失败.json', {'status': 'FAILED', 'error': str(exc), 'output': str(folder)})
        raise
    return folder
