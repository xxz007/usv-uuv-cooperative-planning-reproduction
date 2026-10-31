"""One fixed-target incremental segment and one complete D* Lite fleet run."""
import copy
import json
import sys
from collections import Counter

from usv_uuc_cooperation.config import (PROJECT_ROOT, VERSIONS_ROOT, FRAMEWORK_VERSION,
                                         read_json, new_run_directory)
from usv_uuc_cooperation.control.x2_executor import simulate
from usv_uuc_cooperation.experiments.x2_comparison import load_config, CONFIG_PATH
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest, write_csv
from usv_uuc_cooperation.metrics.x2_check import observed_cells, check_execution
from usv_uuc_cooperation.metrics.dstar_check import check_dstar
from usv_uuc_cooperation.planning.dstar_lite import DStarSession


def local_segment(config):
    """Same-map T08->T19 leg; time-labelled queries, not fictitious boat motion."""
    physical = {'map': config['map']}
    planner = DStarSession(config)
    planner.context(0, 'LOCAL_U1_T03')
    planner(config['starts_m']['U1'], config['tasks_m']['T03'], physical, planner.graph.blocked)
    planner.route_context('LOCAL_T08_T19', 'T19', 'FIXED_TARGET_SEGMENT')
    start, goal = config['tasks_m']['T08'], config['tasks_m']['T19']
    planner.context(0, 'LOCAL_SEGMENT_INITIAL')
    planner(start, goal, physical, planner.graph.blocked)
    for t in (6, 18):
        if t == 18:
            # Current start advances along the previously repaired legal path.
            start = planner.rows[-1]['path'][1]
        cells, version = observed_cells(config, t)
        planner.observe(physical, cells, version)
        planner.context(t, 'LOCAL_SEGMENT_MAP_OBSERVATION')
        planner(start, goal, physical, cells)
    planner.context(18, 'LOCAL_NEW_GOAL')
    planner.route_context('LOCAL_T08_T19', 'T15', 'NEW_TARGET_DEMONSTRATION')
    planner(start, config['tasks_m']['T15'], physical, planner.graph.blocked)
    return planner


def totals(rows):
    modes = Counter(r['mode'] for r in rows)
    return {'queries': len(rows), 'modes': dict(modes),
            'logical_pops': sum(r['expanded_nodes'] for r in rows),
            'update_vertex_calls': sum(r['update_vertex_calls'] for r in rows),
            'search_ns': sum(r['search_ns'] for r in rows),
            'state_initialization_ns': sum(r['state_initialization_ns'] for r in rows),
            'graph_diff_ns': sum(r['graph_diff_ns'] for r in rows),
            'scope': 'all fleet queries including unselected connectors/dispatch/budget lookahead; local segment excluded',
            'km_scope': 'key offset for query-start changes; includes hypothetical connectors/lookahead, not boat distance'}


def summary(local, groups, costs):
    text = ['# X2-L2｜D* Lite增量重规划', '',
            f'{FRAMEWORK_VERSION} · 输入X2-V01 · 仅一组完整D* Lite执行。', '',
            '## 本轮关注：局部固定目标段（T08→T19，同一海域）', '',
            '这是局部算法查询，不是六艇实际运动记录。仅t18将起点推进到先前修复路径的下一节点。', '',
            '| 时刻/操作 | 状态编号与模式 | g(start)=rhs(start)/m | 同图A*核对 |',
            '|---|---|---:|---|']
    for r in local.rows[1:]:
        text.append(f"| t{r['time_s']}，{r['start']}→{r['goal']} | {r['state_id']} / {r['mode']} | {r['search_cost_m']:.9f} | PASS |")
    repair = local.rows[2]
    before = {(x, y): (g, rhs) for x, y, g, rhs in repair['after_edge_updates']['g_rhs']}
    a = tuple(repair['start'])
    text += ['', f"t6边更新后、修复前，当前起点g/rhs={before[a]}；本次改变{len(repair['changed_edges'])}条有向边、重算{len(repair['changed_tails'])}个边起点。",
             f"t6保留状态编号{repair['state_id']}，逻辑队列弹出{repair['expanded_nodes']}次；不是重新运行A*。",
             '三项关注字段：state_id/mode、g/rhs变化、同图A*核对。移动起点更新km；换成新目标建立新的全∞g状态/目标rhs=0，不跨目标混用。', '',
             '## 完整六艇22点实际执行（教师核对）', '',
             '| 算法 | 完成任务 | 实际总航程/m | 团队完成时间/s |', '|---|---:|---:|---:|']
    s = groups['summary']
    text.append(f"| DstarLite | {s['completed_task_count']}/{s['known_task_count']} | {s['actual_total_distance_m']:.9f} | {s['team_completion_time_s'] if s['team_completion_time_s'] is not None else 'null'} |")
    text += ['', '同20初始+t11两任务，t24 U4故障，t6禁止(12,3)/t18撤销，120秒全窗。未来事件只由执行环境读取。',
             '每个艇/目标拥有独立正确缓存；新目标初始化记GOAL_RESET，旧目标缓存返回记cache_reactivated，图更新后按当前图修复。',
             'ACTUAL_HEAD、LOOKAHEAD_TAIL和DISPATCH_CANDIDATE分别标记当前航段、后续预算航段和调度候选；km不是实际航程。', '',
             '## 完整规划工作量', '',
             f"全部查询{costs['queries']}次，模式计数{costs['modes']}。",
             f"逻辑队列弹出{costs['logical_pops']}次；UpdateVertex调用{costs['update_vertex_calls']}次。",
             f"状态初始化{costs['state_initialization_ns']/1e6:.6f}ms；更新/修复循环{costs['search_ns']/1e6:.6f}ms；图差分{costs['graph_diff_ns']/1e6:.6f}ms（分列）。",
             'D*逻辑弹出包括旧key重入，不等同于A*有效扩展。搜索计时排除图构建/差分、状态快照、路径提取、独立A*核查、IO和绘图；一次数据不支持普遍快慢结论。', '',
             '[局部真实状态](局部增量状态.json) · [局部核查](局部核查.json) · [完整核查](统一核查.json) · [运行来源](运行元数据.json)', '',
             '本人只需报告首表的关键状态变化并解释本课新增边界；教师完整核查不代替本人运行/解释。效果由教师从保存日志生成，不重跑模拟。']
    return '\n'.join(text) + '\n'


def run(run_id):
    from usv_uuc_cooperation.main import save, assert_frozen_core
    assert_frozen_core()
    config = load_config()
    folder = new_run_directory('X2', run_id)
    try:
        save(folder / '统一配置.json', config)
        local = local_segment(config)
        save(folder / '局部增量状态.json', local.rows)
        local_check = check_dstar(config, local.rows, local.graphs)
        save(folder / '局部核查.json', local_check)
        print('局部固定目标状态/移动起点/新目标重置与同图A*核查PASS', flush=True)
        planner = DStarSession(config)
        result = simulate(config, config['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner)
        result['summary']['actual_total_distance_m'] = sum(b['distance_m'] for b in result['summary']['boat_totals'].values())
        result.update(framework=FRAMEWORK_VERSION, adjacency=8, input_version='X2-V01')
        part = folder / 'DstarLite'
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
        checks = {'search': check_dstar(config, searches, read_json(part / '图版本.json')),
                  'execution': check_execution(saved, searches)}
        save(part / '核查.json', checks)
        costs = totals(searches)
        save(folder / '搜索费用.json', {'DstarLite': costs})
        save(folder / '指标.json', {'DstarLite': saved['summary']})
        save(folder / '统一核查.json', {'status': 'PASS', 'local': local_check, 'groups': {'DstarLite': checks},
              'learner_acceptance': 'PENDING_LEARNER_RUN_CONFIRMATION_AND_EXPLANATION', 'visual_status': 'PENDING'})
        save(folder / '运行元数据.json', {'framework': FRAMEWORK_VERSION, 'lesson': 'X2-L2',
              'input_version': 'X2-V01', 'config_sha256': digest(CONFIG_PATH),
              'source_manifest_sha256': digest(VERSIONS_ROOT / FRAMEWORK_VERSION / '源码配置_SHA256.txt'),
              'command': sys.argv, 'python': sys.executable, 'output': str(folder),
              'group_execution_count': 1, 'status': 'COMPLETE',
              'reference': 'offline repeated A* for every query, no full A* simulation or independent A* figure',
              'initial_assignment': 'accepted X1 eight-neighbor initial solution reused, solver runs 0',
              'role': 'learner/teacher invocation evidence recorded separately in lesson record'})
        (folder / '摘要.md').write_text(summary(local, saved, costs))
        write_manifest(part)
        write_manifest(folder)
        print(f"DstarLite完整保存核查PASS，完成{saved['summary']['completed_task_count']}/22", flush=True)
    except Exception as exc:
        save(folder / '失败.json', {'status': 'FAILED', 'error': str(exc), 'output': str(folder)})
        raise
    return folder
