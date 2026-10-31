"""Fixed five-seed local PSO statistics and one first-seed fleet execution."""
import json
import sys

from usv_uuc_cooperation.config import PACKAGE_ROOT, VERSIONS_ROOT, FRAMEWORK_VERSION, read_json, new_run_directory
from usv_uuc_cooperation.control.x2_executor import simulate
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.experiments.x2_comparison import load_config, CONFIG_PATH
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest, write_csv
from usv_uuc_cooperation.metrics.pso_check import check_pso, check_queries
from usv_uuc_cooperation.planning.pso_search import PSOSession
from usv_uuc_cooperation.planning.pso_routes import pso_rebuild

POLICY_PATH = PACKAGE_ROOT / 'configs/X2-PSO-V01.json'
GROUP = 'PSO'


def local_statistics(config, policy):
    samples, traces = [], []
    for seed in policy['seeds']:
        planner = PSOSession(config, policy, seed)
        planner.context(0, 'LOCAL_U1_T03')
        planner.route_context('U1', 'T03', 'LOCAL_DEMO')
        planner(config['starts_m']['U1'], config['tasks_m']['T03'], {'map': config['map']}, forbidden_nodes(config))
        check = check_queries(config, planner.rows, planner.graphs, policy)
        row = planner.rows[0]
        gap = check['feasible_gaps'][0] if check['feasible_gaps'] else None
        samples.append({'seed': seed, 'status': row['status'], 'distance_m': row['distance_m'],
            'gap_m': gap['gap_m'] if gap else None, 'exact_reference_m': gap['exact_m'] if gap else None,
            'evaluations': row['evaluations'], 'search_ns': row['search_ns'], 'iterations': row['iterations']})
        traces.append({'seed': seed, 'searches': planner.rows, 'graphs': planner.graphs, 'checks': check})
    return {'status': 'PASS', 'scope': 'same initial U1 to T03 query, five predeclared seeds; not five fleet runs',
        'successes': sum(s['distance_m'] is not None for s in samples), 'seed_count': len(samples), 'samples': samples}, traces


def summary(result, local, checks, costs):
    lines = ['# X2完整扩展课程｜粒子群路线搜索', '',
        f'{FRAMEWORK_VERSION} · 共同输入X2-V01 · 策略X2-PSO-V01。', '',
        '8个中间航点、40个粒子、最多60轮（每查询最多2440次评价）；固定直线/边界模板与随机初始化，无A*初始化、修复或失败回退。',
        'pbest和gbest仅保存已找到的合法最好路线；候选位置可以变差。gbest不代表已证明全局最优。', '',
        '## 本轮读取：5种子成功数、主种子局部差距、完整组完成数', '',
        f"同一初始U1→T03查询：{local['successes']}/{local['seed_count']}个种子找到合法路线。", '',
        '| 种子 | 合法路线/m | 与独立最短路差距/m | 评价次数 |', '|---|---:|---:|---:|']
    for s in local['samples']:
        distance = f"{s['distance_m']:.9f}" if s['distance_m'] is not None else 'null'
        gap = f"{s['gap_m']:.9f}" if s['gap_m'] is not None else 'null'
        lines.append(f"| {s['seed']} | {distance} | {gap} | {s['evaluations']} |")
    s = result['summary']
    lines += ['', '种子20261026固定为完整组主种子，未从5次结果中择优。独立参照在优化结束后计算；统计只描述这5次局部查询，不推断整体成功概率。', '',
        '## 一次主种子完整六艇执行', '',
        f"- 完成任务：{s['completed_task_count']}/{s['known_task_count']}。",
        f"- 六艇[0,120)实际总航程：{s['actual_total_distance_m']:.9f} m。",
        f"- 团队完成时间：{s['team_completion_time_s'] if s['team_completion_time_s'] is not None else 'null'} s。",
        f"- 规划保留决定：{checks['boat_hold_decisions']}次；未找到路线查询：{costs['failed_queries']}次（含重复查询）。", '',
        '未找到路线只表示固定搜索预算内未找到；保持可执行前缀，未规划尾部保持归属，艇不再接收新任务，普通事件到来才重试。规划保留与t24物理故障分开，未全完成则团队时间为null。', '',
        '## 计算口径与来源', '',
        f"查询{costs['queries']}次，独立优化{costs['unique_optimizations']}次，适应度评价{costs['fitness_evaluations']}次，缓存复用{costs['cache_hits']}次，优化计时{costs['search_ns']/1e6:.6f} ms。",
        '适应度评价次数与A*扩展或D*队列弹出不是同一种工作单位；计时包括初始化/PSO更新/解码评价/追踪，不含建图、独立参照、调度、序列化、IO或绘图。不据单次计时排名。', '',
        '[5种子局部统计](五种子局部统计.json) · [搜索费用](搜索费用.json) · [完整核查](统一核查.json) · [运行来源](运行元数据.json) · [固定参数](PSO策略.json)', '',
        '教师机械核查与本人运行/读数分别登记。效果图随后由保存日志生成，旧组不重跑；完整X2尚未结束。']
    return '\n'.join(lines) + '\n'


def run(run_id):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    assert_frozen_core()
    config, policy = load_config(), read_json(POLICY_PATH)
    assert policy['version'] == 'X2-PSO-V01' and policy['main_seed'] == policy['seeds'][0]
    folder = new_run_directory('X2', run_id)
    try:
        save(folder / '统一配置.json', config)
        save(folder / 'PSO策略.json', policy)
        local, traces = local_statistics(config, policy)
        save(folder / '五种子局部统计.json', local)
        save(folder / '五种子局部追踪.json', traces)
        print(f"PSO固定5种子局部查询：{local['successes']}/5合法；开始固定首种子完整组。", flush=True)
        planner = PSOSession(config, policy)
        result = simulate(config, config['x2']['initial_assignment'], 'DYNAMIC_REPLAN', planner, rebuild_routes=pso_rebuild)
        result['summary']['actual_total_distance_m'] = sum(b['distance_m'] for b in result['summary']['boat_totals'].values())
        result.update(framework=FRAMEWORK_VERSION, adjacency=8, input_version='X2-V01', algorithm=GROUP, pso_policy=policy)
        part = folder / GROUP
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
        checks = check_pso(read_json(part / '结果.json'), read_json(part / '搜索记录.json'), read_json(part / '图版本.json'), policy)
        save(part / '核查.json', checks)
        costs = {**{key: checks['search'][key] for key in ('queries', 'unique_optimizations', 'failed_queries', 'fitness_evaluations')},
            'search_ns': sum(r['search_ns'] for r in planner.rows), 'cache_hits': sum(r['cache_hit'] for r in planner.rows),
            'graph_build_ns': sum(g['build_ns'] for g in planner.graphs), 'fallback_search_calls': 0}
        save(folder / '搜索费用.json', {GROUP: costs})
        save(folder / '指标.json', {GROUP: result['summary']})
        save(folder / '统一核查.json', {'status': 'PASS', 'groups': {GROUP: checks},
            'learner_acceptance': 'PENDING_LEARNER_RUN_AND_READOUT', 'visual_status': 'PENDING'})
        save(folder / '运行元数据.json', {'framework': FRAMEWORK_VERSION, 'course': 'X2', 'topic': 'PSO_waypoint_search',
            'lesson': 'X2-PSO', 'algorithm': GROUP, 'input_version': 'X2-V01', 'config_sha256': digest(CONFIG_PATH),
            'pso_policy_sha256': digest(POLICY_PATH), 'main_seed': policy['main_seed'], 'local_seeds': policy['seeds'],
            'source_manifest_sha256': digest(VERSIONS_ROOT / FRAMEWORK_VERSION / '源码配置_SHA256.txt'),
            'command': sys.argv, 'python': sys.executable, 'output': str(folder), 'group_execution_count': 1,
            'status': 'COMPLETE', 'role': 'invocation ownership confirmed separately in complete-course record',
            'changed_factor': 'segment route planner and explicit safe-prefix policy for bounded heuristic failure', 'solver_runs': 0})
        (folder / '摘要.md').write_text(summary(result, local, checks, costs))
        write_manifest(part)
        write_manifest(folder)
        print(f"PSO完整保存核查PASS，完成{result['summary']['completed_task_count']}/22。", flush=True)
    except Exception as exc:
        save(folder / '失败.json', {'status': 'FAILED', 'error': str(exc), 'output': str(folder)})
        raise
    return folder
