"""L3 fixed queues first, then real CP-SAT reassignment with each adjacency."""
import sys

from usv_uuc_cooperation.config import PACKAGE_ROOT, PROJECT_ROOT, FRAMEWORK_VERSION, new_run_directory, read_json
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.metrics.logger import digest
from usv_uuc_cooperation.scheduling.x1_assignment import build_problem, solve_assignment, nearest_order, route_budget


def load_config():
    path = PACKAGE_ROOT / 'configs/X1-L3_V01.json'
    config = read_json(path)
    source = PROJECT_ROOT / config['source']['path']
    if digest(source) != config['source']['sha256']:
        raise ValueError('P2-L4 frozen case source changed')
    original = read_json(source)
    if (config['starts_m'] != original['starts'] or config['tasks_m'] != original['points']
            or config['boats'] != original['boats'] or config['tasks'] != original['tasks']
            or config['capacity'] != [original['capacity']] * len(original['boats'])):
        raise ValueError('L3 endpoints/tasks/capacity differ from P2-L4')
    expected = {(x, y) for x in range(original['map']['x_range'][0], original['map']['x_range'][1] + 1)
                for y in range(original['map']['y_range'][0], original['map']['y_range'][1] + 1)
                if any(max(abs(x-ox), abs(y-oy)) <= 1 for ox, oy in original['map']['obstacles'])}
    if (forbidden_nodes(config) != expected or config['map']['x_range'] != original['map']['x_range']
            or config['map']['y_range'] != original['map']['y_range'] or config['map']['spacing_m'] != original['map']['spacing_m']):
        raise ValueError('L3 forbidden cells/map differ from original P2-L4')
    return path, config, source


def calculate(config, phase):
    if phase not in ('fixed', 'reassign'):
        raise ValueError('Choose fixed or reassign')
    groups = {}
    for mode in (4, 8):
        problem = build_problem(config, mode)
        solver = solve_assignment(problem) if phase == 'reassign' else None
        if solver is not None and solver['assignment'] is None:
            groups[str(mode)] = {'problem': problem, 'solver': solver, 'route_budget': None}
            continue
        queues = (config['fixed_queues'] if phase == 'fixed' else
                  {b: nearest_order(b, solver['assignment'][b], problem) for b in config['boats']})
        groups[str(mode)] = {'problem': problem, 'solver': solver, 'route_budget': route_budget(config, problem, queues)}
    return {'framework': FRAMEWORK_VERSION, 'phase': phase, 'config': config, 'groups': groups,
            'comparison': 'same queues; geometry effect only' if phase == 'fixed' else 'each adjacency solves same scaled start-task objective, then same nearest-next rule; ownership/order may change'}


def summary(result):
    phase, groups = result['phase'], result['groups']
    text = [f"# X1-L3 {'固定队列' if phase == 'fixed' else '重新分配'}对照", '',
            f"{result['framework']}；同P2-L4地图/起点/任务/容量，四／八邻接安全最短路。",
            '', '本页仅规划与预算预估，没有执行日志或动态融合。统计仅UUV，不返航。', '',
            '| 邻接 | 队列 | route_total_m 路线总长/m | 预测能耗 U1/U2 NEU | all_budget_ok |',
            '|---|---|---:|---|---|']
    for mode, group in groups.items():
        route = group['route_budget']
        if route is None:
            text.append(f"| {mode} | 无可用解：{group['solver']['status']} | null | null | null |")
            continue
        queues = '；'.join(f"{b}: {'→'.join(q) or '空'}" for b, q in route['queues'].items())
        energies = '/'.join(f"{r['predicted_energy_NEU']:.9f}" for r in route['boats'].values())
        text.append(f"| {mode} | {queues} | {route['route_total_m']:.9f} | {energies} | {route['all_budget_ok']} |")
    text += ['', '预算各40NEU，预测能耗按完整浮点路线×2核对；预算为分配后的检查，不是CP-SAT约束。', '',
             '## 起点到任务成本矩阵（m）', '', '| 邻接/艇 | A | B | C | D |', '|---|---:|---:|---:|---:|']
    for mode, group in groups.items():
        p = group['problem']
        for b, row in zip(p['boats'], p['cost_m']):
            text.append(f"| {mode}/{b} | " + ' | '.join('不可达' if c is None else f'{c:.9f}' for c in row) + ' |')
    text += ['', '成本矩阵、排序、每段路径和预算均使用该行邻接配置；完整路线包含任务间航段。', '',
             '## 缩放与目标', '', 'ROUND_HALF_UP，倍率1000；每项舍入误差≤0.0005m，四项之和≤0.002m。浮点路径本身的数值误差另按1e-9m核查。']
    if phase == 'reassign':
        text += ['', '| 邻接 | CP-SAT状态 | objective_int | scaled_objective_m | float_start_task_sum_m | 舍入误差/m |',
                 '|---|---|---:|---:|---:|---:|']
        for mode, group in groups.items():
            s = group['solver']
            if s['assignment'] is None:
                text.append(f"| {mode} | {s['status']} | null | null | null | null |")
            else:
                text.append(f"| {mode} | {s['status']} | {s['objective_int']} | {s['scaled_objective_m']:.9f} | {s['float_start_task_sum_m']:.9f} | {s['selected_sum_error_m']:+.9f} |")
        text += ['', 'OPTIMAL仅证明本次整数缩放分配模型最优；完整路线、队列排序、能耗与完成时间未由这个目标证明最优。排序为统一的最近下一任务规则，距离并列按任务ID。']
    else:
        text += ['', '本阶段不调用分配求解器，队列冻结为U1:A→B、U2:D→C；不能据此宣称重新分配效果。']
    text += ['', '[原始成本/路径/预算](对照结果.json) · [运行身份](运行元数据.json)', '',
             '教师机械检查与本人运行、读数和因果解释分别登记。']
    return '\n'.join(text) + '\n'


def run(run_id, phase):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    assert_frozen_core()
    path, config, source = load_config()
    # Check/reserve output before computing, so a duplicate run-id never reruns a solve.
    folder = new_run_directory('X1', run_id)
    try:
        result = calculate(config, phase)
        save(folder / '对照结果.json', result)
        (folder / '摘要.md').write_text(summary(result), encoding='utf-8')
        save(folder / '运行元数据.json', {'framework': FRAMEWORK_VERSION, 'lesson': 'X1-L3',
             'phase': phase, 'command': sys.argv, 'python': sys.executable, 'config_sha256': digest(path),
             'source_case_sha256': digest(source), 'output': str(folder), 'status': 'COMPLETE',
             'scope': 'static planning and budget prediction; no simulation or L4'})
        (folder / 'SHA256.txt').write_text(''.join(f'{digest(p)}  {p.name}\n' for p in sorted(folder.iterdir()) if p.is_file()))
    except Exception as exc:
        save(folder / '失败.json', {'error': str(exc), 'status': 'FAILED'})
        raise
    return folder
