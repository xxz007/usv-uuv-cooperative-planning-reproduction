"""One D* or grid-scan group figure from saved logs; never rerun simulation."""
from pathlib import Path
import math
import re

from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from PIL import Image

from usv_uuc_cooperation.config import RESULTS_ROOT, PROJECT_ROOT, FRAMEWORK_VERSION, read_json
from usv_uuc_cooperation.experiments.p4_integrated import verify_manifest
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest
from usv_uuc_cooperation.visualization.x1_integrated import COLORS, background, tasks_draw
import matplotlib.pyplot as plt


def draw(result, local, totals, run_id, target):
    c, snapshots = result['config'], result['snapshots']
    s, final = result['summary'], snapshots[-1]
    fig = plt.figure(figsize=(14, 9), facecolor='white')
    original = result['algorithm'] == 'Dstar'
    scan = result['algorithm'] == 'GridScanAstar'
    pso = result['algorithm'] == 'PSO'
    name = 'PSO · 中间航点与可行历史记忆' if pso else '网格扫描任务遍历＋A*连接' if scan else '原始D* · h/p与回指针修复' if original else 'D* Lite · g/rhs增量修复'
    prefix = 'X2完整扩展' if original or scan or pso else 'X2-L2'
    fig.text(.055, .95, f'{prefix}  |  {name} · 完整六艇实际执行', fontsize=20, weight='bold', color='#21374c')
    subtitle = '八邻接 · 同初始归属 · 蛇形访问顺序 · A*连接 · 20初始＋t11新增2点' if scan else '八邻接 · 同初始归属与顺序 · 同动态策略 · 20初始＋t11新增2点'
    fig.text(.055, .91, subtitle, fontsize=11, color='#536579')
    ax = fig.add_axes([.065, .20, .60, .65])
    from usv_uuc_cooperation.env.obstacle import forbidden_nodes
    flow_snap = snapshots[min(11, len(snapshots) - 1)]
    background(ax, c, forbidden_nodes(c), flow_snap['current_m_per_s'])
    for event in c['events']:
        if event['type'] == 'MAP_UPDATE':
            for x, y in event['add_cells']:
                ax.add_patch(Rectangle((x - .5, y - .5), 1, 1, facecolor='#e8a135', alpha=.7, hatch='///', lw=.8))
                ax.annotate('t6禁行／t18撤销', (x, y), xytext=(0, -27), textcoords='offset points',
                            ha='center', fontsize=8, color='#996012', arrowprops={'arrowstyle': '-', 'color': '#996012'})
    for boat, state in final['boats'].items():
        # Initial plan is visibly separated from the complete actual trajectory.
        initial = [snapshots[0]['boats'][boat]['position_m']] + [op['end'] for op in snapshots[0]['remaining_plans'].get(boat, []) if op['action'] == 'move']
        if len(initial) > 1:
            ax.plot(*zip(*initial), ls=':', lw=1, alpha=.38, color=COLORS[boat], zorder=2)
        trajectory = [snap['boats'][boat]['position_m'] for snap in snapshots]
        ax.plot(*zip(*trajectory), color=COLORS[boat], lw=1.8, alpha=.85, zorder=4)
        ax.scatter(*trajectory[0], s=34, marker='o', facecolor='white', edgecolor=COLORS[boat], zorder=5)
        ax.scatter(*state['position_m'], marker='X' if state['failure'] else '*' if boat == 'USV' else 's',
                   s=120 if boat == 'USV' else 60, color=COLORS[boat], edgecolor='white', lw=.8, zorder=9)
        ax.annotate(boat, state['position_m'], xytext=(5, 9), textcoords='offset points', fontsize=9, color=COLORS[boat], zorder=10)
        if state['failure']:
            ax.annotate('t24故障', state['position_m'], xytext=(5, -22), textcoords='offset points', fontsize=8, color=COLORS[boat])
    tasks_draw(ax, final['tasks'])
    ax.set_title(f"实际轨迹 t0–{c['cutoff_time_s']}s · 任务状态取t{c['cutoff_time_s']}", fontsize=12, pad=10)
    x = .72
    fig.text(x, .80, '完整执行结果', fontsize=14, weight='bold', color='#21374c')
    team = f"{s['team_completion_time_s']} s" if s['team_completion_time_s'] is not None else 'null（未完成）'
    for y, label, value in [(.75, '完成任务', f"{s['completed_task_count']} / {s['known_task_count']}"),
                            (.68, '六艇实际总航程', f"{s['actual_total_distance_m']:.3f} m"), (.61, '团队完成时间', team)]:
        fig.text(x, y, label, fontsize=10, color='#536579')
        fig.text(x, y - .032, value, fontsize=17, weight='bold', color='#21374c')
    fig.text(x, .51, '固定种子 / 有限预算' if pso else '扫描排序 / A*连接' if scan else '共同观测 / 同目标修复', fontsize=12, weight='bold', color='#21374c')
    phases = totals['map_repairs']
    detail = ('种子20261026 · 8个中间航点\n40粒子 · 60轮 · ≤2440评价/查询' if pso else '按y行升序，逐行交替x方向\n已到达/服务任务保持队首' if scan else
              '\n'.join(f"t{r['time_s']} 图v{r['graph_version']}：{r['queries']}次修复 / {r['logical_pops']}次弹出" for r in phases))
    hold_count = sum(len(d.get('planning_holds', {})) for d in result['replans'])
    mechanism = f"无A*修复；失败{totals['failed_queries']}次/保留{hold_count}次" if pso else '只排序已发布任务；单段仍用A*' if scan else 'h/p、回指针与OPEN保留' if original else 'g/rhs与队列保留，按当前边更新'
    fig.text(x, .475, detail + '\n' + mechanism, fontsize=10, color='#536579', linespacing=1.8, va='top')
    work_label, time_label = ('适应度评价', 'PSO优化') if pso else ('有效扩展', 'A*搜索') if scan else ('逻辑弹出', '更新/修复')
    work_count = totals['fitness_evaluations'] if pso else totals['logical_pops']
    fig.text(x, .355, f"完整规划 {totals['queries']}次查询\n{work_label} {work_count}次\n{time_label} {totals['search_ns']/1e6:.3f}ms", fontsize=10, color='#536579', linespacing=1.8, va='top')
    handles = [Line2D([0], [0], color=COLORS[b], lw=2, label=b) for b in COLORS]
    handles += [Line2D([0], [0], color='#667788', lw=1.8, label='实际轨迹'),
                Line2D([0], [0], color='#667788', lw=1, ls=':', label='初始规划'),
                Line2D([0], [0], marker='o', color='none', markerfacecolor='#2b9864', markeredgecolor='#2b9864', label='服务完成'),
                Line2D([0], [0], marker='*', color='none', markerfacecolor='#2b9864', markeredgecolor='#2b9864', label='t11新增任务'),
                Line2D([0], [0], marker='o', color='none', markerfacecolor='white', markeredgecolor='#667788', label='待执行／未完成'),
                Line2D([0], [0], marker='X', color='none', markerfacecolor=COLORS['U4'], markeredgecolor=COLORS['U4'], label='故障终态')]
    fig.legend(handles=handles, loc='lower left', bbox_to_anchor=(.715, .13), ncol=2, fontsize=9,
               frameon=False, handlelength=1.7, columnspacing=1.1)
    flow = flow_snap['current_m_per_s']
    fig.text(.065, .13, f'蓝箭头：t{flow_snap["time_s"]}洋流 ({flow[0]:+.2f}, {flow[1]:+.2f})m/s，方向示意；橙格为临时禁行历史。', fontsize=9, color='#536579')
    note = ('总航程含六艇[0,120)全部移动及完工后中继；pbest/gbest是已找到的最好，不代表全局最优；不隐藏失败或回退。' if pso else '总航程含六艇[0,120)全部移动及完工后中继；改变任务顺序，A*仍为连接器；不等于区域全格覆盖或全局最优顺序。' if scan else
            '总航程含六艇[0,120)全部移动及完工后中继；D*逻辑弹出含重入；初始化/图差分单列，独立A*仅作核查，不作普遍快慢结论。')
    fig.text(.065, .095, note, fontsize=9, color='#536579')
    fig.text(.065, .060, f"{result['framework']}  |  输入 X2-V01  |  run-id: {run_id}", fontsize=9, color='#536579')
    fig.text(.065, .030, '来源：本次完整执行日志的121个状态快照；二维课堂代理 · 几何NEU · 距离阈值通信。', fontsize=9, color='#536579')
    fig.savefig(target, dpi=300, facecolor='white')
    plt.close(fig)



def render(run, revision=None):
    from usv_uuc_cooperation.main import save, assert_frozen_core
    assert_frozen_core()
    run = Path(run).resolve()
    root = RESULTS_ROOT / 'x2_algorithms'
    if not run.is_relative_to(root / 'runs'):
        raise ValueError('D* Lite renderer requires a saved X2-V01 run')
    verify_manifest(run)
    metadata, config = read_json(run / '运行元数据.json'), read_json(run / '统一配置.json')
    groups = {'X2-L2': 'DstarLite', 'X2-Dstar': 'Dstar', 'X2-Scan': 'GridScanAstar', 'X2-PSO': 'PSO'}
    assert metadata['lesson'] in groups and metadata['status'] == 'COMPLETE'
    algorithm = groups[metadata['lesson']]
    if algorithm == 'GridScanAstar':
        assert metadata['ordering_policy_sha256'] == digest(run / '排序策略.json')
    if algorithm == 'PSO':
        from usv_uuc_cooperation.config import PACKAGE_ROOT
        assert read_json(run / 'PSO策略.json') == read_json(PACKAGE_ROOT / 'configs/X2-PSO-V01.json')
        assert metadata['pso_policy_sha256'] == digest(PACKAGE_ROOT / 'configs/X2-PSO-V01.json')
    assert metadata['group_execution_count'] == 1
    assert metadata['config_sha256'] == digest(run / '统一配置.json')
    assert read_json(run / '统一核查.json')['status'] == 'PASS'
    source = run / algorithm / '结果.json'
    result = read_json(source)
    assert result['config'] == config and result['algorithm'] == algorithm
    assert len(result['snapshots']) == 121 and len(result['steps']) == 720
    assert set(result['snapshots'][-1]['boats']) == {'USV', 'U1', 'U2', 'U3', 'U4', 'U5'}
    assert len(result['tasks']) == 22
    assert math.isclose(math.fsum(s['actual_distance_m'] for s in result['steps']), result['summary']['actual_total_distance_m'], abs_tol=1e-8)
    output = root / 'showcase' / run.name
    if revision is not None:
        if not re.fullmatch(r'V[0-9]{2}', revision):
            raise ValueError('Use a new revision label such as V02')
        output = output / revision
    output.mkdir(parents=True, exist_ok=False)
    costs = read_json(run / '搜索费用.json')[algorithm]
    searches = read_json(run / algorithm / '搜索记录.json')
    costs['map_repairs'] = [] if algorithm == 'PSO' else [
        {'time_s': t, 'graph_version': v,
         'queries': sum(r['time_s'] == t and (algorithm == 'GridScanAstar' or r.get('mode') == 'SAME_GOAL_REPAIR') for r in searches),
         'logical_pops': sum(r['expanded_nodes'] for r in searches if r['time_s'] == t and (algorithm == 'GridScanAstar' or r.get('mode') == 'SAME_GOAL_REPAIR'))}
        for t, v in ((6, 1), (18, 2))]
    target = output / ('网格扫描任务遍历_效果图.png' if algorithm == 'GridScanAstar' else f'{algorithm}_效果图.png')
    local = [] if algorithm in ('GridScanAstar', 'PSO') else read_json(run / '局部增量状态.json')
    draw(result, local, costs, run.name, target)
    with Image.open(target) as im:
        im.load()
        assert im.size == (4200, 2700) and min(im.info['dpi']) >= 299
    save(output / '绘图来源与核查.json', {
        'source_status': 'PASS', 'visual_status': 'PENDING_TEACHER_INSPECTION',
        'run': str(run), 'framework': metadata['framework'], 'render_framework': FRAMEWORK_VERSION,
        'input_version': 'X2-V01', 'simulation_reruns': 0,
        'config_sha256': digest(run / '统一配置.json'), 'run_manifest_sha256': digest(run / 'SHA256.txt'),
        'render_source_sha256': digest(Path(__file__)), 'render_revision': revision,
        'sources': [{'algorithm': algorithm, 'source': str(source.relative_to(PROJECT_ROOT)),
                     'source_sha256': digest(source), 'search_source_sha256': digest(run / algorithm / '搜索记录.json'),
                     'png': target.name, 'png_sha256': digest(target),
                     'snapshots': 121, 'actual_step_rows': 720, 'boat_count': 6, 'task_count': 22,
                     'size': [4200, 2700], 'dpi': 300}], 'map_repairs': costs['map_repairs'],
        'common_style': 'same accepted fixed map bounds, equal aspect, boat/task colors, 300dpi',
        'state_rule': 'actual six-boat trajectories/terminal task states; orange cell is observation history; flow arrows t11',
        'command': ['python', '-m', 'usv_uuc_cooperation.main', 'render', '--stage', 'X2', '--run', str(run)]})
    title = 'X2完整扩展课程｜PSO' if algorithm == 'PSO' else 'X2完整扩展课程｜网格扫描任务遍历＋A*连接' if algorithm == 'GridScanAstar' else 'X2完整扩展课程｜原始D*' if algorithm == 'Dstar' else 'X2-L2完整六艇D* Lite'
    (output / '摘要.md').write_text(f'# {title}效果\n\n'
        f"{metadata['framework']} · 输入X2-V01 · run-id `{run.name}`。121快照、720步、6艇、22任务。\n\n"
        f'[{algorithm}效果图]({target.name}) · [绘图来源](绘图来源与核查.json)\n\n'
        f'[本人运行摘要]({run / "摘要.md"})\n\n'
        '实线为实际轨迹，点线为初始规划；右侧说明该组真实机制与工作量，橙格标共同禁行/撤销历史。'
        f'仅生成{algorithm}独立图；没有重跑模拟或生成A*图。来源/规格PASS，视觉检查见课程验收。\n')
    write_manifest(output)
    return output
