"""Two 300dpi fleet figures from saved X2-L1 logs; never run a simulation."""
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
    name = 'A*' if result['algorithm'] == 'Astar' else 'Dijkstra（h=0）'
    fig.text(.055, .95, f'X2-L1  |  {name} · 完整六艇实际执行', fontsize=20, weight='bold', color='#21374c')
    fig.text(.055, .91, '八邻接 · 同初始归属与顺序 · 同动态策略 · 20初始＋t11新增2点', fontsize=11, color='#536579')
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
    fig.text(x, .51, 'U1→T03局部搜索 · 图v0', fontsize=12, weight='bold', color='#21374c')
    fig.text(x, .475, f"最短路 {local['distance_m']:.6f} m\n有效扩展 {local['expanded_nodes']}\n单次搜索 {local['search_ns']/1e6:.4f} ms", fontsize=11, color='#536579', linespacing=1.8, va='top')
    fig.text(x, .355, f"完整执行规划：{totals['queries']}次查询\n有效扩展合计 {totals['expanded_nodes']}\n搜索累计 {totals['search_ns']/1e6:.3f} ms", fontsize=10, color='#536579', linespacing=1.8, va='top')
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
    fig.text(.065, .095, '总航程含六艇[0,120)全部移动及完工后中继；搜索计时排除图构建/路径恢复/IO/绘图，单次不作普遍快慢结论。', fontsize=9, color='#536579')
    fig.text(.065, .060, f"{result['framework']}  |  输入 X2-V01  |  run-id: {run_id}", fontsize=9, color='#536579')
    fig.text(.065, .030, '来源：本次完整执行日志的121个状态快照；二维课堂代理 · 几何NEU · 距离阈值通信。', fontsize=9, color='#536579')
    fig.savefig(target, dpi=300, facecolor='white')
    plt.close(fig)


def render_pair(run, revision=None):
    from usv_uuc_cooperation.main import save, assert_frozen_core
    assert_frozen_core()
    run = Path(run).resolve()
    root = RESULTS_ROOT / 'X2/X2-V01'
    if not run.is_relative_to(root / 'runs'):
        raise ValueError('X2 renderer requires a saved X2-V01 run directory')
    verify_manifest(run)
    metadata, config = read_json(run / '运行元数据.json'), read_json(run / '统一配置.json')
    assert metadata['status'] == 'COMPLETE' and metadata['group_execution_count'] == 2
    assert metadata['config_sha256'] == digest(run / '统一配置.json')
    assert read_json(run / '统一核查.json')['status'] == 'PASS'
    output = root / 'showcase' / run.name
    if revision is not None:
        if not re.fullmatch(r'V[0-9]{2}', revision):
            raise ValueError('Render revision must be a version label such as V02')
        output = output / revision
    output.mkdir(parents=True, exist_ok=False)
    totals, local = read_json(run / '搜索费用.json'), read_json(run / '局部搜索对照.json')
    sources = []
    for name in ('Astar', 'Dijkstra'):
        source = run / name / '结果.json'
        result = read_json(source)
        assert result['config'] == config and result['algorithm'] == name
        assert len(result['snapshots']) == 121 and len(result['steps']) == 720
        assert set(result['snapshots'][-1]['boats']) == {'USV', 'U1', 'U2', 'U3', 'U4', 'U5'}
        assert len(result['tasks']) == 22
        total = math.fsum(s['actual_distance_m'] for s in result['steps'])
        assert math.isclose(total, result['summary']['actual_total_distance_m'], abs_tol=1e-8)
        target = output / f'{name}_效果图.png'
        draw(result, local[name], totals[name], run.name, target)
        with Image.open(target) as im:
            im.load()
            assert im.size == (4200, 2700) and min(im.info['dpi']) >= 299
        sources.append({'algorithm': name, 'source': str(source.relative_to(PROJECT_ROOT)), 'source_sha256': digest(source),
                        'png': target.name, 'png_sha256': digest(target), 'snapshots': 121, 'actual_step_rows': 720,
                        'boat_count': 6, 'task_count': 22, 'size': [4200, 2700], 'dpi': 300})
    save(output / '绘图来源与核查.json', {'source_status': 'PASS', 'visual_status': 'PENDING_TEACHER_INSPECTION',
        'run': str(run), 'command': ['python', '-m', 'usv_uuc_cooperation.main', 'render', '--stage', 'X2', '--run', str(run)],
        'config_sha256': digest(run / '统一配置.json'), 'run_manifest_sha256': digest(run / 'SHA256.txt'),
        'framework': metadata['framework'], 'render_framework': FRAMEWORK_VERSION,
        'render_source_sha256': digest(Path(__file__)), 'render_revision': revision,
        'input_version': 'X2-V01', 'simulation_reruns': 0, 'sources': sources,
        'common_style': 'fixed map limits, equal aspect, accepted six boat colors/task states and 300dpi',
        'state_rule': 'all actual snapshots/positions; terminal task service status; temporary map cell explicitly historical; flow arrows explicitly t11'})
    (output / '摘要.md').write_text('# X2-L1完整六艇效果\n\n'
        f"{metadata['framework']} · 输入X2-V01 · 真实run-id `{run.name}`。两组各121快照、720步记录、6艇、22任务。\n\n"
        '[A*效果图](Astar_效果图.png) · [Dijkstra效果图](Dijkstra_效果图.png)\n\n'
        f'[本人运行摘要]({run / "摘要.md"}) · [绘图来源](绘图来源与核查.json)\n\n'
        '图为保存完整日志回放，绘图未重跑模拟/求解器。实线为实际轨迹，点线为初始规划，终态任务区分服务状态；橙格标记t6禁行/t18撤销历史，洋流箭头标记t11。\n\n'
        '来源与图片规格检查PASS；视觉检查和本人验收状态见本课验收目录，不由此自动宣告通过。\n', encoding='utf-8')
    write_manifest(output)
    return output
