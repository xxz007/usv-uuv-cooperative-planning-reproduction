"""Six accepted saved runs: common-clock animation and comparable outcome data."""
import argparse
import io
from pathlib import Path
import re

from usv_uuc_cooperation.config import PROJECT_ROOT, RESULTS_ROOT, FRAMEWORK_VERSION, read_json
from usv_uuc_cooperation.experiments.p4_integrated import verify_manifest
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest
from usv_uuc_cooperation.visualization.x1_integrated import COLORS, draw_panel, record
from matplotlib.lines import Line2D
from PIL import Image
import matplotlib.pyplot as plt

SOURCES = [
    ('Astar', 'A*', 'X2-L1_本人V01'), ('Dijkstra', 'Dijkstra', 'X2-L1_本人V01'),
    ('DstarLite', 'D* Lite', 'X2-L2_本人V01'), ('Dstar', '原始D*', 'X2_完整扩展_Dstar_本人V01'),
    ('GridScanAstar', '扫描＋A*', 'X2_完整扩展_Scan_本人V01'), ('PSO', 'PSO', 'X2_完整扩展_PSO_本人V01')]


def load_groups():
    groups, provenance, metrics = {}, [], []
    common, observed = None, None
    for algorithm, label, run_id in SOURCES:
        run = RESULTS_ROOT / 'X2/X2-V01/runs' / run_id
        verify_manifest(run)
        result_path = run / algorithm / '结果.json'
        result = read_json(result_path)
        config, metadata = read_json(run / '统一配置.json'), read_json(run / '运行元数据.json')
        assert result['config'] == config and result['algorithm'] == algorithm and metadata['status'] == 'COMPLETE'
        assert read_json(run / '统一核查.json')['status'] == 'PASS'
        assert metadata['config_sha256'] == digest(run / '统一配置.json')
        assert config == common if common is not None else True
        common = config
        snaps = result['snapshots']
        assert [s['time_s'] for s in snaps] == list(range(121)) and len(result['steps']) == 720
        assert len(result['tasks']) == 22 and set(snaps[-1]['boats']) == set(COLORS)
        schedule = [{k: s[k] for k in ('time_s', 'current_m_per_s', 'observed_blocked_cells', 'graph_version', 'event_ids')} for s in snaps]
        assert schedule == observed if observed is not None else True
        observed = schedule
        # Reconstruct all reported outcomes from saved physical evidence.
        distance = sum(s['actual_distance_m'] for s in result['steps'])
        summary = result['summary']
        assert abs(distance - summary['actual_total_distance_m']) < 1e-8
        complete = [t for t in result['tasks'].values() if t['status'] == 'completed']
        assert summary['completed_task_count'] == len(complete)
        team = max(t['completion_s'] for t in complete) if len(complete) == 22 else None
        assert summary['team_completion_time_s'] == team
        costs = read_json(run / '搜索费用.json')[algorithm]
        work = costs['fitness_evaluations'] if algorithm == 'PSO' else costs.get('logical_pops', costs.get('expanded_nodes'))
        unit = '适应度评价' if algorithm == 'PSO' else '有效扩展' if algorithm in ('Astar', 'Dijkstra', 'GridScanAstar') else '逻辑弹出'
        row = {'algorithm': algorithm, 'label': label, 'completed': len(complete), 'known': 22,
            'actual_total_distance_m': distance, 'team_completion_time_s': team, 'queries': costs['queries'],
            'work_value': work, 'work_unit': unit, 'search_ns': costs['search_ns'],
            'run_id': run_id, 'execution_framework': metadata['framework']}
        metrics.append(row)
        groups[algorithm] = result
        provenance.append({'algorithm': algorithm, 'result': str(result_path.relative_to(PROJECT_ROOT)),
            'result_sha256': digest(result_path), 'config_sha256': digest(run / '统一配置.json'),
            'run_manifest_sha256': digest(run / 'SHA256.txt'), 'execution_framework': metadata['framework'],
            'snapshots': 121, 'steps': 720})
    return groups, provenance, metrics


def data_figure(metrics, target):
    fig, axes = plt.subplots(1, 3, figsize=(15, 8.5), facecolor='white')
    fig.subplots_adjust(left=.10, right=.97, top=.79, bottom=.35, wspace=.53)
    fig.text(.065, .94, 'X2完整扩展课程  |  六组真实执行结果', fontsize=22, weight='bold', color='#21374c')
    fig.text(.065, .89, '共同X2-V01 · 八邻接 · 1USV＋5UUV · 22任务 · 相同事件与120秒观察窗', fontsize=12, color='#536579')
    labels = [m['label'] for m in metrics]
    for ax, field, title, limit in zip(axes,
            ['completed', 'actual_total_distance_m', 'team_completion_time_s'],
            ['完成任务数 / 个', '六艇全窗实际总航程 / m', '团队完成时间 / s'], [24, 225, 100]):
        values = [m[field] if m[field] is not None else 0 for m in metrics]
        bars = ax.barh(labels, values, color=['#6985a1'] * 4 + ['#be873b', '#3a9187'], height=.55)
        ax.invert_yaxis(); ax.set_xlim(0, limit); ax.set_title(title, fontsize=12, pad=15)
        ax.grid(axis='x', alpha=.14); ax.set_axisbelow(True)
        for bar, m in zip(bars, metrics):
            value = m[field]
            text = 'null' if value is None else f'{value:.3f}' if field == 'actual_total_distance_m' else str(value)
            ax.text(bar.get_width() + limit * .025, bar.get_y() + bar.get_height()/2, text, va='center', fontsize=10)
        for spine in ('right', 'top'): ax.spines[spine].set_visible(False)
    cells = [[m['label'], str(m['queries']), f"{m['work_value']} {m['work_unit']}", f"{m['search_ns']/1e6:.3f}", m['execution_framework']] for m in metrics]
    table_ax = fig.add_axes([.065, .105, .90, .20]); table_ax.axis('off')
    table = table_ax.table(cellText=cells, colLabels=['规划费用辅助信息', '完整规划查询', '工作量（单位分列）', '记录内搜索/优化 ms', '执行源码版本'], loc='center', cellLoc='left', colLoc='left', colWidths=[.18, .14, .28, .22, .18])
    table.auto_set_font_size(False); table.set_fontsize(9); table.scale(1, 1.45)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#dbe4eb'); cell.set_facecolor('#edf3f7' if row == 0 else 'white')
    fig.text(.065, .065, '航程含故障前及完工后中继；未全完成时团队时间为null。扫描改变访问顺序，PSO显式保留有限搜索失败，不放宽物理约束。', fontsize=9, color='#536579')
    fig.text(.065, .035, '有效扩展、逻辑弹出、适应度评价不可直接互比；各组计时边界有差异，不作普遍速度排名。不同等长路线也可能有不同离散完成秒。', fontsize=9, color='#536579')
    fig.savefig(target, dpi=300, facecolor='white'); plt.close(fig)


def animation(groups, output):
    images, ledger = [], []
    fig = plt.figure(figsize=(18, 11.5), dpi=110, facecolor='white')
    for index in range(121):
        fig.clear()
        frame = {'time_s': index, 'groups': {}}
        fig.text(.05, .968, f'X2六组同帧执行  |  t = {index:03d} s', fontsize=21, weight='bold', color='#21374c')
        snap = groups['Astar']['snapshots'][index]
        events = [e for e in groups['Astar']['event_log'] if e['time_s'] <= index]
        event_names = {'MAP_UPDATE': '地图更新', 'TASK_ARRIVAL': '新任务', 'UUV_FAILURE': 'U4故障'}
        history = '；'.join(f"t{e['time_s']} {event_names.get(e['type'], e['type'])}" for e in events) or '初始任务20点；t11两点尚未发布'
        fig.text(.05, .937, f"当前流 ({snap['current_m_per_s'][0]:+.2f}, {snap['current_m_per_s'][1]:+.2f}) m/s  |  已观测：{history}", fontsize=11, color='#536579')
        for number, (algorithm, label, _) in enumerate(SOURCES, 1):
            result = groups[algorithm]; state = result['snapshots'][index]
            ax = fig.add_subplot(2, 3, number)
            draw_panel(ax, result, index, set(map(tuple, state['observed_blocked_cells'])))
            if 6 <= index < 18:
                ax.annotate('临时禁行', (12, 3), xytext=(8, -12), textcoords='offset points', fontsize=7, color='#a96517')
            row = record(result, index)
            frame['groups'][algorithm] = row
            ax.set_title(f"{label}  |  {row['completed']}/{row['known_tasks']}  |  {row['actual_total_distance_m']:.1f} m", fontsize=12, pad=9)
            ax.tick_params(labelsize=8)
        fig.subplots_adjust(left=.045, right=.97, bottom=.10, top=.875, hspace=.30, wspace=.20)
        handles = [Line2D([], [], color=color, label=boat, lw=2) for boat, color in COLORS.items()]
        handles += [Line2D([], [], marker='o', color='none', markerfacecolor=color, markeredgecolor='#667788', label=label) for color, label in [('#ffffff', '待执行/到达'), ('#ecc052', '服务中'), ('#2b9864', '完成'), ('#d84054', '失去归属')]]
        fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .042), ncol=10, fontsize=9, frameon=False, handlelength=1.5)
        fig.text(.05, .022, '实线=实际轨迹；虚线=剩余规划；点线/绿线=旧规划/通信；X=故障；星形任务=t11发布后出现。每1s日志播放200ms，同尺度/时刻，未按进度拉伸。', fontsize=10, color='#536579')
        buffer = io.BytesIO(); fig.savefig(buffer, format='png', dpi=110, facecolor='white'); buffer.seek(0)
        with Image.open(buffer) as image: images.append(image.convert('RGB').quantize(colors=192))
        ledger.append(frame)
        if index % 30 == 0: print(f'同帧日志绘图 {index}/120', flush=True)
    plt.close(fig)
    images[0].save(output / '任务执行_同帧对比.gif', save_all=True, append_images=images[1:], duration=200, loop=0, disposal=2, optimize=False)
    for index in (0, 6, 11, 18, 24, 55, 85, 120): images[index].save(output / f'同帧_t{index:03d}.png')
    return ledger


def render(name):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    assert_frozen_core()
    if not re.fullmatch(r'[\w\-]+', name): raise ValueError('Use one unique showcase name')
    groups, provenance, metrics = load_groups()
    output = RESULTS_ROOT / 'X2/X2-V01/showcase' / name
    output.mkdir(parents=True, exist_ok=False)
    data_figure(metrics, output / '算法效果_数据对比.png')
    ledger = animation(groups, output)
    save(output / '逐帧数据.json', ledger)
    save(output / '对比指标.json', metrics)
    with Image.open(output / '任务执行_同帧对比.gif') as im:
        assert im.n_frames == 121 and im.size == (1980, 1265)
        durations = []
        for index in range(im.n_frames): im.seek(index); durations.append(im.info['duration'])
        assert durations == [200] * 121
    with Image.open(output / '算法效果_数据对比.png') as im:
        assert im.size == (4500, 2550) and min(im.info['dpi']) >= 299
    save(output / '来源与规格核查.json', {'source_status': 'PASS', 'visual_status': 'PENDING_TEACHER_INSPECTION',
        'render_framework': FRAMEWORK_VERSION, 'render_source_sha256': digest(Path(__file__)), 'sources': provenance,
        'same_physical_config_and_observed_schedule': 'PASS', 'common_times': list(range(121)),
        'gif_frames': 121, 'gif_size': [1980, 1265], 'duration_ms_each': 200,
        'data_png_size': [4500, 2550], 'data_png_dpi': 300, 'simulation_reruns': 0, 'solver_runs': 0,
        'outcomes_independently_reconstructed_from_saved_steps_and_tasks': 'PASS',
        'command': ['python', '-B', '-m', 'usv_uuc_cooperation.visualization.x2_final', '--showcase', name]})
    lines = ['# X2完整扩展课程｜六组同帧与数据对比', '',
        f'绘图{FRAMEWORK_VERSION}；共同输入X2-V01；只读已通过本人运行日志，未重跑模拟/求解器。', '',
        '[六组数据对比](算法效果_数据对比.png) · [六组同帧GIF](任务执行_同帧对比.gif) · [逐帧数据](逐帧数据.json) · [来源核查](来源与规格核查.json)', '',
        '| 组 | 完成 | 六艇全窗实际航程/m | 团队完成秒 |', '|---|---:|---:|---:|']
    lines += [f"| {m['label']} | {m['completed']}/22 | {m['actual_total_distance_m']:.9f} | {m['team_completion_time_s'] if m['team_completion_time_s'] is not None else 'null'} |" for m in metrics]
    lines += ['', 't0～t120每秒一帧、每帧200ms，所有面板使用同一地图范围/真实时刻/颜色与播放速度。每组使用自己的实际状态，事件按观测到的时刻显示；未发布任务不提前出现。', '',
        '扫描改变任务访问顺序；其单段仍用A*。PSO固定预算/主种子，失败保留单列。本次结果不构成所有场景的最优性或速度证明；5种子局部0差也不意味着完整运行所有查询0差。', '',
        '主指标为完成数、全窗航程和团队完成时间。搜索辅助工作量的单位/计时范围不同，不直接排名。来源/规格PASS，视觉检查及本人综合解释另行登记，完整课程尚未最终验收。']
    (output / '摘要.md').write_text('\n'.join(lines) + '\n')
    write_manifest(output)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--showcase', required=True)
    args = parser.parse_args()
    print(render(args.showcase) / '摘要.md')
