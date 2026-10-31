"""Same-scale, same-time 4/8 panels and actual trajectory overlay from saved logs."""
import io
import os
from pathlib import Path
import tempfile
os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'usv-uuv-matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle
from PIL import Image

from usv_uuc_cooperation.config import PROJECT_ROOT, RESULTS_ROOT, read_json
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.experiments.p4_integrated import verify_manifest
from usv_uuc_cooperation.experiments.x1_integrated import write_manifest
from usv_uuc_cooperation.metrics.logger import digest

plt.rcParams.update({'font.family': 'PingFang SC', 'axes.unicode_minus': False})
COLORS = dict(zip(['U1', 'U2', 'U3', 'U4', 'U5', 'USV'],
                  ['#b86b12', '#8b4faf', '#128783', '#bf4055', '#6c7635', '#d32121']))


def background(ax, config, blocked, current):
    grid = config['map']
    ax.set(xlim=(grid['x_range'][0] - .5, grid['x_range'][1] + .5),
           ylim=(grid['y_range'][0] - .5, grid['y_range'][1] + .5), xlabel='x / m', ylabel='y / m')
    ax.set_aspect('equal')
    ax.set_facecolor('#f1f7fa')
    ax.set_xticks(range(grid['x_range'][0], grid['x_range'][1] + 1, 2))
    ax.set_yticks(range(grid['y_range'][0], grid['y_range'][1] + 1, 2))
    ax.grid(alpha=.16)
    for x, y in sorted(blocked):
        ax.add_patch(Rectangle((x - .5, y - .5), 1, 1, facecolor='#d6b1a6', alpha=.6, lw=0))
    for c in grid['circles']:
        ax.add_patch(Circle(c['center_m'], c['radius_m'], facecolor='#a36c60', alpha=.9))
        ax.add_patch(Circle(c['center_m'], c['radius_m'] + grid['safety_buffer_m'], fill=False, ls='--', color='#965c4b', lw=.8))
    if any(current):
        points = [(x, y) for x in range(1, grid['x_range'][1], 4) for y in range(1, grid['y_range'][1], 4)]
        ax.quiver([p[0] for p in points], [p[1] for p in points], [current[0]] * len(points), [current[1]] * len(points),
                  angles='xy', scale_units='xy', scale=.1, color='#2464ac', alpha=.55, width=.004)


def tasks_draw(ax, tasks):
    for task, item in tasks.items():
        color = {'completed': '#2b9864', 'servicing': '#ecc052', 'orphaned': '#d84054'}.get(item['status'], '#ffffff')
        ax.scatter(*item['position_m'], s=78 if item['release_s'] else 35, marker='*' if item['release_s'] else 'o',
                   color=color, edgecolor=COLORS.get(item['boat_id'], '#d84054'), lw=1, zorder=6)
        ax.annotate(task, item['position_m'], xytext=(3, -8), textcoords='offset points', fontsize=6.5,
                    bbox=dict(facecolor='white', edgecolor='none', alpha=.6, pad=.1), zorder=8)


def record(result, index):
    snap = result['snapshots'][index]
    t = snap['time_s']
    team = result['summary']['team_completion_time_s']
    return {'time_s': t, 'current_m_per_s': snap['current_m_per_s'],
            'completed': sum(x['status'] == 'completed' for x in snap['tasks'].values()),
            'known_tasks': len(snap['tasks']),
            'actual_total_distance_m': sum(b['distance_m'] for b in snap['boats'].values()),
            'team_completion_time_s': team if team is not None and t >= team else None,
            'positions_m': {b: s['position_m'] for b, s in snap['boats'].items()},
            'failed_boats': sorted(b for b, s in snap['boats'].items() if s['failure']),
            'replanning_count': snap['replanning_count'],
            'task_states': {k: {f: v[f] for f in ('status', 'boat_id', 'service_progress_s', 'completion_s')}
                            for k, v in snap['tasks'].items()},
            'observed_events': [e['event_id'] for e in result['event_log'] if e['time_s'] <= t]}


def draw_panel(ax, result, index, blocked):
    snap = result['snapshots'][index]
    background(ax, result['config'], blocked, snap['current_m_per_s'])
    t = snap['time_s']
    old = next((s for s in reversed(result['snapshots'][:index + 1]) if s.get('old_plans') and t - s['time_s'] <= 3), None)
    for b, state in snap['boats'].items():
        trail = [s['boats'][b]['position_m'] for s in result['snapshots'][:index + 1]]
        ax.plot(*zip(*trail), color=COLORS[b], lw=1.5, alpha=.85, zorder=4)
        remaining = [state['position_m']] + [p['end'] for p in snap['remaining_plans'].get(b, []) if p['action'] == 'move']
        if len(remaining) > 1:
            ax.plot(*zip(*remaining), ls='--', color=COLORS[b], lw=.85, alpha=.55, zorder=3)
        if old and b != 'USV':
            previous = [old['boats'][b]['position_m']] + [p['end'] for p in old['old_plans'].get(b, []) if p['action'] == 'move']
            if len(previous) > 1:
                ax.plot(*zip(*previous), ls=':', color='#777777', lw=1, alpha=.7, zorder=2)
        ax.scatter(*state['position_m'], marker='X' if state['failure'] else '*' if b == 'USV' else 's',
                   s=105 if b == 'USV' else 52, color=COLORS[b], edgecolor='white', lw=.7, zorder=9)
        ax.annotate(b, state['position_m'], xytext=(-19, 7) if b == 'U4' else (5, 7), textcoords='offset points',
                    fontsize=8, color=COLORS[b], zorder=10)
    usv = snap['boats']['USV']['position_m']
    ax.add_patch(Circle(usv, result['config']['communication']['radius_m'], fill=False, ls=':', color='#419373', alpha=.5))
    for b, connected in snap['connected'].items():
        if connected:
            p = snap['boats'][b]['position_m']
            ax.plot([usv[0], p[0]], [usv[1], p[1]], color='#419373', alpha=.3, lw=.7, zorder=2)
    tasks_draw(ax, snap['tasks'])


def draw_frame(fig, groups, index, blocked):
    fig.clear()
    records = {mode: record(result, index) for mode, result in groups.items()}
    t, current = records['4']['time_s'], records['4']['current_m_per_s']
    fig.text(.05, .952, 'X1  |  同一P4案例 · 四／八邻接动态执行', fontsize=19, weight='bold', color='#21374c')
    fig.text(.05, .918, f't = {t:03d} s    水体限速 1m/s · dt=1s · 同DYNAMIC_REPLAN · 10倍速', fontsize=11, color='#536579')
    fig.text(.67, .918, f'洋流 ({current[0]:+.2f}, {current[1]:+.2f}) m/s  |  固定箭头尺度', fontsize=10, color='#2464ac')
    for mode, x in [('4', .05), ('8', .53)]:
        d, result = records[mode], groups[mode]
        ax = fig.add_axes([x, .255, .42, .62])
        draw_panel(ax, result, index, blocked)
        ax.set_title(f'{mode}邻接 · 各自初始分配与队列', fontsize=13, pad=8, weight='bold')
        team = str(d['team_completion_time_s']) + ' s' if d['team_completion_time_s'] is not None else 'null'
        fig.text(x, .187, f"完成 {d['completed']}/{d['known_tasks']}     实际总航程 {d['actual_total_distance_m']:.3f} m     团队时间 {team}", fontsize=11, weight='bold', color='#21374c')
        fig.text(x, .151, f"重规划 {d['replanning_count']}次  |  故障艇：{'、'.join(d['failed_boats']) or '无'}", fontsize=10, color='#a93e50' if d['failed_boats'] else '#536579')
    phase = '尚未观测动态事件' if t < 11 else 't11：T21/T22已到达，依据当前状态重规划' if t < 24 else 't11到达已观测；t24 U4故障已观测，遗留任务按同规则接手'
    fig.text(.05, .113, phase, fontsize=11, color='#b04722')
    fig.text(.05, .077, '彩色实线：实际轨迹   彩色虚线：剩余计划   灰点线：事件前计划（3s）   绿：完成   黄：服务   星形任务：动态到达   X：故障', fontsize=9, color='#536579')
    fig.text(.05, .039, '总航程含六艇[0,120)全窗位移及完工后中继；同尺度/同帧时刻。二维课堂代理 · 距离阈值通信 · E=cL几何NEU', fontsize=9, color='#536579')
    return records


def draw_overlay(groups, path, blocked):
    fig = plt.figure(figsize=(14, 9), facecolor='white')
    ax = fig.add_axes([.07, .11, .61, .78])
    config = groups['4']['config']
    background(ax, config, blocked, [0, 0])
    for mode, result in groups.items():
        for b in result['snapshots'][0]['boats']:
            trail = [s['boats'][b]['position_m'] for s in result['snapshots']]
            ax.plot(*zip(*trail), color=COLORS[b], ls='-' if mode == '4' else '--', lw=1.5, alpha=.8)
            fault = next((e for e in result['event_log'] if e['type'] == 'UUV_FAILURE' and e['boat_id'] == b), None)
            if fault:
                ax.scatter(*fault['position_m'], marker='X', color=COLORS[b], s=85)
                ax.annotate(f'故障{mode}', fault['position_m'], xytext=(5, 10 if mode == '4' else -14), textcoords='offset points', fontsize=9)
    tasks_draw(ax, groups['4']['snapshots'][-1]['tasks'])
    handles = [Line2D([0], [0], color='black', ls=style, label=label) for style, label in [('-', '四邻接实际轨迹'), ('--', '八邻接实际轨迹')]]
    handles += [Line2D([0], [0], color=c, label=b) for b, c in COLORS.items()]
    ax.legend(handles=handles, loc='upper right', fontsize=8, ncol=2)
    fig.text(.07, .949, 'X1-L4  |  四／八邻接实际路径叠图', fontsize=19, weight='bold', color='#21374c')
    fig.text(.715, .86, '同输入 · 同动态响应规则', fontsize=13, weight='bold')
    fig.text(.715, .80, '实线四邻接 / 虚线八邻接\n颜色对应艇；X为各组故障位置\n星形T21/T22为t11动态到达', fontsize=10, linespacing=1.7, va='top')
    y = .65
    for mode, result in groups.items():
        s = result['summary']; team = s['team_completion_time_s']
        fig.text(.715, y, f"{mode}邻接：{s['completed_task_count']}/{s['known_task_count']}\n实际总航程 {s['actual_total_distance_m']:.3f} m\n团队完成时间 {team if team is not None else 'null'} s", fontsize=12, linespacing=1.8, va='top')
        y -= .2
    fig.text(.715, .21, '统计六艇[0,120)全窗实际位移\n含故障前移动及完工后中继\n归属/顺序/接手允许不同\n差值解释为系统整体效果', fontsize=10, linespacing=1.7, va='top')
    fig.savefig(path, dpi=140)
    plt.close(fig)


def render_pair(run_folder):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    from usv_uuc_cooperation.metrics.x1_integrated_check import check_result, check_pair
    assert_frozen_core()
    run_folder = Path(run_folder).resolve()
    expected = RESULTS_ROOT / 'x1_adjacency/runs'
    if not run_folder.is_relative_to(expected):
        raise ValueError('X1 rendering requires an X1 run directory')
    verify_manifest(run_folder)
    groups = {str(mode): read_json(run_folder / f'邻接{mode}/结果.json') for mode in (4, 8)}
    check_pair(groups)
    for mode, result in groups.items():
        verify_manifest(run_folder / f'邻接{mode}')
        check_result(result)
    destination = RESULTS_ROOT / 'x1_adjacency/showcase' / run_folder.name
    destination.mkdir(parents=True, exist_ok=False)
    reps = destination / '代表帧'; reps.mkdir()
    blocked = forbidden_nodes(groups['4']['config'])
    count = len(groups['4']['snapshots'])
    keys = {0, 8, 11, 20, 23, 24, 40, count - 1} | {r['summary']['team_completion_time_s'] for r in groups.values() if r['summary']['team_completion_time_s'] is not None}
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    frames, records = [], []
    for index in range(count):
        data = draw_frame(fig, groups, index, blocked)
        assert data['4']['time_s'] == data['8']['time_s'] == index
        assert data['4']['current_m_per_s'] == data['8']['current_m_per_s']
        records.append(data)
        buf = io.BytesIO(); fig.savefig(buf, format='png', dpi=90); buf.seek(0)
        with Image.open(buf) as im:
            frames.append(im.convert('RGB').quantize(colors=256))
        if index in keys:
            fig.savefig(reps / f'并排_t{index:03d}.png', dpi=120)
        if index % 30 == 0:
            print(f'同帧并排：已渲染实际t={index}s', flush=True)
    plt.close(fig)
    gif = destination / '四八邻接_同帧并排.gif'
    frames[0].save(gif, save_all=True, append_images=frames[1:], duration=[100] * (count - 1) + [1000], loop=0, optimize=False, disposal=2)
    with Image.open(gif) as im:
        assert im.n_frames == count and im.size == (1440, 810)
        for index in range(im.n_frames):
            im.seek(index); im.load()
            assert im.info['duration'] == (1000 if index == count - 1 else 100)
    draw_overlay(groups, destination / '四八邻接_实际路径叠图.png', blocked)
    for mode, result in groups.items():
        last, s = records[-1][mode], result['summary']
        assert last['completed'] == s['completed_task_count']
        assert last['team_completion_time_s'] == s['team_completion_time_s']
        assert abs(last['actual_total_distance_m'] - s['actual_total_distance_m']) < 1e-8
    save(destination / '帧来源.json', {'frames': records, 'results': {mode: {'path': str(run_folder / f'邻接{mode}/结果.json'), 'sha256': digest(run_folder / f'邻接{mode}/结果.json')} for mode in groups}})
    save(destination / '显示核查.json', {'status': 'PASS', 'frame_count': count, 'dimensions_px': [1440, 810], 'same_times_scales_flows': 'PASS',
         'fixed_flow_scale': '0.2m/s =2 map metres', 'representative_times_s': sorted(keys), 'resimulation': False, 'solver_rerun': False,
         'visual_review': 'pending PNG inspection; decoding and source checks do not imply visual acceptance'})
    save(destination / '来源.json', {'run': str(run_folder), 'run_manifest_sha256': digest(run_folder / 'SHA256.txt'),
         'renderer': str(Path(__file__).relative_to(PROJECT_ROOT)), 'renderer_sha256': digest(Path(__file__)), 'no_old_log_splice': True})
    text = (run_folder / '摘要.md').read_text().replace('# X1-L4 P4融合实际对照', '# X1-L4 同场景展示', 1)
    text += f'\n[同帧并排GIF](<{gif}>) · [实际路径叠图](<{destination / "四八邻接_实际路径叠图.png"}>) · [本人运行原始摘要](<{run_folder / "摘要.md"}>)\n'
    (destination / '摘要.md').write_text(text)
    write_manifest(destination)
    return destination
