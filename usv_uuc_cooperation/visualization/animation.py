"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L4/源码/p3_l4_animation.py
"""
import os
import tempfile
from pathlib import Path
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir())/"usv-uuv-matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Rectangle



def background(ax, result, blocked):
    for x, y in sorted(blocked):
        ax.add_patch(Rectangle((x - .5, y - .5), 1, 1, facecolor='#e5a69a', alpha=.35, linewidth=0))
    for circle in result['config']['map']['circles']:
        ax.add_patch(Circle(circle['center_m'], circle['radius_m'], facecolor='#c46b59', alpha=.7))
        ax.add_patch(Circle(circle['center_m'], circle['radius_m'] + 1, fill=False,
                            linestyle='--', edgecolor='#b34532', linewidth=.8))
    ax.set(xlim=(0, 24), ylim=(0, 18), xlabel='x (m)', ylabel='y (m)')
    ax.set_aspect('equal'); ax.grid(alpha=.15)


def render(result, blocked, base):
    ids = result['config']['uuc_ids'] + ['USV']
    colors = {b: c for b, c in zip(ids, ('#1f77b4', '#9467bd', '#17becf', '#ff7f0e', '#636363', '#c52930'))}
    snapshots = result['snapshots']
    tasks = sorted(result['tasks'])
    fig, ax = plt.subplots(figsize=(8.8, 6.8))
    background(ax, result, blocked)
    task_artist = ax.scatter([result['config']['tasks_m'][t][0] for t in tasks],
                             [result['config']['tasks_m'][t][1] for t in tasks], s=22, zorder=3)
    for task in tasks:
        ax.annotate(task, result['config']['tasks_m'][task], xytext=(3, 4), textcoords='offset points', fontsize=6)
    trails, dots, labels, connections = {}, {}, {}, {}
    for b in ids:
        trails[b], = ax.plot([], [], color=colors[b], linewidth=1, alpha=.5)
        dots[b], = ax.plot([], [], marker='*' if b == 'USV' else 's', color=colors[b], markersize=8)
        labels[b] = ax.text(0, 0, b, color=colors[b], fontsize=7)
        if b != 'USV':
            connections[b], = ax.plot([], [], color='#459661', linewidth=.9, alpha=.55)
    radius = Circle((0, 0), result['config']['communication']['radius_m'], fill=False,
                    edgecolor=colors['USV'], linestyle=':', linewidth=.8)
    ax.add_patch(radius)
    ax.set_title(f'{result["case"]["case_id"]} | actual motion; green tasks completed', fontsize=10)
    header = fig.text(.05, .025, '', fontsize=9)

    def update(index):
        snap = snapshots[index]
        for b in ids:
            point = snap['positions_m'][b]
            trail = [s['positions_m'][b] for s in snapshots[:index + 1]]
            trails[b].set_data([p[0] for p in trail], [p[1] for p in trail])
            dots[b].set_data([point[0]], [point[1]])
            labels[b].set_position((point[0] + .25, point[1] + .3))
            failure = result['summary']['boat_totals'][b]['failure']
            labels[b].set_text(b + (' STOP' if failure and index > failure['decision_time_s'] else ''))
            if b != 'USV':
                usv = snap['positions_m']['USV']
                connections[b].set_data([usv[0], point[0]] if snap['connected'][b] else [],
                                        [usv[1], point[1]] if snap['connected'][b] else [])
        radius.center = snap['positions_m']['USV']
        task_artist.set_color(['#288950' if t in snap['completed_tasks'] else '#39778d' for t in tasks])
        header.set_text(f't = {index:3d} s  |  completed {len(snap["completed_tasks"]):2d}/20  |  links {sum(snap["connected"].values())}/5  |  no boat-collision model')

    fig.tight_layout(rect=(0, .05, 1, 1))
    animation = FuncAnimation(fig, update, frames=len(snapshots), interval=100, blit=False)
    gif = Path(str(base) + '_动画.gif')
    animation.save(gif, writer=PillowWriter(fps=10), dpi=75)
    checkpoints = [0, 8, result['summary']['team_completion_time_s'] or 20, 120]
    outputs = [gif.name]
    for index in dict.fromkeys(checkpoints):
        update(index)
        path = Path(str(base) + f'_t{index:03d}.png')
        fig.savefig(path, dpi=125); outputs.append(path.name)
    plt.close(fig)
    return outputs

