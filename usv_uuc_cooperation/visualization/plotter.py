"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L4/源码/p3_l4_map.py
"""
from pathlib import Path



def plot_map(config, report, output):
    import os
    import tempfile
    os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'p3-l4-matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    fig, ax = plt.subplots(figsize=(10, 7.8))
    for x, y in report['blocked_nodes']:
        ax.add_patch(Rectangle((x - .5, y - .5), 1, 1, facecolor='#e5a69a', alpha=.4, linewidth=0))
    for circle in config['map']['circles']:
        ax.add_patch(Circle(circle['center_m'], circle['radius_m'], facecolor='#c46b59', alpha=.8))
        ax.add_patch(Circle(circle['center_m'], circle['radius_m'] + config['map']['safety_buffer_m'],
                            fill=False, edgecolor='#b34532', linestyle='--'))
    for label, (x, y) in config['tasks_m'].items():
        ax.scatter(x, y, c='#176e83', s=28)
        ax.annotate(label, (x, y), xytext=(4, 5), textcoords='offset points', fontsize=8)
    for label, (x, y) in config['starts_m'].items():
        ax.scatter(x, y, marker='*' if label == 'USV' else 's', s=95, color='#233f65')
        ax.annotate(label, (x, y), xytext=(-28, 5), textcoords='offset points', fontsize=9)
    ax.set(xlim=(0, 24), ylim=(0, 18), xlabel='x (m)', ylabel='y (m)',
           title='P3-L4 fixed map | circles, 1 m buffer and conservative forbidden grid cells')
    ax.set_xticks(range(0, 25, 2)); ax.set_yticks(range(0, 19, 2))
    ax.set_aspect('equal'); ax.grid(alpha=.18)
    fig.tight_layout(); fig.savefig(output, dpi=160); plt.close(fig)

