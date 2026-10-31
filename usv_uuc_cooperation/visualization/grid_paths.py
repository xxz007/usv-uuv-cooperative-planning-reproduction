"""Static overlay from saved geometric paths; no simulated motion or timing."""
import os
from pathlib import Path
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "usv-uuv-matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def render(result, target):
    plt.rcParams.update({"font.family": "PingFang SC", "axes.unicode_minus": False})
    grid = result["config"]["map"]
    fig, ax = plt.subplots(figsize=(10, 7.5))
    fig.subplots_adjust(left=.08, right=.98, top=.91, bottom=.14)
    ax.set_aspect("equal")
    ax.set(xlim=(-.5, 24.5), ylim=(-.5, 18.5), xlabel="x / m", ylabel="y / m",
           title="X1-L1｜同一P4地图：四邻接 / 安全八邻接")
    ax.set_facecolor("#f5f9fc")
    ax.set_xticks(range(grid["x_range"][0], grid["x_range"][1] + 1, 2))
    ax.set_yticks(range(grid["y_range"][0], grid["y_range"][1] + 1, 2))
    ax.grid(alpha=.15)
    for x, y in result["blocked_nodes"]:
        ax.add_patch(Rectangle((x-.5, y-.5), 1, 1, facecolor="#bdc4cc", alpha=.7, lw=.35,
                               edgecolor="white"))
    for circle in grid["circles"]:
        ax.add_patch(Circle(circle["center_m"], circle["radius_m"], facecolor="#737d88", alpha=.9))
        ax.add_patch(Circle(circle["center_m"], circle["radius_m"] + grid["safety_buffer_m"],
                            fill=False, ls="--", color="#737d88", lw=1))
    for mode, color, style in (("4", "#2874b5", "--"), ("8", "#db6824", "-")):
        route = result["routes"][mode]
        ax.plot(*zip(*route["path"]), ls=style, color=color, lw=2, marker="o", ms=3,
                label=f"{mode}邻接：{route['distance_m']:.3f} m / {route['step_count']}步", zorder=4)
    for point, label in ((result["config"]["start_m"], "起点 U1"),
                         (result["config"]["goal_m"], "终点 T03")):
        ax.scatter(*point, s=100, facecolor="white", edgecolor="#18344b", zorder=6)
        ax.annotate(label, point, xytext=(8, 8), textcoords="offset points", fontsize=11, zorder=7)
    ax.legend(loc="lower right", framealpha=.95)
    fig.text(.5, .035, "灰色：禁行闭方格（含安全缓冲）｜虚线圆：半径 + 1 m缓冲｜仅几何规划，无执行时间指标",
             ha="center", fontsize=10)
    fig.savefig(target, dpi=180, bbox_inches="tight")
    plt.close(fig)
