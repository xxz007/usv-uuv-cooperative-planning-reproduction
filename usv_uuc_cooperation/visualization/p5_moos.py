"""Render only the saved P5 platform trajectories; never launch a simulation."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import tempfile


def render(run):
    from usv_uuc_cooperation.main import assert_frozen_core
    from usv_uuc_cooperation.config import RESULTS_ROOT, FRAMEWORK_VERSION
    assert_frozen_core()
    run = Path(run).resolve()
    output = RESULTS_ROOT / "P5/P5-V01/showcase" / run.name
    output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "usv-uuv-p5-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    config = json.loads((run / "config.json").read_text())
    metrics = json.loads((run / "paired_metrics.json").read_text())
    sources = [run / "example/trajectory.csv", run / "success/trajectory.csv", run / "reference_2d.csv", run / "paired_metrics.json"]
    trajectories = []
    for source in sources[:3]:
        with source.open() as stream:
            trajectories.append([{k: float(v) if v else None for k, v in row.items()} for row in csv.DictReader(stream)])
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    for title, rows, filename in [("MOOS s1_alpha: recorded vessel movement", trajectories[0], "MOOS_original_example"),
                                  ("One assigned task: 2D reference and MOOS execution", trajectories[1], "2D_MOOS_same_task")]:
        fig, ax = plt.subplots(figsize=(9, 6), layout="constrained")
        if filename.startswith("2D"):
            reference = trajectories[2]
            ax.plot([r["x_m"] for r in reference], [r["y_m"] for r in reference], "--", color="#A65E00", lw=2, label="2D kinematic reference")
            goal = metrics["goal_m"]
            ax.add_patch(Circle(goal, config["arrival_tolerance_m"], color="#72AE83", alpha=.25))
            ax.scatter(*goal, marker="*", s=170, color="#28713E", label="Goal and arrival radius", zorder=5)
        ax.plot([r["x_m"] for r in rows], [r["y_m"] for r in rows], color="#176B91", lw=2, label="Actual MOOS NAV trajectory")
        ax.scatter(rows[0]["x_m"], rows[0]["y_m"], color="#222222", s=40, label="First recorded pose", zorder=6)
        ax.scatter(rows[-1]["x_m"], rows[-1]["y_m"], facecolors="none", edgecolors="#176B91", s=70, label="Last recorded pose", zorder=6)
        ax.set(title=title, xlabel="Local ENU x / m (east)", ylabel="Local ENU y / m (north)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.2)
        ax.margins(.15)
        ax.legend(loc="best", fontsize=9)
        fig.savefig(output / (filename + ".png"), dpi=300)
        fig.savefig(output / (filename + ".pdf"))
        plt.close(fig)
    provenance = {"framework": FRAMEWORK_VERSION, "run": str(run), "simulation_runs": 0,
                  "sources": {str(p.relative_to(run)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  "boundary": "Actual MOOS platform simulation; 2D reference has no turning/thrust dynamics; no VRX or real-vessel evidence."}
    (output / "来源.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + "\n")
    (output / "摘要.md").write_text("# P5 保存日志展示\n\n[MOOS原例实际航迹](MOOS_original_example.png) · [二维与MOOS同任务对照](2D_MOOS_same_task.png)\n\n来源为本次运行的真实NAV日志；未重跑模拟。二维参考不含MOOS转弯/推力动力学，结果不代表VRX或实艇验证。\n", encoding="utf-8")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    print(render(parser.parse_args().run))
