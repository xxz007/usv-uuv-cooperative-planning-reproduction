"""Plot saved P5 VRX/ideal-2D CSVs without launching or replaying a simulator."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path


def render(source_run, output):
    source_run, output = Path(source_run), Path(output)
    if not source_run.is_absolute() or not output.is_absolute():
        raise ValueError("--source-run and --output must be absolute paths")
    source_run, output = source_run.resolve(), output.resolve()
    names = ("common_inputs.json", "odometry.csv", "reference_2d.csv", "commands.csv", "summary.json")
    sources = {name: (source_run / name).read_bytes() for name in names}
    common = json.loads(sources["common_inputs.json"])
    summary = json.loads(sources["summary.json"])
    metrics = {row["system"]: row for row in summary["metrics"]}
    task, envelope = common["task"], common["operating_envelope"]
    start, goal = common["start"], common["goal"]
    forward_x, forward_y = common["forward_unit"]
    limits = {"VRX": metrics["VRX"]["completion_time_s"],
              "ideal_2D": metrics["ideal_2D"]["completion_time_s"]}
    if any(value is not None and (not isinstance(value, (int, float))
                                 or not math.isfinite(value) or value < 0) for value in limits.values()):
        raise ValueError("Completion times must be finite nonnegative numbers or null")
    datasets, raw_counts = {}, {}
    required = {
        "odometry.csv": ("elapsed_sim_s", "x_m", "y_m", "goal_distance_m", "cross_track_m"),
        "reference_2d.csv": ("elapsed_sim_s", "x_m", "y_m", "goal_distance_m", "cross_track_m"),
        "commands.csv": ("elapsed_sim_s", "left_thrust_n", "right_thrust_n")
    }
    for name, columns in required.items():
        rows = list(csv.DictReader(sources[name].decode("utf-8").splitlines()))
        raw_counts[name], selected = len(rows), []
        system = "ideal_2D" if name == "reference_2d.csv" else "VRX"
        for row in rows:
            if name != "reference_2d.csv" and row["phase"] != "task":
                continue
            if name == "odometry.csv" and row["usable"].lower() != "true":
                continue
            if any(not row.get(key) for key in columns):
                continue
            sample = {key: float(row[key]) for key in columns}
            if not all(math.isfinite(value) for value in sample.values()):
                continue
            t = sample["elapsed_sim_s"]
            if t < 0 or (limits[system] is not None and t > limits[system]+1e-9):
                continue
            selected.append(sample)
        if any(b["elapsed_sim_s"] < a["elapsed_sim_s"] for a, b in zip(selected, selected[1:])):
            raise ValueError(f"Nonmonotonic saved task times in {name}")
        datasets[name] = selected

    output.mkdir(parents=True, exist_ok=False)
    os.environ["MPLCONFIGDIR"] = str(output / ".matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    # Use the existing P5 MOOS figure's ASCII labels and bundled DejaVu font.
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False, "pdf.fonttype": 42})
    vrx, reference = datasets["odometry.csv"], datasets["reference_2d.csv"]
    blue, orange = "#176B91", "#A65E00"
    role, status = summary["role"], summary["status"]
    title = f"P5 one-waypoint task | {role} | {status}"
    fig, ax = plt.subplots(figsize=(10, 7), layout="constrained")
    corners = [[start["x_m"]+along*forward_x-cross*forward_y,
                start["y_m"]+along*forward_y+cross*forward_x]
               for along, cross in ((envelope["along_m"][0], envelope["cross_m"][0]),
                                    (envelope["along_m"][1], envelope["cross_m"][0]),
                                    (envelope["along_m"][1], envelope["cross_m"][1]),
                                    (envelope["along_m"][0], envelope["cross_m"][1]))]
    ax.add_patch(Polygon(corners, closed=True, fill=False, edgecolor="#777777", lw=1,
                         linestyle="--", label="Predefined coordinate envelope"))
    ax.plot([start["x_m"], goal["x_m"]], [start["y_m"], goal["y_m"]],
            color="#B0B0B0", lw=1, linestyle=":", label="Frozen start-to-goal line")
    ax.add_patch(Circle((goal["x_m"], goal["y_m"]), task["arrival_radius_m"],
                        color="#72AE83", alpha=0.25))
    ax.scatter(goal["x_m"], goal["y_m"], marker="*", s=150, color="#28713E",
               label="Goal and arrival radius", zorder=6)
    if reference:
        ax.plot([row["x_m"] for row in reference], [row["y_m"] for row in reference],
                "--", color=orange, lw=2, label="Independently executed ideal 2D")
    if vrx:
        ax.plot([row["x_m"] for row in vrx], [row["y_m"] for row in vrx],
                ".-", color=blue, lw=1.5, markersize=3, label="Received VRX odometry samples")
        ax.scatter(vrx[-1]["x_m"], vrx[-1]["y_m"], s=70, facecolors="none", edgecolors=blue,
                   label="Last selected VRX sample", zorder=7)
    else:
        ax.text(0.03, 0.97, "No valid VRX task samples received", transform=ax.transAxes,
                va="top", color=blue)
    ax.scatter(start["x_m"], start["y_m"], color="#222222", s=35,
               label="Frozen observed start", zorder=8)
    ax.annotate("", xy=(start["x_m"]+2*forward_x, start["y_m"]+2*forward_y),
                xytext=(start["x_m"], start["y_m"]), arrowprops={"arrowstyle": "->", "color": "#222222"})
    ax.set(title=title+"\nSame native-odometry coordinates",
           xlabel=f"Native odometry X / m (frame: {common['coordinate_frame']})", ylabel="Native odometry Y / m")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.2)
    ax.margins(0.08)
    ax.legend(loc="best", fontsize=8)
    fig.savefig(output / "2D_VRX_same_task.png", dpi=300)
    fig.savefig(output / "2D_VRX_same_task.pdf")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, layout="constrained")
    for rows, color, label in ((vrx, blue, "Received VRX samples"), (reference, orange, "Ideal 2D reference")):
        if rows:
            times = [row["elapsed_sim_s"] for row in rows]
            axes[0].plot(times, [row["goal_distance_m"] for row in rows], color=color, lw=1.7, label=label)
            axes[1].plot(times, [row["cross_track_m"] for row in rows], color=color, lw=1.7, label=label)
    axes[0].axhline(task["arrival_radius_m"], color="#28713E", linestyle="--", lw=1, label="Arrival radius")
    for system, color in (("VRX", blue), ("ideal_2D", orange)):
        if limits[system] is not None:
            for ax in axes:
                ax.axvline(limits[system], color=color, linestyle=":", lw=1, alpha=0.8)
    commands = datasets["commands.csv"]
    if commands:
        times = [row["elapsed_sim_s"] for row in commands]
        axes[2].step(times, [row["left_thrust_n"] for row in commands], where="post", color=blue, label="Left ROS command")
        axes[2].step(times, [row["right_thrust_n"] for row in commands], where="post", color=orange,
                     linestyle="--", label="Right ROS command")
    else:
        axes[2].text(0.03, 0.9, "No task-phase thrust publications", transform=axes[2].transAxes, va="top")
    axes[0].set(title=title, ylabel="Goal distance / m")
    axes[1].set(ylabel="Absolute cross-track error / m")
    axes[2].set(ylabel="Published thrust / N", xlabel="Simulation seconds since frozen dispatch_clock_s")
    axes[2].axhline(0, color="#AAAAAA", lw=0.7)
    for ax in axes:
        ax.grid(alpha=0.2)
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="best", fontsize=8)
        ax.set_xlim(left=0)
    fig.savefig(output / "VRX_progress_error.png", dpi=300)
    fig.savefig(output / "VRX_progress_error.pdf")
    plt.close(fig)

    provenance = {
        "source_run": str(source_run), "framework_version": summary["framework_version"],
        "role": role, "run_status": status, "source_id": common["source_id"],
        "controller_script_sha256": common["script_sha256"], "config_sha256": common["config_sha256"],
        "renderer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_sha256": {name: hashlib.sha256(content).hexdigest() for name, content in sources.items()},
        "raw_row_counts": raw_counts,
        "selected_row_counts": {name: len(rows) for name, rows in datasets.items()},
        "completion_cutoff_sim_s": limits, "metrics_from_saved_summary": summary["metrics"],
        "simulation_runs": 0, "reference_model_runs": 0,
        "window": "VRX: usable task-phase samples at elapsed>=0, through recorded completion if present; shutdown excluded. Ideal 2D: saved rows through its recorded completion. Unfinished completion remains null.",
        "boundary": "Lines connect saved received samples; no lossless transport or continuous-state observation claim. Commands are ROS publications, not Gazebo acknowledgement. Coordinate envelope only; no collision, energy or real-vessel validation."
    }
    (output / "来源.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    lines = ["# P5 VRX 保存日志展示", "", f"运行状态：{status}；角色：{role}；版本：{summary['framework_version']}。", "",
             "[同坐标轨迹 PNG](2D_VRX_same_task.png) · [PDF](2D_VRX_same_task.pdf)", "",
             "[进度、误差与推力 PNG](VRX_progress_error.png) · [PDF](VRX_progress_error.pdf)", "",
             "| 系统 | 原运行状态 | 完成时间（模拟秒） | 最大横向偏差（米） | 坐标包络违规事件 |",
             "|---|---|---:|---:|---:|"]
    for system in ("VRX", "ideal_2D"):
        row = metrics[system]
        values = ["null" if row[key] is None else str(row[key]) for key in
                  ("completion_time_s", "maximum_cross_track_m", "boundary_violation_events")]
        lines.append(f"| {system} | {row['status']} | {' | '.join(values)} |")
    lines += ["", "指标沿用本次保存的 summary.json，不从末尾位置猜测成功；未完成的完成时间保持 null。",
              "", "VRX 图只使用实际收到且标记可用的 task 阶段采样，时间从冻结派发时钟起算；有完成时间时截止该时刻，排除关停阶段。未完成运行显示截至最后有效任务采样的数据。轨迹连线仅连接离散采样，不表示连续观测或传输无丢失。",
              "", "二维曲线读取独立执行保存的 reference_2d.csv，未在绘图时重跑模型。两端共用冻结起点、朝向、目标、标称速度、到达及保持规则；二维是更简单的点运动学参考，不具备 WAM-V 动力学等价性。",
              "", "第三幅子图为实际 ROS 推力发布记录，不等于仿真端已确认接收。包络违规仅指预定义坐标范围；本展示不声明碰撞、真实能耗、实艇验证或本人理解通过。",
              "", f"来源运行：`{source_run}`；文件 SHA 与绘图窗口详见 [来源.json](来源.json)。"]
    (output / "摘要.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Absolute new directory; existing outputs are never overwritten")
    arguments = parser.parse_args()
    print(render(arguments.source_run, arguments.output))
