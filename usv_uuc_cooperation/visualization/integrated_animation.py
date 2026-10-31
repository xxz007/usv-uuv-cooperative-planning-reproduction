"""Render the single integrated timeline from saved, hash-verified results only."""
import io
import os
import tempfile
from pathlib import Path
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "usv-uuv-matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from PIL import Image

from usv_uuc_cooperation.config import RESULTS_ROOT, DOCS_ROOT, PROJECT_ROOT, read_json
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.experiments.p4_integrated import verify_manifest, save
from usv_uuc_cooperation.metrics.logger import digest

plt.rcParams.update({"font.family": "PingFang SC", "axes.unicode_minus": False})
COLORS = dict(zip(["U1", "U2", "U3", "U4", "U5", "USV"],
                  ["#bd6818", "#9852b3", "#228a89", "#b84054", "#63713a", "#df2430"]))
KEY_TIMES = [0, 8, 10, 11, 20, 23, 24, 54, 65, 120]


def frame_data(result, snap):
    t = snap["time_s"]
    complete = sum(a["status"] == "completed" for a in snap["tasks"].values())
    # Never display a future completion time before it actually happened.
    team_time = result["summary"]["team_completion_time_s"]
    if team_time is None or t < team_time:
        team_time = None
    step = next((r for r in result["steps"] if r["time_s"] == t and r["boat_id"] == "U2"), None)
    return {"time_s": t, "current_m_per_s": snap["current_m_per_s"],
            "known_tasks": len(snap["tasks"]), "completed": complete,
            "team_completion_time_s": team_time,
            "actual_moving_NEU": sum(b["energy_NEU"] for b in snap["boats"].values()),
            "positions_m": {b: s["position_m"] for b, s in snap["boats"].items()},
            "task_states": {a: {k: v[k] for k in ("status", "boat_id", "service_progress_s", "completion_s")}
                            for a, v in snap["tasks"].items()},
            "observed_events": [e["event_id"] for e in result["event_log"] if e["time_s"] <= t],
            "velocity_example": None if step is None else {k: step[k] for k in
                ("boat_id", "time_s", "next_time_s", "action", "water_m_per_s", "current_m_per_s", "ground_m_per_s")}}


def draw_frame(fig, result, index, blocked):
    fig.clear()
    snap = result["snapshots"][index]
    data = frame_data(result, snap)
    t, current = data["time_s"], data["current_m_per_s"]
    dynamic = result["strategy"] == "DYNAMIC_REPLAN"
    ax = fig.add_axes([.055, .235, .65, .635])
    ax.set(xlim=(-.5, 24.5), ylim=(-.5, 18.5), xlabel="x / m", ylabel="y / m")
    ax.set_aspect("equal")
    ax.set_facecolor("#eef7fc")
    ax.grid(alpha=.12)
    for x, y in sorted(blocked):
        ax.add_patch(Rectangle((x-.5, y-.5), 1, 1, facecolor="#d4a89a", alpha=.45, lw=0))
    for circle in result["config"]["map"]["circles"]:
        ax.add_patch(Circle(circle["center_m"], circle["radius_m"], facecolor="#a96f61", alpha=.9))
        ax.add_patch(Circle(circle["center_m"], circle["radius_m"]+1, fill=False, ls="--", color="#a45748", lw=.8))
    # Fixed scale: 0.2 m/s always occupies 2 map metres. No per-frame autoscaling.
    if any(current):
        xy = [(x, y) for x in range(1, 24, 3) for y in range(1, 18, 3)]
        ax.quiver([a[0] for a in xy], [a[1] for a in xy], [current[0]]*len(xy),
                  [current[1]]*len(xy), color="#005bd4", alpha=.65,
                  angles="xy", scale_units="xy", scale=.1, width=.0035, zorder=2)
    ax.annotate("", xy=(3, .4), xytext=(1, .4), arrowprops=dict(arrowstyle="->", color="#005bd4", lw=2))
    ax.text(3.3, .4, "洋流标尺 0.20 m/s（固定）", fontsize=8, va="center", color="#005bd4",
            bbox=dict(facecolor="white", edgecolor="none", alpha=.85, pad=2))
    for b, state in snap["boats"].items():
        trail = [s["boats"][b]["position_m"] for s in result["snapshots"][:index+1]]
        ax.plot(*zip(*trail), color=COLORS[b], lw=1.7, alpha=.85, zorder=4)
        pos = state["position_m"]
        plan = [pos] + [r["end"] for r in snap["remaining_plans"].get(b, []) if r["action"] == "move"]
        if len(plan) > 1:
            ax.plot(*zip(*plan), ls="--", lw=1, color=COLORS[b], alpha=.65, zorder=3)
        if dynamic:
            previous = next((s for s in reversed(result["snapshots"][:index+1])
                             if s.get("old_plans") and 0 <= t-s["time_s"] <= 3), None)
            if previous:
                old = [previous["boats"][b]["position_m"]] + [r["end"] for r in previous["old_plans"].get(b, []) if r["action"] == "move"]
                if len(old) > 1:
                    ax.plot(*zip(*old), ls=":", color="#727a83", lw=1.5, alpha=.6, zorder=2)
        ax.scatter(*pos, marker="X" if state["failure"] else "*" if b=="USV" else "s",
                   s=110 if b=="USV" else 48, color=COLORS[b], edgecolor="white", lw=.7, zorder=7)
        # U4/U2 can share the service point. Offset their labels in opposite directions.
        ax.annotate(b + (" 任务退出" if state["failure"] else ""), pos,
                    xytext=(-42, -12) if b=="U4" else (5, 7), textcoords="offset points",
                    fontsize=8, color=COLORS[b], zorder=9)
    usv = snap["boats"]["USV"]["position_m"]
    ax.add_patch(Circle(usv, result["config"]["communication"]["radius_m"], fill=False,
                        color="#409166", ls=":", lw=.9, alpha=.6))
    for b, connected in snap["connected"].items():
        if connected:
            p = snap["boats"][b]["position_m"]
            ax.plot([usv[0],p[0]], [usv[1],p[1]], color="#31a86e", lw=1, alpha=.65, zorder=3)
    for task, state in snap["tasks"].items():
        status = state["status"]
        color = {"completed":"#219456", "servicing":"#e8ad14", "orphaned":"#d12b47"}.get(status, "#eef7fc")
        edge = COLORS.get(state["boat_id"], "#d12b47")
        ax.scatter(*state["position_m"], s=82 if state["release_s"] else 38,
                   marker="*" if state["release_s"] else "o", color=color, edgecolor=edge, lw=1.1, zorder=6)
        ax.annotate(task, state["position_m"], xytext=(4,-9), textcoords="offset points", fontsize=7,
                    zorder=8, bbox=dict(facecolor="white", edgecolor="none", alpha=.65, pad=.1))
    fig.text(.055, .947, "P4 连续海上作业  |  " + ("事件响应与重规划" if dynamic else "冻结初始归属"), fontsize=17, weight="bold")
    fig.text(.055, .912, f"{result['strategy']}    t = {t:03d} s    ·    实际日志快照 / 10 倍速播放", fontsize=11, color="#475569")
    fig.text(.737, .846, "当前洋流", fontsize=12, weight="bold", color="#005bd4")
    fig.text(.737, .82, f"c = ({current[0]:+.2f}, {current[1]:+.2f}) m/s\n|c| = {(current[0]**2+current[1]**2)**.5:.3f} m/s", fontsize=10, linespacing=1.6, va="top")
    fig.text(.737, .748, "实线：实际艇轨迹\n彩色虚线：当前剩余计划\n灰色点线：事件前旧计划（3s）\n绿色连线：水平通信 ≤ 6m\n绿圆：完成   黄圆：服务中\n红圈：遗留   星形任务：新到达", fontsize=9, linespacing=1.65, va="top")
    for j,b in enumerate(COLORS):
        fig.text(.737 + (j%3)*.075, .605-(j//3)*.028, b, fontsize=9, color=COLORS[b], weight="bold")
    va = fig.add_axes([.763, .265, .205, .26])
    va.set(xlim=(-1.25,1.25), ylim=(-1.25,1.25), aspect="equal")
    va.axhline(0, lw=.5, color="#ccd3db"); va.axvline(0,lw=.5,color="#ccd3db")
    va.tick_params(labelsize=7); va.grid(alpha=.2)
    v = data["velocity_example"]
    if v:
        water, flow, ground = v["water_m_per_s"], v["current_m_per_s"], v["ground_m_per_s"]
        for origin, vec, col in [([0,0],water,"#dd702e"),(water,flow,"#005bd4"),([0,0],ground,"#25364a")]:
            if any(vec):
                va.annotate("", xy=(origin[0]+vec[0], origin[1]+vec[1]), xytext=origin,
                            arrowprops=dict(arrowstyle="->",color=col,lw=1.7))
        va.set_title(f"UUV U2 · [{t},{t+1}) s\n相对水体 + 洋流 = 对地", fontsize=8)
        fig.text(.745,.233,f"橙：水体相对速度   蓝：洋流\n黑：对地速度   单位 m/s · {v['action']}",fontsize=8,linespacing=1.4,va="top")
    else:
        va.set_title("终态：无下一控制区间",fontsize=9)
    observed = [e for e in result["event_log"] if e["time_s"] <= t]
    event_lines = []
    if observed:
        event_lines.append("t=11  T21/T22 到达；" + ("保护 U1/T06 已进行服务" if dynamic else "不接受新任务"))
    if any(e["type"]=="UUV_FAILURE" for e in observed):
        event_lines.append("t=24  U4 任务退出，T11 服务 1→0；" + ("U2 接手完整服务" if dynamic else "保留故障遗留"))
    if not event_lines:
        event_lines=["t=0  共享 Pyomo 初始分配 → 访问顺序 / A*；尚未观测到任务或故障事件"]
    fig.text(.055,.15,"\n".join(event_lines),fontsize=10,linespacing=1.5,color="#334155",va="top")
    time_label = str(data["team_completion_time_s"])+" s" if data["team_completion_time_s"] is not None else "null" if t==120 else "进行中"
    fields=[("完成数",f"{data['completed']}/{data['known_tasks']}"),("团队完成时间",time_label),("实际移动 NEU",f"{data['actual_moving_NEU']:.3f}")]
    for x,(label,value) in zip((.055,.35,.645),fields):
        fig.text(x,.082,label,fontsize=10,color="#64748b")
        fig.text(x+.115,.082,value,fontsize=15,weight="bold",color="#192c43")
    fig.text(.055,.03,"二维课堂代理 · 服务/等待/故障理想驻留 · NEU仅实际移动，统计全窗[0,120)含后续中继 · 截止≠完工",fontsize=9,color="#64748b")
    return data


def render_pair(run_folder):
    run_folder = Path(run_folder).resolve()
    verify_manifest(run_folder)
    destination = RESULTS_ROOT / "P4/P4-V01/showcase"
    destination.mkdir(parents=True, exist_ok=False)
    reps = destination / "代表帧"
    reps.mkdir()
    metadata, checks = {}, {}
    for strategy in ("STATIC_FROZEN", "DYNAMIC_REPLAN"):
        part = run_folder / strategy
        verify_manifest(part)
        result = read_json(part / "结果.json")
        blocked = forbidden_nodes(result["config"])
        fig = plt.figure(figsize=(13,9), facecolor="white")
        frames, records = [], []
        for i,snap in enumerate(result["snapshots"]):
            records.append(draw_frame(fig,result,i,blocked))
            buf = io.BytesIO()
            fig.savefig(buf,format="png",dpi=90)
            buf.seek(0)
            with Image.open(buf) as im:
                frames.append(im.convert("RGB").quantize(colors=256))
            if snap["time_s"] in KEY_TIMES:
                fig.savefig(reps / f"{strategy}_t{snap['time_s']:03d}.png",dpi=130)
            if i%30==0:
                print(f"{strategy}: rendered actual t={snap['time_s']}s",flush=True)
        plt.close(fig)
        gif=destination/f"{strategy}.gif"
        frames[0].save(gif,save_all=True,append_images=frames[1:],duration=[100]*120+[1000],loop=0,optimize=False,disposal=2)
        with Image.open(gif) as im:
            assert im.n_frames==len(records)==121
            assert im.size==(1170,810)
            durations=[]
            for i in range(im.n_frames):
                im.seek(i); im.load(); durations.append(im.info['duration'])
        assert [r['time_s'] for r in records]==list(range(121))
        for record,snap in zip(records,result['snapshots']):
            assert record['known_tasks']==(20 if record['time_s']<11 else 22)
            assert record['current_m_per_s']==snap['current_m_per_s']
            v=record['velocity_example']
            if v:
                assert all(abs(w+c-g)<1e-9 for w,c,g in zip(v['water_m_per_s'],v['current_m_per_s'],v['ground_m_per_s']))
        last=records[-1]; summary=result['summary']
        assert last['completed']==summary['completed_task_count']
        assert last['team_completion_time_s']==summary['team_completion_time_s']
        assert abs(last['actual_moving_NEU']-summary['total_energy_NEU'])<1e-9
        metadata[strategy]={"result_path":str(part/'结果.json'),"result_sha256":digest(part/'结果.json'),"frames":records}
        checks[strategy]={"status":"PASS","frames":121,"actual_times_s":[0,120],"dimensions_px":[1170,810],
                          "duration_ms":durations,"gif_sha256":digest(gif),"representative_times_s":KEY_TIMES,
                          "currents_events_task_counts_terminal_metrics_velocity_identity":"PASS"}
        del frames
    save(destination/'帧来源.json',metadata)
    save(destination/'显示核查.json',{"status":"PASS","method":"saved-log checks and Pillow decoding; PNG visual review recorded separately", "groups":checks,
                                     "flow_scale":"0.2m/s = 2 map metres; fixed scale .1", "playback":"10x; t120 held1s, terminal is not observation interval",
                                     "resimulation":False,"rerun_solver":False})
    save(destination/'统一配置.json',read_json(run_folder/'统一配置.json'))
    save(destination/'指标.json',read_json(run_folder/'指标.json'))
    save(destination/'来源.json',{"run_folder":str(run_folder),"run_manifest_sha256":digest(run_folder/'SHA256.txt'),
                                  "renderer":str(Path(__file__).relative_to(PROJECT_ROOT)),"renderer_sha256":digest(Path(__file__)),
                                  "rendered_from":"each group's own actual integrated result JSON; no lesson log splice"})
    (destination/'摘要.md').write_text("# P4 统一案例展示\n\n两组来自同一冻结配置、同一初始解的实际统一执行。\n\n"
        "| 策略 | 完成数 | 团队完成时间 s | 实际移动 NEU |\n|---|---|---|---|\n"+
        ''.join(f"| {r['strategy']} | {r['completed']}/{r['known_tasks']} | {r['team_completion_time_s'] if r['team_completion_time_s'] is not None else 'null'} | {r['total_energy_NEU']:.6f} |\n" for r in read_json(run_folder/'指标.json'))+
        f"\n[冻结初始归属](<{destination/'STATIC_FROZEN.gif'}>) · [动态重规划](<{destination/'DYNAMIC_REPLAN.gif'}>)\n\n"
        f"[实际运行摘要](<{run_folder/'摘要.md'}>) · [代表帧](<{reps}>) · [实现与核查](<{DOCS_ROOT/'P4统一案例实现与展示_V01.md'}>)\n\n"
        "t11 两新任务到达（20→22）；t24 U4任务能力故障，未完成服务清零。健康UUV不受事件打断；接手T11重新完整服务。\n\n"
        "每个GIF121个真实快照，t0..120，10倍播放。蓝色洋流固定m/s标尺；彩虚线当前计划、灰点线旧计划、彩实线实际轨迹。\n\n"
        "初始Pyomo253m仅起点到任务A*分配代理；不称完整路线或动态联合最优。NEU是六艇[0,120)全窗实际移动，包含完工后的中继；两组完成量不同，不能据此断言同任务节能。\n\n"
        "师方统一执行与展示；沿用课堂4/4原验收，本任务未新增本人操作。二维代理、水平通信与理想定点补偿边界保持。\n",encoding="utf-8")
    (destination/'SHA256.txt').write_text(''.join(f"{digest(p)}  {p.relative_to(destination)}\n" for p in sorted(destination.rglob('*')) if p.is_file()))
    return destination
