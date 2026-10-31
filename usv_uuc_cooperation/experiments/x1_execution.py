"""Execute the learner's frozen L1 paths with the unchanged P4 current executor."""
import copy
import sys

from usv_uuc_cooperation.config import (PACKAGE_ROOT, PROJECT_ROOT, FRAMEWORK_VERSION,
                                       new_run_directory, read_json)
from usv_uuc_cooperation.control.current_executor import simulate
from usv_uuc_cooperation.control.integrated_executor import current_at
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.metrics.execution_check import audit
from usv_uuc_cooperation.metrics.logger import digest, write_csv


def load_inputs():
    spec = read_json(PACKAGE_ROOT / "configs/X1-L2_V01.json")
    inputs = {}
    for label, reference in spec["sources"].items():
        path = PROJECT_ROOT / reference["path"]
        if digest(path) != reference["sha256"]:
            raise ValueError(f"Frozen L2 input changed: {label}")
        inputs[label] = read_json(path)
    source, saved, geometry = inputs["p4_config"], inputs["l1_paths"], inputs["l1_config"]
    assert saved["config"] == geometry
    assert source["map"] == geometry["map"]
    assert geometry["start_m"] == source["starts_m"]["U1"]
    assert geometry["goal_m"] == source["tasks_m"]["T03"]
    config = copy.deepcopy(source)
    config.update(config_version="X1-L2_V01", uuc_ids=["U1"],
                  starts_m={b: source["starts_m"][b] for b in ("U1", "USV")},
                  tasks_m={"T03": source["tasks_m"]["T03"]}, events=[])
    assert config["dt_s"] == 1 and config["speed_m_per_s"]["U1"] == 1
    return spec, config, saved["routes"]


def execute_route(config, route, currents):
    path = route["path"]
    assert path[0] == config["starts_m"]["U1"] and path[-1] == config["tasks_m"]["T03"]
    # The planner supplies waypoints, not a one-second teleport per grid edge.
    ops = [{"action": "move", "start": list(a), "end": list(b), "task": "T03",
            "arrived": i == len(path) - 2} for i, (a, b) in enumerate(zip(path, path[1:]))]
    ops.extend({"action": "service", "start": list(path[-1]), "end": list(path[-1]),
                "task": "T03", "arrived": False} for _ in range(config["service_s_per_task"]))
    budgets = {"U1": config["energy"]["uuc_budget_NEU"], "USV": config["energy"]["usv_budget_NEU"]}
    summary, tasks, steps, links, snapshots, controls = simulate(
        config, {"U1": ["T03"]}, {"U1": ops}, forbidden_nodes(config), budgets, currents)
    return {"config": config, "route": route, "currents_m_per_s": currents,
            "summary": summary, "tasks": tasks, "steps": steps, "links": links,
            "snapshots": snapshots, "controls": controls}


def run(run_id):
    from usv_uuc_cooperation.main import assert_frozen_core, save
    assert_frozen_core()
    spec, config, routes = load_inputs()
    folder = new_run_directory("X1", run_id)
    try:
        save(folder / "配置引用.json", spec)
        save(folder / "有效配置.json", config)
        metrics, reports = [], {}
        for flow in ("NO_FLOW", "P4_FLOW"):
            currents = [[0, 0] for _ in range(config["cutoff_time_s"])] if flow == "NO_FLOW" else [
                current_at(config, t) for t in range(config["cutoff_time_s"])]
            for mode in ("4", "8"):
                result = execute_route(config, routes[mode], currents)
                result.update(flow_case=flow, adjacency=int(mode), framework=FRAMEWORK_VERSION)
                report = audit(result)
                tag = f"{flow}_{mode}"
                output = folder / tag
                output.mkdir()
                save(output / "结果.json", result)
                save(output / "机械核查.json", report)
                write_csv(output / "逐步任务能量.csv", result["steps"])
                write_csv(output / "逐步控制.csv", result["controls"])
                reports[tag] = report
                metrics.append({"flow_case": flow, "adjacency": int(mode),
                                "actual_distance_m": report["actual_distance_m"],
                                "energy_NEU": report["energy_NEU"], "arrival_s": report["arrival_s"]})
        save(folder / "指标.json", metrics)
        save(folder / "机械核查.json", {"status": "PASS", "cases": reports,
                                       "learner_acceptance": "PENDING"})
        rows = "\n".join(f"| {r['flow_case']} | {r['adjacency']} | {r['actual_distance_m']:.9f} | "
                         f"{r['energy_NEU']:.9f} | {r['arrival_s']} |" for r in metrics)
        timing = "\n".join(f"| {tag} | {r['arrival_s']} | {r['first_service_interval_s']} | {r['completion_s']} | "
                           f"{r['peak_water_speed_m_per_s']:.9f} |" for tag, r in reports.items())
        (folder / "摘要.md").write_text(
            f"# X1-L2 速度、时间与能耗\n\n{FRAMEWORK_VERSION}；L1本人保存路径直接执行，不重规划。\n\n"
            "主读数仅U1，USV辅助日志单列；先无流子例，再原P4半开时段洋流。每组相同地图、路径来源、"
            "水体速度上限1m/s、dt=1s、完整服务2s、预算及每步最多一个航点；剩余时间不跨航点。"
            "只执行U1→T03固定队列，P4动态事件在本子例关闭。\n\n"
            "| 子例 | 邻接 | actual_distance_m 实际航程/m | energy_NEU 能耗 | arrival_s 到达/s |\n"
            f"|---|---:|---:|---:|---:|\n{rows}\n\n"
            "| 子例/邻接 | 到达/s | 首服务区间/s（左闭右开） | 完成/s | 水体速度峰值 m/s |\n"
            f"|---|---:|---|---:|---:|\n{timing}\n\n"
            "E=2L；驻留能耗仍按原几何模型为0。日志中的水体速度由实际位移减当步流速重建，"
            "不以斜边节点数计时间或航程。每个航点到达时刻见机械核查中的waypoint_arrivals；"
            "到达步不计服务，首个服务区间从已到达时刻开始。\n\n"
            f"[完整指标]({folder / '指标.json'}) · [机械核查]({folder / '机械核查.json'}) · "
            f"[无流八邻接逐步账本]({folder / 'NO_FLOW_8/逐步任务能量.csv'}) · "
            f"[无流八邻接控制]({folder / 'NO_FLOW_8/逐步控制.csv'})\n\n"
            "教师机械PASS不替代本人读数与因果解释；课堂验收另存本课验收目录。\n", encoding="utf-8")
        save(folder / "运行元数据.json", {"framework": FRAMEWORK_VERSION, "lesson": "X1-L2",
            "command": sys.argv, "python": sys.executable, "output": str(folder), "status": "COMPLETE",
            "config_sha256": digest(PACKAGE_ROOT / "configs/X1-L2_V01.json"), "sources": spec["sources"],
            "source_manifest": f"输出文件/版本快照/{FRAMEWORK_VERSION}/源码配置_SHA256.txt"})
        (folder / "SHA256.txt").write_text("\n".join(
            f"{digest(p)}  {p.relative_to(folder).as_posix()}" for p in sorted(folder.rglob("*")) if p.is_file()) + "\n")
    except Exception as exc:
        save(folder / "失败.json", {"status": "FAILED", "error": str(exc)})
        raise
    return folder
