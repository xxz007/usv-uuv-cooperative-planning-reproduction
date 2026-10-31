"""The only P4 main comparison: one initial solution, one frozen environment, two responses."""
import json
from pathlib import Path
import sys
import time

from usv_uuc_cooperation.config import (PACKAGE_ROOT, PROJECT_ROOT, DOCS_ROOT, VERSIONS_ROOT,
                                      FRAMEWORK_VERSION, new_run_directory, read_json)
from usv_uuc_cooperation.control.integrated_executor import simulate, STRATEGIES
from usv_uuc_cooperation.metrics.integrated_check import check_initial, check_result, combination_probes
from usv_uuc_cooperation.metrics.logger import digest, write_csv

CONFIG_PATH = PACKAGE_ROOT / "configs/P4-integrated-V01.json"


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def verify_manifest(folder):
    for line in (folder / "SHA256.txt").read_text().splitlines():
        expected, relative = line.split("  ", 1)
        if digest(folder / relative) != expected:
            raise ValueError(f"Saved artifact changed: {relative}")


def run_pair(run_id, initial_file, render=False):
    initial_file = Path(initial_file).resolve()
    verify_manifest(initial_file.parent)
    config, initial = read_json(CONFIG_PATH), read_json(initial_file)
    for name, expected in initial["source_sha256"].items():
        if digest(PROJECT_ROOT / name) != expected:
            raise ValueError(f"Initial solver source identity changed: {name}")
    problem = read_json(initial_file.parent / "初始问题.json")
    if digest(initial_file.parent / "初始问题.json") != initial["problem_sha256"]:
        raise ValueError("Initial problem identity changed")
    initial_check = check_initial(config, problem, initial)
    # Before the final FRAMEWORK freeze, use the recoverable execution checkpoint.
    frozen = VERSIONS_ROOT / FRAMEWORK_VERSION / "源码配置_SHA256.txt"
    checkpoint = VERSIONS_ROOT / "备份/20261011_P4统一实现_V02_执行前/源码配置_SHA256.txt"
    guard = frozen if frozen.is_file() else checkpoint
    for line in guard.read_text().splitlines():
        expected, name = line.split("  ", 1)
        if digest(PROJECT_ROOT / name) != expected:
            raise ValueError(f"Execution checkpoint changed: {name}")
    folder = new_run_directory("P4", run_id)
    try:
        save(folder / "统一配置.json", config)
        save(folder / "初始分配引用.json", {"path": str(initial_file), "sha256": digest(initial_file),
                                          "problem_sha256": initial["problem_sha256"], "status": initial["status"]})
        checks, results, metrics = {}, {}, []
        for strategy in STRATEGIES:
            part = folder / strategy
            part.mkdir()
            stamp = time.perf_counter()
            result = simulate(config, initial["assignment"], strategy)
            result.update(initial_assignment_sha256=digest(initial_file), config_sha256=digest(CONFIG_PATH))
            result["run_metadata"] = {"framework": FRAMEWORK_VERSION, "stage": "P4-V01",
                                      "role": "teacher integrated execution; not learner acceptance",
                                      "execute_wall_s": time.perf_counter() - stamp,
                                      "python": sys.executable, "command": sys.argv}
            save(part / "结果.json", result)
            with (part / "实际控制与能量.jsonl").open("w", encoding="utf-8") as f:
                for row in result["steps"]:
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
            write_csv(part / "逐时通信.csv", result["links"])
            save(part / "事件与重规划.json", {"events": result["event_log"], "replans": result["replans"]})
            save(part / "指标.json", result["summary"])
            (part / "执行日志.txt").write_text("\n".join([
                f"Actual {strategy}; 0..{config['cutoff_time_s']}s; source snapshots in result JSON",
                *[f"t={e['time_s']} {e['event_id']} observed={e['observed_at_s']} processed={e['processed_at_s']}" for e in result["event_log"]],
                f"Final actual status={result['summary']['status']} completed={result['summary']['completed_task_count']}/{result['summary']['known_task_count']}",
                f"Team completion={result['summary']['team_completion_time_s']}s; cutoff={config['cutoff_time_s']}s; energy={result['summary']['total_energy_NEU']}NEU"]) + "\n")
            # Check the serialized artifact that will be rendered, not only an in-memory object.
            checks[strategy] = check_result(read_json(part / "结果.json"))
            save(part / "增量核查.json", checks[strategy])
            (part / "SHA256.txt").write_text(''.join(f"{digest(p)}  {p.name}\n" for p in sorted(part.iterdir()) if p.is_file()))
            results[strategy] = result
            s = result["summary"]
            metrics.append({"strategy": strategy, "status": s["status"], "completed": s["completed_task_count"],
                            "known_tasks": s["known_task_count"], "team_completion_time_s": s["team_completion_time_s"],
                            "total_energy_NEU": s["total_energy_NEU"], "cutoff_time_s": s["cutoff_time_s"]})
        first_event = min(e["time_s"] for e in config["events"])
        assert results[STRATEGIES[0]]["initial_queues"] == results[STRATEGIES[1]]["initial_queues"]
        assert [r for r in results[STRATEGIES[0]]["steps"] if r["time_s"] < first_event] == [r for r in results[STRATEGIES[1]]["steps"] if r["time_s"] < first_event]
        assert results[STRATEGIES[0]]["config"] == results[STRATEGIES[1]]["config"]
        assert all(r["summary"]["known_task_count"] == 22 for r in results.values())
        save(folder / "指标.json", metrics)
        save(folder / "统一核查.json", {"status": "PASS", "initial": initial_check, "groups": checks,
                                        "shared_config_initial_solution_currents_events_budgets": "PASS",
                                        "prefix_before_event_identical": "PASS", "single_changed_factor": "event_response_strategy",
                                        "both_denominators_include_arrivals": 22,
                                        "no_future_planner_data": [r["planner_config_keys"] for r in results.values()],
                                        "learner_operations_this_task": 0})
        save(folder / "组合探针.json", combination_probes(config))
        save(folder / "运行元数据.json", {"framework": FRAMEWORK_VERSION, "stage_version": "P4-V01", "role": "teacher",
                                          "python": sys.executable, "command": sys.argv, "status": "COMPLETE",
                                          "checkpoint": str(guard), "checkpoint_sha256": digest(guard),
                                          "source_sha256": {name: expected for expected, name in (line.split('  ', 1) for line in guard.read_text().splitlines())},
                                          "config_sha256": digest(CONFIG_PATH), "initial_sha256": digest(initial_file)})
        text = "# P4 统一实际对照\n\n| 策略 | 完成数 | 团队完成时间 s | 实际移动 NEU |\n|---|---|---|---|\n"
        for r in metrics:
            text += f"| {r['strategy']} | {r['completed']}/{r['known_tasks']} | {r['team_completion_time_s'] if r['team_completion_time_s'] is not None else 'null'} | {r['total_energy_NEU']:.6f} |\n"
        text += (f"\n实际观察[0,{config['cutoff_time_s']})，NEU含六艇全窗实际移动及完工后中继。截止不是完工；失败不证明全局不可行。\n"
                 f"\n仅事件响应策略不同；初始求解、容量6、流场、事件和预算相同。初始{initial['objective_m']}m是A*起点分配代理成本，不是完整航程最优。\n"
                 f"\n输出目录：`{folder}`。两组`结果.json`保存同一执行器产生的全部时间轴/控制/事件/服务/计划；师方运行不新增本人课堂验收。\n")
        (folder / "摘要.md").write_text(text, encoding="utf-8")
        (folder / "SHA256.txt").write_text(''.join(f"{digest(p)}  {p.relative_to(folder)}\n"
                                                  for p in sorted(folder.rglob('*')) if p.is_file() and p != folder / "SHA256.txt"))
        if render:
            from usv_uuc_cooperation.visualization.integrated_animation import render_pair
            render_pair(folder)
    except Exception as exc:
        save(folder / "失败.json", {"status": "FAILED", "error": str(exc), "output": str(folder)})
        raise
    return folder
