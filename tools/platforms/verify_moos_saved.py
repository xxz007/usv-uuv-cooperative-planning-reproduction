"""Reanalyse saved P5 MOOS logs without starting any platform or simulation."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import shutil
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from usv_uuc_cooperation.config import FRAMEWORK_VERSION, RESULTS_ROOT
from usv_uuc_cooperation.experiments.p5_moos import alog_evidence, save
from usv_uuc_cooperation.main import assert_frozen_core
from usv_uuc_cooperation.metrics.logger import digest


def main(source_run, analysis_id, output=None):
    assert_frozen_core()
    if not source_run.is_absolute():
        raise ValueError("--source-run must be an absolute saved-run directory")
    source_run = source_run.resolve(strict=True)
    if not re.fullmatch(r"[\w-]+", analysis_id):
        raise ValueError("--analysis-id must be a single name with letters, numbers, underscores or hyphens")
    folder = output if output is not None else RESULTS_ROOT / "p5_platforms" / "checks" / analysis_id
    if not folder.is_absolute():
        raise ValueError("--output must be an absolute new directory")
    if folder.is_relative_to(source_run):
        raise ValueError("Analysis output must be outside the original saved run")
    config = json.loads((source_run / "config.json").read_text())
    original_metadata = json.loads((source_run / "metadata.json").read_text())
    original_result = json.loads((source_run / "result.json").read_text())
    assignment = json.loads((source_run / "scheduler_assignment.json").read_text())
    chosen = next(t for t in config["tasks"] if t["id"] == assignment["trace"][0]["task"])
    source_sha = {str(p.relative_to(source_run)): digest(p)
                  for p in sorted(source_run.rglob("*")) if p.is_file()}
    folder.mkdir(parents=True, exist_ok=False)
    # Preserve the source bytes that matter to replay. Old derived results are
    # recorded by SHA and provenance rather than overwritten or reused as truth.
    derived = {"trajectory.csv", "result.json", "paired_metrics.json", "摘要.md"}
    copied = []
    for name, expected in source_sha.items():
        relative = Path(name)
        if relative.name in derived:
            continue
        target = folder / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_run / relative, target)
        if digest(target) != expected:
            raise RuntimeError(f"Saved-evidence copy SHA mismatch: {relative}")
        copied.append(name)
    save(folder / "source_result_before_reanalysis.json", original_result)
    start = [config["initial_pose"]["x_m"], config["initial_pose"]["y_m"]]
    groups = {}
    for case in ("example", "success", "timeout"):
        part = folder / case
        task_record = json.loads((part / "task.json").read_text())
        task = task_record["task"]
        goal = [60.0, -40.0] if case == "example" else task["goal_m"]
        feedback = json.loads((part / "feedback.json").read_text())
        evidence = alog_evidence(part, task["id"], start, goal, config["operating_envelope_m"])
        moved = math.hypot(evidence["last_pose"]["x_m"] - evidence["first_pose"]["x_m"],
                           evidence["last_pose"]["y_m"] - evidence["first_pose"]["y_m"])
        pose = evidence["completion_pose"]
        completed = (feedback["returncode"] == 0 and evidence["completion_time_s"] is not None
                     and pose is not None and pose["goal_distance_m"] < config["arrival_tolerance_m"]
                     and any("behavior-name=p5_task,completed" in row["value"]
                             for row in evidence["behavior_completion_messages"]))
        status = "EXAMPLE_MOVEMENT" if case == "example" and moved > 1 else "COMPLETED" if completed else "TIMEOUT"
        evidence.update(task_id=task["id"], status=status, completed=completed,
                        movement_displacement_m=moved, goal_m=goal, query_returncode=feedback["returncode"],
                        executed_framework=original_metadata["framework"], analyzed_framework=FRAMEWORK_VERSION,
                        simulation_runs=0, evidence_source="copied original raw alog and feedback, verified by SHA")
        save(part / "result.json", evidence)
        groups[case] = evidence
    success, timed_out = groups["success"], groups["timeout"]
    rejected = json.loads((folder / "invalid_target.json").read_text())
    with (folder / "reference_2d.csv").open(newline="", encoding="utf-8") as stream:
        reference = list(csv.DictReader(stream))
    paired = json.loads((source_run / "paired_metrics.json").read_text())
    paired.update(executed_framework=original_metadata["framework"], analyzed_framework=FRAMEWORK_VERSION,
                  simulation_runs=0, reference_2d_sha256=digest(folder / "reference_2d.csv"),
                  reference_2d_reused_byte_identical=True)
    paired["moos"] = {"completion_time_s": success["completion_time_s"],
                      "max_cross_track_m": success["maximum_cross_track_m"],
                      "operating_envelope_violations": success["operating_envelope_violations"],
                      "actual_distance_until_completion_m": success["actual_distance_m"]}
    save(folder / "paired_metrics.json", paired)
    source_after_sha = {str(p.relative_to(source_run)): digest(p)
                        for p in sorted(source_run.rglob("*")) if p.is_file()}
    source_unchanged = source_sha == source_after_sha
    checks = {
        "example_actual_movement": groups["example"]["status"] == "EXAMPLE_MOVEMENT",
        "scheduler_task_actual_completion": success["completed"] and success["task_id"] == chosen["id"],
        "timeout_no_false_completion": not timed_out["completed"] and timed_out["complete_alog_time_s"] is None
                                     and timed_out["query_returncode"] == 1 and timed_out["dispatch_alog_time_s"] is not None,
        "invalid_goal_rejected_before_publish": rejected["status"] == "REJECTED" and not rejected["completed"]
                                               and not rejected["published_to_moos"],
        "actual_trajectory_recorded": all(group["sample_count"] > 2 for group in groups.values()),
        "operating_envelope": success["operating_envelope_violations"] == 0,
        "all_original_owned_processes_stopped": all(json.loads((folder / case / "owned_processes.json").read_text())["all_stopped"]
                                                    for case in groups),
        "reference_2d_reused_byte_identical": digest(folder / "reference_2d.csv") == source_sha["reference_2d.csv"],
        "source_run_all_bytes_unchanged": source_unchanged,
    }
    result = {
        "engineering_status": "PASS" if all(checks.values()) else "FAIL",
        "executed_framework": original_metadata["framework"], "analyzed_framework": FRAMEWORK_VERSION,
        "operator_role": original_metadata["operator_role"], "analysis_operator_role": "teacher",
        "simulation_runs": 0, "solver_runs": 0, "moos_process_launches": 0,
        "source_run": str(source_run), "checks": checks,
        "completed_task_count": int(success["completed"]), "completion_time_s": success["completion_time_s"],
        "maximum_cross_track_m": success["maximum_cross_track_m"],
        "timeout_task_status": timed_out["status"], "invalid_task_status": rejected["status"],
        "learner_course_status": "REQUIRES_SEPARATE_USER_EXECUTION_AND_EXPLANATION_RECORD",
        "vrx_status": "NOT_EXECUTED",
        "analysis_change": "Strip alog value whitespace at the owning read boundary, then re-extract matching task dispatch/completion events",
    }
    save(folder / "result.json", result)
    save(folder / "source_SHA256.json", {
        "source_run": str(source_run), "source_files": len(source_sha), "copied_files_sha_verified": len(copied),
        "source_all_bytes_unchanged": source_unchanged, "simulation_runs": 0,
        "files": [{"path": name, "sha256_before": expected,
                   "sha256_after": source_after_sha.get(name), "copied": name in copied}
                  for name, expected in source_sha.items()],
    })
    completion = success["completion_time_s"]
    lines = ["# P5 保存日志复核", "",
             f"工程核查：**{result['engineering_status']}**。实际执行版本 **{original_metadata['framework']}**；保存日志分析版本 **{FRAMEWORK_VERSION}**。", "",
             "本次重新启动平台、模拟、求解器均为 **0 次**。原运行全部文件已核对 SHA，原字节保持；二维参考 CSV 按字节沿用。", "",
             "| 核心读数 | 结果 |", "|---|---:|",
             f"| 完成任务数 | {int(success['completed'])}/1 |",
             f"| MOOS 派发至完成时间 / 模拟秒 | {completion if completion is not None else 'null'} |",
             f"| 派发至完成窗口最大横向偏差 / m | {success['maximum_cross_track_m']:.6f} |", "",
             "| 保存案例 | 重新判定 |", "|---|---|",
             f"| MOOS 原例实际运动 | {groups['example']['status']} |",
             f"| 分配器任务 {chosen['id']} | {success['status']} |",
             f"| 短反馈等待窗口 | {timed_out['status']}，未误报完成 |",
             f"| 无效目标 | {rejected['status']}，未发送至平台 |", "",
             f"二维参考完成时间沿用 {float(reference[-1]['time_s'])} 秒；MOOS 包含实际转向与速度响应，两者不宣称动力学相同。", "",
             "成功依据同一任务 ID 的派发、pHelmIvP 行为完成和实际 NAV 位置。原始 alog 的值末尾带空格；旧分析漏读精确 ID，新分析在读取边界去除空白后恢复事件关联。", "",
             "[结构化核查](result.json) · [配对指标](paired_metrics.json) · [原始文件 SHA 与复制核查](source_SHA256.json) · [旧分析结果](source_result_before_reanalysis.json)", "",
             "[成功任务原始反馈](success/feedback.json) · [成功任务重建轨迹](success/trajectory.csv) · [成功任务重建结果](success/result.json)", "",
             "教师机械核查、本人实际操作及本人解释分别记录。此处不新增本人验收；VRX 未执行。",
             "短等待为 uQueryDB 自身启动后的 MOOS 时间观察窗，不是自派发时刻起的严格端到端截止。", ""]
    (folder / "摘要.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"output": str(folder), "summary": str(folder / "摘要.md"),
                      "engineering_status": result["engineering_status"], "simulation_runs": 0,
                      "source_files_sha_verified": len(source_sha), "source_all_bytes_unchanged": source_unchanged}, ensure_ascii=False))
    if not all(checks.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--analysis-id", required=True)
    parser.add_argument("--output", type=Path, help="Optional absolute new analysis directory; defaults to results/p5_platforms/checks/ID")
    args = parser.parse_args()
    main(args.source_run, args.analysis_id, args.output)
