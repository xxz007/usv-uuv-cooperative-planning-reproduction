"""Independently audit saved P5 VRX CSVs; never start ROS or a simulation.

Usage: python3 -B THIS_FILE --source-run /absolute/saved/run --output /absolute/new/check
Only the new output directory is written. A PASS verifies the saved sampled
evidence and its arithmetic, not lossless transport, Gazebo command acceptance,
the deployed upstream build, learner understanding, or complete P5 acceptance.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def number(value):
    if isinstance(value, bool):
        raise ValueError("A numeric field contains a boolean")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("A numeric field is not finite")
    return value


def equal_number(left, right):
    if left is None or right is None:
        return left is right
    return math.isclose(number(left), number(right), rel_tol=1e-7, abs_tol=1e-6)


def csv_rows(path, required):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = set(required) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path.name}: missing columns {sorted(missing)}")
        yield from reader


def geometry(x, y, common, envelope):
    yaw = number(common["start"]["yaw_rad"])
    dx, dy = x-number(common["start"]["x_m"]), y-number(common["start"]["y_m"])
    along = dx*math.cos(yaw)+dy*math.sin(yaw)
    signed_cross = -dx*math.sin(yaw)+dy*math.cos(yaw)
    distance = math.hypot(x-number(common["goal"]["x_m"]), y-number(common["goal"]["y_m"]))
    outside = not (envelope["along_m"][0] <= along <= envelope["along_m"][1]
                   and envelope["cross_m"][0] <= signed_cross <= envelope["cross_m"][1])
    return {"goal_distance_m": distance, "along_m": along,
            "cross_track_m": abs(signed_cross), "outside_envelope": outside}


def run(source_run, output):
    source, out = Path(source_run), Path(output)
    if not source.is_absolute() or not out.is_absolute():
        raise ValueError("Both paths must be absolute")
    source, out = source.resolve(), out.resolve()
    if not source.is_dir() or out.exists() or source == out or source in out.parents:
        raise ValueError("Source must exist; output must be new and outside source")
    project = Path(__file__).resolve().parents[3]
    controller = project / "usv_uuc_cooperation/experiments/p5_vrx.py"
    active_config = project / "usv_uuc_cooperation/configs/P5-VRX-V01.json"
    tracked = {f"run/{path.name}": path for path in sorted(source.iterdir()) if path.is_file()}
    for name, path in (("controller", controller), ("active_config", active_config)):
        if path.is_file():
            tracked[name] = path
    before = {name: {"sha256": sha256(path), "bytes": path.stat().st_size}
              for name, path in tracked.items()}
    out.mkdir(parents=True, exist_ok=False)
    checks, issues = [], {}
    metrics = [{"system": system, "completion_time_s": None, "maximum_cross_track_m": None,
                "boundary_violation_events": 0} for system in ("VRX", "ideal_2D")]
    windows, counts, provenance = {}, {}, {}

    def check(name, ok, detail=None, incomplete=False):
        checks.append({"name": name, "status": "PASS" if ok else "INCOMPLETE" if incomplete else "FAIL",
                       "detail": detail})

    def issue(group, detail):
        entry = issues.setdefault(group, {"count": 0, "examples": []})
        entry["count"] += 1
        if len(entry["examples"]) < 5:
            entry["examples"].append(detail)

    required = ("metadata.json", "frozen_config.json", "common_inputs.json", "clock.csv",
                "odometry.csv", "commands.csv", "reference_2d.csv", "summary.json")
    missing = [name for name in required if not (source / name).is_file()]
    check("required_saved_inputs", not missing, {"missing": missing}, incomplete=True)
    try:
        if not missing:
            config = json.loads((source / "frozen_config.json").read_text(encoding="utf-8"))
            metadata = json.loads((source / "metadata.json").read_text(encoding="utf-8"))
            common = json.loads((source / "common_inputs.json").read_text(encoding="utf-8"))
            summary = json.loads((source / "summary.json").read_text(encoding="utf-8"))
            task, startup, envelope = config["task"], config["startup"], config["operating_envelope"]
            dispatch = number(common["dispatch_clock_s"])
            budget, radius, hold_required = (number(task[key]) for key in
                                             ("sim_budget_s", "arrival_radius_m", "arrival_hold_sim_s"))
            gap_limit = number(startup["maximum_observation_gap_sim_s"])
            config_hash = before["run/frozen_config.json"]["sha256"]
            check("frozen_config_sha", metadata["config_sha256"] == common["config_sha256"] == config_hash)
            check("controller_sha", "controller" in before
                  and metadata["script_sha256"] == common["script_sha256"] == before["controller"]["sha256"],
                  "Compared with this project's actual controller bytes; deployment identity needs separate launch evidence")
            check("active_config_sha", "active_config" in before and before["active_config"]["sha256"] == config_hash)
            check("source_id", config["source_id"] == common["source_id"] == metadata["source_id"] == summary["source_id"])
            check("common_task_and_criteria", common["task_id"] == task["id"] and common["task"] == task
                  and common["operating_envelope"] == envelope and common["reference_model"] == config["reference"])
            check("metadata_identity", metadata["framework_version"] == summary["framework_version"]
                  and metadata["role"] == summary["role"] and metadata["topics"] == config["topics"]
                  and metadata["config_version"] == config["config_version"])
            yaw = number(common["start"]["yaw_rad"])
            forward = [math.cos(yaw), math.sin(yaw)]
            check("goal_and_forward_from_measured_start", all(equal_number(a, b) for a, b in zip(forward, common["forward_unit"]))
                  and len(common["forward_unit"]) == 2
                  and all(equal_number(common["goal"][axis], number(common["start"][axis])+task["forward_distance_m"]*forward[i])
                          for i, axis in enumerate(("x_m", "y_m"))))
            age = dispatch-number(common["start"]["odom_stamp_s"])
            check("dispatch_time_origin_and_initial_age", equal_number(age, common["initial_odometry_age_s"])
                  and -0.1 <= age <= startup["maximum_observation_age_sim_s"])
            provenance = {"source_id": config["source_id"], "framework_version": metadata["framework_version"],
                          "script_sha256": metadata["script_sha256"], "config_sha256": config_hash,
                          "dispatch_clock_s": dispatch, "initial_odometry_age_s": age,
                          "upstream_build_verification": "NOT_VERIFIED_BY_THIS_SAVED_LOG_CHECKER"}
            clocks, prior_wall, prior_clock = [], -math.inf, None
            for index, row in enumerate(csv_rows(source / "clock.csv", ("wall_elapsed_s", "phase", "sec", "nanosec", "clock_s")), 1):
                wall, stamp = number(row["wall_elapsed_s"]), number(row["clock_s"])
                if wall < prior_wall or wall < 0 or not equal_number(stamp, number(row["sec"])+number(row["nanosec"])*1e-9):
                    issue("clock_fields", f"row {index}: wall order or sec/nanosec disagrees")
                if row["phase"] == "task" and prior_clock is not None and stamp < prior_clock-1e-9:
                    issue("clock_fields", f"row {index}: task clock moved backwards")
                clocks.append((wall, stamp, row["phase"]))
                prior_wall, prior_clock = wall, stamp
            counts["clock"] = len(clocks)
            check("dispatch_clock_received", any(equal_number(t, dispatch) for _, t, _ in clocks), incomplete=True)
            command_rows, shutdown_wall, prior_wall = [], None, -math.inf
            for index, row in enumerate(csv_rows(source / "commands.csv", ("wall_elapsed_s", "phase", "clock_s", "elapsed_sim_s",
                                                                          "odom_stamp_s", "left_thrust_n", "right_thrust_n", "reason")), 1):
                wall, left, right = number(row["wall_elapsed_s"]), number(row["left_thrust_n"]), number(row["right_thrust_n"])
                if wall < prior_wall or not all(config["controller"]["minimum_thrust_n"] <= thrust <= config["controller"]["maximum_thrust_n"] for thrust in (left, right)):
                    issue("commands", f"row {index}: wall order or thrust bounds")
                if row["elapsed_sim_s"] and not equal_number(row["elapsed_sim_s"], number(row["clock_s"])-dispatch):
                    issue("commands", f"row {index}: elapsed time is not clock minus dispatch")
                if row["phase"] == "shutdown" and shutdown_wall is None:
                    shutdown_wall = wall
                command_rows.append(row)
                prior_wall = wall
            counts["commands"] = len(command_rows)
            check("owned_node_zero_publications", len(command_rows) >= 3
                  and all(row["phase"] == "shutdown" and row["reason"] == "finally_stop"
                          and number(row["left_thrust_n"]) == number(row["right_thrust_n"]) == 0 for row in command_rows[-3:]),
                  "Publish records only; not an acknowledgement from Gazebo", incomplete=True)
            check("task_thrust_publish_evidence", any(row["phase"] == "task"
                  and (number(row["left_thrust_n"]) != 0 or number(row["right_thrust_n"]) != 0) for row in command_rows), incomplete=True)
            vrx, last_pose, prior_wall = metrics[0], None, -math.inf
            hold_start, terminal, was_outside, anchor = None, None, False, None
            used, excluded, odom_count = 0, {"pre_dispatch": 0, "shutdown_or_waiting": 0, "unusable": 0}, 0
            first_task_wall, last_task_stamp, last_task_wall, failed_gap = None, None, None, None
            for index, row in enumerate(csv_rows(source / "odometry.csv", ("wall_elapsed_s", "phase", "clock_s", "stamp_s", "stamp_sec", "stamp_nanosec",
                    "frame_id", "child_frame_id", "x_m", "y_m", "qx", "qy", "qz", "qw", "yaw_rad", "usable",
                    "elapsed_sim_s", "goal_distance_m", "along_m", "cross_track_m", "outside_envelope")), 1):
                odom_count += 1
                wall = number(row["wall_elapsed_s"])
                if wall < prior_wall:
                    issue("odometry_fields", f"row {index}: wall time moved backwards")
                prior_wall = wall
                if row["usable"] not in ("True", "False"):
                    raise ValueError(f"odometry row {index}: invalid usable flag")
                if row["usable"] == "False":
                    excluded["unusable"] += 1
                    continue
                stamp, x, y = number(row["stamp_s"]), number(row["x_m"]), number(row["y_m"])
                q = [number(row[key]) for key in ("qx", "qy", "qz", "qw")]
                norm = math.sqrt(sum(v*v for v in q))
                if not math.isfinite(norm) or norm < 1e-9:
                    raise ValueError(f"odometry row {index}: usable invalid quaternion")
                qx, qy, qz, qw = [v/norm for v in q]
                observed_yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                if not equal_number(stamp, number(row["stamp_sec"])+number(row["stamp_nanosec"])*1e-9) or not equal_number(observed_yaw, row["yaw_rad"]):
                    issue("odometry_fields", f"row {index}: native timestamp or quaternion-derived yaw disagrees")
                gap = stamp-last_pose["stamp"] if last_pose else None
                if gap is not None and gap <= 0:
                    issue("odometry_fields", f"row {index}: usable stamp did not advance")
                pose = {"stamp": stamp, "x": x, "y": y, "yaw": observed_yaw, "frame": row["frame_id"], "child": row["child_frame_id"]}
                last_pose = pose
                elapsed = stamp-dispatch
                measures = geometry(x, y, common, envelope)
                if row["elapsed_sim_s"]:
                    if not equal_number(row["elapsed_sim_s"], elapsed):
                        issue("odometry_fields", f"row {index}: elapsed time is not stamp minus dispatch")
                    for key, value in measures.items():
                        matches = (row[key] in ("True", "False") and (row[key] == "True") == value) if isinstance(value, bool) else equal_number(row[key], value)
                        if not matches:
                            issue("logged_geometry", f"odometry row {index}: {key}")
                if row["phase"] == "waiting_input":
                    if equal_number(stamp, common["start"]["odom_stamp_s"]):
                        anchor = pose
                if row["phase"] != "task":
                    excluded["shutdown_or_waiting"] += 1
                    continue
                if elapsed < 0:
                    excluded["pre_dispatch"] += 1
                    continue
                if pose["frame"] != common["coordinate_frame"] or pose["child"] != common["child_frame"]:
                    issue("odometry_fields", f"row {index}: task frame changed")
                if not -0.1 <= number(row["clock_s"])-stamp <= startup["maximum_observation_age_sim_s"]:
                    issue("odometry_fields", f"row {index}: task pose is outside the frozen clock-age limit")
                if gap is not None and gap > gap_limit:
                    failed_gap = {"row": index, "gap_sim_s": gap}
                    terminal = terminal or "FEEDBACK_GAP"
                    continue
                if terminal is not None:
                    issue("task_window", f"odometry row {index}: extra task observation after {terminal}")
                    continue
                if elapsed > budget+1e-9 or wall > task["wall_budget_s"] or (shutdown_wall is not None and wall >= shutdown_wall):
                    issue("task_window", f"odometry row {index}: outside task time/phase window")
                    continue
                used += 1
                first_task_wall = wall if first_task_wall is None else first_task_wall
                last_task_stamp, last_task_wall = stamp, wall
                vrx["maximum_cross_track_m"] = max(vrx["maximum_cross_track_m"] or 0.0, measures["cross_track_m"])
                if measures["outside_envelope"] and not was_outside:
                    vrx["boundary_violation_events"] += 1
                was_outside = measures["outside_envelope"]
                hold_start = (stamp if hold_start is None else hold_start) if measures["goal_distance_m"] <= radius+1e-9 else None
                if hold_start is not None and stamp-hold_start >= hold_required-1e-9:
                    vrx["completion_time_s"], terminal = elapsed, "COMPLETED"
                elif was_outside and envelope["stop_on_violation"]:
                    terminal = "BOUNDARY_STOP"
            counts["odometry"] = odom_count
            check("usable_post_dispatch_odometry", used > 0, {"used": used, "excluded": excluded}, incomplete=True)
            check("observed_hold_gap_limit", failed_gap is None, failed_gap, incomplete=True)
            check("native_start_pose", anchor is not None and all(equal_number(anchor[key], common["start"][field])
                  for key, field in (("x", "x_m"), ("y", "y_m"), ("yaw", "yaw_rad")))
                  and anchor["frame"] == common["coordinate_frame"] and anchor["child"] == common["child_frame"], incomplete=True)
            windows["VRX"] = {"dispatch_clock_s": dispatch, "first_task_wall_s": first_task_wall,
                               "last_included_odom_stamp_s": last_task_stamp, "last_included_wall_s": last_task_wall,
                               "terminal_observed": terminal,
                               "shutdown_first_publish_wall_s": shutdown_wall, "valid_rows": used, "excluded_rows": excluded}
            reference, previous, hold_start, was_outside, terminal = metrics[1], None, None, False, None
            reference_count = 0
            sample_dt = number(config["reference"]["sample_dt_s"])
            for index, row in enumerate(csv_rows(source / "reference_2d.csv", ("elapsed_sim_s", "x_m", "y_m", "yaw_rad", "speed_mps",
                    "goal_distance_m", "along_m", "cross_track_m", "outside_envelope", "inside_hold_s")), 1):
                reference_count += 1
                t, x, y = number(row["elapsed_sim_s"]), number(row["x_m"]), number(row["y_m"])
                measures = geometry(x, y, common, envelope)
                if terminal is not None or t < 0 or t > budget+1e-9:
                    issue("reference_fields", f"row {index}: outside reference task window")
                    continue
                if previous is None:
                    if not equal_number(t, 0) or not equal_number(x, common["start"]["x_m"]) or not equal_number(y, common["start"]["y_m"]):
                        issue("reference_fields", "reference initial time/position differs from common input")
                else:
                    dt = t-previous["t"]
                    step = min(previous["speed"]*sample_dt, max(0.0, previous["distance"]-radius))
                    if not equal_number(dt, sample_dt) or dt > gap_limit+1e-9 or not all(equal_number(value, previous[key]+step*forward[i]) for i, (key, value) in enumerate((("x", x), ("y", y)))):
                        issue("reference_fields", f"row {index}: frozen sample interval or recorded kinematic transition disagrees")
                inside = measures["goal_distance_m"] <= radius+1e-9
                hold_start = (t if hold_start is None else hold_start) if inside else None
                hold = t-hold_start if hold_start is not None else 0.0
                speed = 0.0 if inside else number(task["target_speed_mps"])
                if not equal_number(row["yaw_rad"], yaw) or not equal_number(row["speed_mps"], speed) or not equal_number(row["inside_hold_s"], hold):
                    issue("reference_fields", f"row {index}: yaw, nominal speed or sampled hold disagrees")
                for key, value in measures.items():
                    matches = (row[key] in ("True", "False") and (row[key] == "True") == value) if isinstance(value, bool) else equal_number(row[key], value)
                    if not matches:
                        issue("logged_geometry", f"reference row {index}: {key}")
                reference["maximum_cross_track_m"] = max(reference["maximum_cross_track_m"] or 0.0, measures["cross_track_m"])
                if measures["outside_envelope"] and not was_outside:
                    reference["boundary_violation_events"] += 1
                was_outside = measures["outside_envelope"]
                previous = {"t": t, "x": x, "y": y, "speed": speed, "distance": measures["goal_distance_m"]}
                if inside and hold >= hold_required-1e-9:
                    reference["completion_time_s"], terminal = t, "COMPLETED"
                elif was_outside and envelope["stop_on_violation"]:
                    terminal = "BOUNDARY_STOP"
            counts["reference_2d"] = reference_count
            last_budget_sample = math.floor((budget+1e-9)/sample_dt)*sample_dt
            check("reference_full_terminal_window", previous is not None and (terminal is not None or equal_number(previous["t"], last_budget_sample)), incomplete=True)
            windows["ideal_2D"] = {"elapsed_zero_s": 0.0, "last_included_elapsed_s": previous["t"] if previous else None,
                                    "terminal_observed": terminal, "logged_rows": reference_count}
            reported = {row["system"]: row for row in summary["metrics"]}
            check("two_reported_systems", len(summary["metrics"]) == 2 and set(reported) == {"VRX", "ideal_2D"})
            for metric in metrics:
                name = metric["system"]
                row = reported[name]
                check(f"{name}_three_metrics", all(equal_number(metric[key], row[key]) for key in
                      ("completion_time_s", "maximum_cross_track_m"))
                      and type(row["boundary_violation_events"]) is int
                      and metric["boundary_violation_events"] == row["boundary_violation_events"],
                      {"recomputed": metric, "reported": row})
                check(f"{name}_completion_status_and_null", (row["status"] == "COMPLETED") == (metric["completion_time_s"] is not None)
                      and (row["completion_time_s"] is None if metric["completion_time_s"] is None else True))
            check("summary_task_completed", summary["task_completed"] is (metrics[0]["completion_time_s"] is not None))
            check("csv_row_counts", counts == summary["received_or_published_rows"], {"recounted": counts})
            if reported["VRX"]["status"] == "SIM_TIMEOUT":
                check("simulation_timeout_clock_witness", any(phase == "task" and t-dispatch >= budget-1e-9 for _, t, phase in clocks))
                final_task_clock = max((t for _, t, phase in clocks if phase == "task"), default=None)
                check("simulation_timeout_feedback_coverage", final_task_clock is not None and last_task_stamp is not None
                      and -0.1 <= final_task_clock-last_task_stamp <= startup["maximum_observation_age_sim_s"], incomplete=True)
            for group in ("clock_fields", "commands", "odometry_fields", "logged_geometry", "task_window", "reference_fields"):
                check(group, group not in issues, issues.get(group))
    except (OSError, ValueError, KeyError, TypeError, csv.Error) as exc:
        check("saved_input_parse_and_schema", False, f"{type(exc).__name__}: {exc}")
    after = {name: {"sha256": sha256(path), "bytes": path.stat().st_size} if path.is_file() else None
             for name, path in tracked.items()}
    check("source_bytes_unchanged", before == after)
    check("source_file_inventory_unchanged", set(name[4:] for name in tracked if name.startswith("run/"))
          == {path.name for path in source.iterdir() if path.is_file()})
    status = "FAIL" if any(row["status"] == "FAIL" for row in checks) else "INCOMPLETE" if any(row["status"] == "INCOMPLETE" for row in checks) else "PASS"
    report = {"status": status, "source_run": str(source), "output": str(out),
              "checker_sha256": sha256(Path(__file__).resolve()), "checks": checks, "metrics": metrics,
              "windows": windows, "row_counts": counts, "provenance": provenance,
              "source_hashes_before": before, "source_hashes_after": after,
              "definition": "Received usable post-dispatch task samples through the first observed terminal event; shutdown excluded. Hold spans sampled poses with a bounded gap, not proof of continuous or lossless observation.",
              "scope": "Saved-log arithmetic only; no simulation, ROS commands, upstream deployment verification, collision/energy/hardware claim or learner acceptance."}
    (out / "checks.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    with (out / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    lines = ["# P5 VRX 保存日志独立复核", "", f"复核状态：{status}。", "",
             "| 系统 | 完成时间（模拟秒） | 最大横向偏差（米） | 包络违规事件 |", "|---|---:|---:|---:|"]
    for metric in metrics:
        values = ["null" if metric[key] is None else str(metric[key]) for key in
                  ("completion_time_s", "maximum_cross_track_m", "boundary_violation_events")]
        lines.append(f"| {metric['system']} | {' | '.join(values)} |")
    lines += ["", "只统计派发后有效任务采样，关停尾段排除；未完成的完成时间为 null。保持判据采用连续收到的采样及限定最大间隔，不声称连续无损。",
              "", f"原始运行：{source}", "", "逐项检查、时间窗口与原文件前后 SHA-256 见 checks.json。原始运行文件未写入。"]
    failed = [row for row in checks if row["status"] != "PASS"]
    if failed:
        lines += ["", "需要处理："]+[f"- {row['status']}：{row['name']}；{row['detail']}" for row in failed]
    (out / "摘要.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    print(out / "摘要.md")
    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    raise SystemExit(run(arguments.source_run, arguments.output))
