"""Read-only audit of native P5 original-example observations; no ROS/simulation."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1048576), b""):
            value.update(chunk)
    return value.hexdigest()


def number(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Nonfinite numeric evidence")
    return value


def close(left, right):
    return left is right if left is None or right is None else math.isclose(number(left), number(right), abs_tol=1e-6, rel_tol=1e-7)


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def run(source_run, output):
    source, out = Path(source_run), Path(output)
    if not source.is_absolute() or not out.is_absolute():
        raise ValueError("Both paths must be absolute")
    source, out = source.resolve(), out.resolve()
    if not source.is_dir() or out.exists() or source == out or source in out.parents:
        raise ValueError("Existing source and a new output outside it are required")
    project = Path(__file__).resolve().parents[3]
    helper = project / "课时资料/P5/源码/VRX_官方原例记录器_V02.py"
    wrapper = project / "usv_uuc_cooperation/experiments/p5_vrx_platform.py"
    config_path = project / "usv_uuc_cooperation/configs/P5-VRX-V01.json"
    controller = project / "usv_uuc_cooperation/experiments/p5_vrx.py"
    launch_path = source.parent / "launch_metadata.json"
    paths = {"run/"+p.name: p for p in sorted(source.iterdir()) if p.is_file()}
    paths.update({name: p for name, p in (("launch", launch_path), ("helper", helper), ("wrapper", wrapper), ("config", config_path), ("controller", controller)) if p.is_file()})
    before = {name: digest(path) for name, path in paths.items()}
    out.mkdir(parents=True, exist_ok=False)
    checks, result = [], {}

    def check(name, ok, detail=None, incomplete=False):
        checks.append({"name": name, "status": "PASS" if ok else "INCOMPLETE" if incomplete else "FAIL", "detail": detail})

    required = ("metadata.json", "summary.json", "model_pose.csv", "commands.csv", "clock.csv", "image.csv", "first_image_header.json", "first_image.data.bin")
    missing = [name for name in required if not (source / name).is_file()]+([] if launch_path.is_file() else [str(launch_path)])
    check("required_inputs", not missing, {"missing": missing}, incomplete=True)
    try:
        if not missing:
            meta, summary, launch, image, config = [json.loads(path.read_text(encoding="utf-8")) for path in
                (source / "metadata.json", source / "summary.json", launch_path, source / "first_image_header.json", config_path)]
            poses, commands, clocks, images = [rows(source / name) for name in ("model_pose.csv", "commands.csv", "clock.csv", "image.csv")]
            check("source_completed", summary["recording_complete"] is True and summary["status"] == "RECORDING_COMPLETE" and launch["status"] == "COMPLETED" and launch["child_returncode"] == 0, incomplete=True)
            check("role_and_child_summary", meta["role"] == summary["role"] == launch["role"] and meta["role"] in ("teacher", "learner") and launch["child_summary"] == summary)
            tools = launch["tools_sha256"]
            check("actual_helper_and_wrapper_sha", before["helper"] == meta["script_sha256"] == summary["script_sha256"] == tools[helper.name] and before["wrapper"] == launch["platform_script_sha256"])
            check("config_and_controller_sha", before["config"] == tools[config_path.name] and before["controller"] == tools[controller.name])
            pin = config["source_id"].rsplit(":", 1)[-1]
            check("wrapper_source_guard", launch["mode"] == "original" and launch["declared_source_id"] == config["source_id"] and launch["expected_commit"] == launch["upstream_commit"] == pin and launch["upstream_tracked_clean"] is True and not any(not line.startswith("?? ") for line in launch["upstream_porcelain"].splitlines()))
            cleanup = launch["owned_process_cleanup"]
            check("owned_groups_closed", {"launch", "readiness", "recorder"} <= {row["label"] for row in cleanup} and all(row["process_group_remaining"] is False for row in cleanup), incomplete=True)
            check("received_row_counts", summary["counts"] == {"clock": len(clocks), "model_pose": len(poses), "commands": len(commands), "image": len(images)})
            clock_values = [number(row["clock_s"]) for row in clocks]
            check("native_clock_fields", bool(clocks) and all(close(row["clock_s"], number(row["sec"])+number(row["nanosec"])*1e-9) for row in clocks) and all(b >= a for a, b in zip(clock_values, clock_values[1:])), incomplete=not clocks)
            nonzero = [row for row in commands if number(row["left_thrust_n"]) != 0 or number(row["right_thrust_n"]) != 0]
            baseline = [row for row in commands if row["reason"] == "zero_baseline"]
            stops = [row for row in commands if row["reason"] == "eight_sim_seconds_thrust_finished"]
            fallback = [row for row in commands if row["reason"] == "finally_zero"]
            start = number(baseline[0]["clock_s"]) if baseline else None
            thrust = number(nonzero[0]["clock_s"]) if nonzero else None
            stop = number((stops or fallback)[0]["clock_s"]) if (stops or fallback) and start is not None else None
            check("publication_boundaries", close(start, summary["protocol_start_clock_s"]) and close(thrust, summary["thrust_start_clock_s"]) and close(stop if thrust is not None else None, summary["thrust_stop_clock_s"]) and close(stop, summary["protocol_stop_clock_s"]))
            check("complete_protocol_evidence", bool(baseline and nonzero and stops), incomplete=True)
            check("zero_baseline_and_100N", all(number(row["left_thrust_n"]) == number(row["right_thrust_n"]) == (100 if row in nonzero else 0) for row in commands) and all(row["reason"] == "equal_thrust" for row in nonzero) and len(nonzero) == summary["nonzero_publication_pairs"])
            check("final_zero_publications", len(commands) >= 3 and all(row["reason"] == "finally_zero" and row["phase"] == "shutdown" and number(row["left_thrust_n"]) == number(row["right_thrust_n"]) == 0 for row in commands[-3:]), incomplete=True)
            max_step = max([b-a for a, b in zip(clock_values, clock_values[1:])] or [0])
            check("3s_baseline_8s_thrust", start is not None and thrust is not None and stop is not None and 3-1e-6 <= thrust-start <= 3+max_step+1e-6 and 8-1e-6 <= stop-thrust <= 8+max_step+1e-6, {"zero_baseline_s": None if thrust is None or start is None else thrust-start, "equal_thrust_s": None if thrust is None or stop is None else stop-thrust}, incomplete=not (baseline and nonzero and stops))
            check("frozen_protocol", meta["protocol"]["zero_baseline_sim_s"] == 3 and meta["protocol"]["equal_thrust_sim_s"] == 8 and meta["protocol"]["each_thrust_n"] == 100)
            native, ids, identity_ok, header_ok = [], set(), True, True
            for row in poses:
                identity_ok &= row["entity_name"] == row["child_frame"] == meta["model_name_exact_match"] == summary["model_name_exact_match"] == "wamv"
                ids.add(int(row["entity_id"]))
                header = json.loads(row["native_header_json"]).get("stamp", {})
                header_ok &= int(header.get("sec", 0)) == int(row["stamp_sec"]) and int(header.get("nsec", 0)) == int(row["stamp_nanosec"])
                if row["usable"] != "True":
                    continue
                values = {key: number(row[key]) for key in ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")}
                if sum(values[key]**2 for key in ("qx", "qy", "qz", "qw")) <= 1e-12:
                    raise ValueError("Usable native sample has an invalid quaternion")
                native.append({**row, **values, "native_stamp_s": int(row["stamp_sec"])+int(row["stamp_nanosec"])*1e-9, "entity_id": int(row["entity_id"])})
            check("native_name_id_header", bool(poses) and identity_ok and header_ok and len(ids) == 1 and next(iter(ids)) > 0 and meta["pose_transport"] == summary["pose_transport"] == "gz.transport13 native Pose_V; no TF conversion", {"entity_ids": sorted(ids)}, incomplete=not poses)
            ready = launch["observed_readiness"]
            check("readiness_native_and_image", ready["ready"] is True and ready["clock_samples"] >= 5 and ready["feedback"]["entity_name"] == "wamv" and ready["feedback"]["entity_id"] in ids and meta["image_topic"] == ready["image"]["topic"] and all(ready["image"][key] == image[key] for key in ("width", "height", "encoding", "step", "data_bytes")))
            check("usable_native_samples", bool(native) and len(native) == len(poses), incomplete=True)
            intervals = {"zero_baseline": (start, thrust if thrust is not None else stop), "equal_thrust": (thrust, stop)}
            for phase, (begin, end) in intervals.items():
                selected = [row for row in native if begin is not None and end is not None and begin <= row["native_stamp_s"] < end]
                first, last = (min(selected, key=lambda row: row["native_stamp_s"]), max(selected, key=lambda row: row["native_stamp_s"])) if selected else (None, None)
                horizontal = math.hypot(last["x_m"]-first["x_m"], last["y_m"]-first["y_m"]) if len(selected) >= 2 else None
                spatial = math.sqrt(sum((last[key]-first[key])**2 for key in ("x_m", "y_m", "z_m"))) if len(selected) >= 2 else None
                result[phase] = {"samples": len(selected), "start_clock_s": begin, "end_clock_s": end, "first": first, "last": last, "horizontal_displacement_m": horizontal, "three_dimensional_displacement_m": spatial}
                saved = summary["segments"][phase]
                matches = saved["samples"] == len(selected) and close(saved["start_clock_s"], begin) and close(saved["end_clock_s"], end) and saved["interval"] == "[start,end)" and close(saved["horizontal_displacement_m"], horizontal) and close(saved["three_dimensional_displacement_m"], spatial)
                for which, point in (("first", first), ("last", last)):
                    matches &= saved[which] is None if point is None else saved[which] is not None and all(close(saved[which][key], point[key]) for key in ("native_stamp_s", "x_m", "y_m", "z_m", "entity_id")) and saved[which]["entity_name"] == point["entity_name"] and saved[which]["native_header_json"] == point["native_header_json"]
                check(phase+"_recomputed", matches)
                check(phase+"_samples", len(selected) >= 2 and first["native_stamp_s"] < last["native_stamp_s"], incomplete=True)
            motion = result["equal_thrust"]["horizontal_displacement_m"]
            check("received_nonzero_horizontal_motion", motion is not None and motion > 1e-6, "Observed displacement only; no attribution to thrust", incomplete=motion is None)
            data_path = source / "first_image.data.bin"
            check("rgb8_image_stride", image["encoding"] == "rgb8" and number(image["step"]) >= 3*number(image["width"]), "This original-example audit checks the observed official rgb8 camera format")
            check("received_image_bytes", image["data_file"] == data_path.name and number(image["width"]) > 0 and number(image["height"]) > 0 and number(image["step"]) > 0 and data_path.stat().st_size == image["data_bytes"] == image["height"]*image["step"] and digest(data_path) == image["data_sha256"] and summary["first_image"] == image and summary["rendered_buffer_received"] is True)
            check("image_csv_header", len(images) == 1 and all(close(images[0][key], image[key]) for key in ("width", "height", "step", "data_bytes", "stamp_sec", "stamp_nanosec")) and all(images[0][key] == image[key] for key in ("encoding", "frame_id", "data_sha256", "data_file")))
    except (OSError, ValueError, KeyError, TypeError, csv.Error) as exc:
        check("input_schema", False, f"{type(exc).__name__}: {exc}", incomplete=isinstance(exc, (KeyError, FileNotFoundError)))
    after = {name: digest(path) if path.is_file() else None for name, path in paths.items()}
    check("source_bytes_unchanged", before == after)
    check("source_inventory_unchanged", {name[4:] for name in paths if name.startswith("run/")} == {p.name for p in source.iterdir() if p.is_file()})
    status = "FAIL" if any(row["status"] == "FAIL" for row in checks) else "INCOMPLETE" if any(row["status"] == "INCOMPLETE" for row in checks) else "PASS"
    report = {"status": status, "source_run": str(source), "launch_metadata": str(launch_path), "checker_sha256": digest(Path(__file__).resolve()), "checks": checks, "segments": result, "source_hashes_before": before, "source_hashes_after": after, "scope": "Received native samples in half-open ROS publication-clock intervals, not simulator acknowledgement or lossless coverage. No causal, collision, energy, hardware or automatic course-acceptance claim. Installed build identity needs external evidence."}
    (out / "checks.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    text = ["# P5 VRX 原例保存日志独立复核", "", f"状态：{status}。", "", "| 原生采样区间 | 样本数 | 水平位移（米） | 三维位移（米） |", "|---|---:|---:|---:|"]
    for phase, metric in result.items():
        text.append(f"| {phase} | {metric['samples']} | {metric['horizontal_displacement_m']} | {metric['three_dimensional_displacement_m']} |")
    text += ["", "按原生时间戳重选半开区间和首末样本；原始CSV处理phase不用于分段。图像核查基于实收缓冲、尺寸和SHA。位移不构成推力因果、碰撞、电量、实船或自动课程通过证据。", "", "逐项检查、原始文件前后SHA与首末原生记录见checks.json。"]
    text += [f"- {row['status']}：{row['name']}；{row['detail']}" for row in checks if row["status"] != "PASS"]
    (out / "摘要.md").write_text("\n".join(text)+"\n", encoding="utf-8")
    print(out / "摘要.md")
    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    raise SystemExit(run(arguments.source_run, arguments.output))
