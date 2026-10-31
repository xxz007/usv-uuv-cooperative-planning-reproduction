"""One real ROS/VRX waypoint run and a separately executed ideal 2D reference.

Copy this file and its JSON configuration together to the ROS machine. An
external launcher owns Gazebo and the bridges; this file owns only its ROS node.
No prerecorded positions are published, and no external process is stopped.
"""
import argparse
from collections import deque
import csv
import hashlib
import json
import math
from pathlib import Path
import signal
import time
import traceback


def run(config_path, out, role, framework_version):
    config_path, out = Path(config_path).resolve(), Path(out)
    if not out.is_absolute():
        raise ValueError("--out must be an absolute, new directory")
    config_bytes = config_path.read_bytes()
    config = json.loads(config_bytes)
    task, startup = config["task"], config["startup"]
    control, envelope = config["controller"], config["operating_envelope"]
    positive = [task[key] for key in ("forward_distance_m", "target_speed_mps", "arrival_radius_m",
                                    "arrival_hold_sim_s", "sim_budget_s", "wall_budget_s")]
    positive += [startup[key] for key in ("stable_window_sim_s", "maximum_window_drift_m",
                                         "maximum_window_yaw_change_rad", "maximum_observation_age_sim_s",
                                         "maximum_observation_gap_sim_s", "feedback_wall_timeout_s")]
    positive += [control[key] for key in ("period_sim_s", "speed_feedforward_n_per_mps",
                                         "speed_gain_n_per_mps", "distance_speed_gain_per_s",
                                         "heading_gain_n_per_rad", "maximum_differential_n", "maximum_thrust_n")]
    positive.append(config["reference"]["sample_dt_s"])
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) or value <= 0 for value in positive):
        raise ValueError("Task, observation and controller scales must be positive finite numbers")
    for key in ("minimum_odometry_samples", "minimum_advancing_clock_samples"):
        if type(startup[key]) is not int or startup[key] < 2:
            raise ValueError(f"{key} must be an integer >= 2")
    if not (0 < control["velocity_filter_alpha"] <= 1
            and math.isfinite(control["minimum_thrust_n"]) and control["minimum_thrust_n"] <= 0):
        raise ValueError("Invalid thrust limit or velocity filter alpha")
    for axis in ("along_m", "cross_m"):
        limits = envelope[axis]
        if (len(limits) != 2 or any(not isinstance(v, (int, float)) or isinstance(v, bool)
                                  or not math.isfinite(v) for v in limits)
                or not limits[0] < 0 < limits[1]):
            raise ValueError(f"Invalid {axis} envelope")
    if not (0 < task["arrival_radius_m"] < task["forward_distance_m"] < envelope["along_m"][1]):
        raise ValueError("Forward goal must be beyond the arrival radius and within the envelope")
    if any(not isinstance(topic, str) or not topic.startswith("/") for topic in config["topics"].values()):
        raise ValueError("ROS topics must be absolute names")

    out.mkdir(parents=True, exist_ok=False)
    (out / "frozen_config.json").write_bytes(config_bytes)
    started_wall = time.monotonic()
    metadata = {
        "framework_version": framework_version, "role": role, "config_version": config["config_version"],
        "source_id": config["source_id"], "upstream": config["upstream"],
        "source_id_evidence": "Declared frozen upstream ID; external launch records must verify the actual VRX checkout/build",
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "topics": config["topics"], "gazebo_process_owner": "external launcher",
        "observation_qos": "BEST_EFFORT/VOLATILE/KEEP_LAST depth 1000; CSV contains all messages received by this node, not proof of lossless transport",
        "command_evidence": "Actual Float64 publish calls; no acknowledgement of Gazebo acceptance is inferred",
        "velocity_estimate": "Filtered native-odometry XY differences projected onto current yaw; twist frame is not assumed",
        "course_boundary": "No automatic learner-understanding or complete-P5 acceptance claim"
    }
    (out / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    fields = {
        "clock": ["wall_elapsed_s", "phase", "sec", "nanosec", "clock_s"],
        "odometry": ["wall_elapsed_s", "phase", "clock_s", "stamp_sec", "stamp_nanosec", "stamp_s",
                     "frame_id", "child_frame_id", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
                     "yaw_rad", "twist_linear_x", "twist_linear_y", "twist_linear_z", "twist_angular_x",
                     "twist_angular_y", "twist_angular_z", "pose_covariance_json", "twist_covariance_json",
                     "elapsed_sim_s", "goal_distance_m", "along_m", "cross_track_m", "outside_envelope",
                     "estimated_forward_speed_mps", "left_thrust_n", "right_thrust_n", "usable"],
        "commands": ["wall_elapsed_s", "phase", "clock_s", "odom_stamp_s", "elapsed_sim_s", "x_m", "y_m",
                     "yaw_rad", "goal_distance_m", "cross_track_m", "estimated_forward_speed_mps",
                     "left_thrust_n", "right_thrust_n", "reason"],
        "reference_2d": ["elapsed_sim_s", "x_m", "y_m", "yaw_rad", "speed_mps", "goal_distance_m",
                         "along_m", "cross_track_m", "outside_envelope", "inside_hold_s"]
    }
    streams = {name: (out / f"{name}.csv").open("w", newline="", encoding="utf-8") for name in fields}
    writers = {name: csv.DictWriter(streams[name], fieldnames=columns) for name, columns in fields.items()}
    for writer in writers.values():
        writer.writeheader()
    state = {"clock": None, "clock_wall": None, "clock_advances": 0, "latest": None,
             "phase": "waiting_input", "left": 0.0, "right": 0.0, "common": None,
             "hold_start": None, "completed": False, "boundary_stop": False,
             "counts": {name: 0 for name in fields}}
    window = deque()
    vrx = {"system": "VRX", "status": "NOT_STARTED", "completion_time_s": None,
           "maximum_cross_track_m": None, "boundary_violation_events": 0}
    reference = {"system": "ideal_2D", "status": "NOT_RUN", "completion_time_s": None,
                 "maximum_cross_track_m": None, "boundary_violation_events": 0}
    common, node, rclpy = None, None, None
    error, shutdown_error, status = None, None, "WAITING_INPUT"
    previous_term_handler = signal.getsignal(signal.SIGTERM)

    def on_terminate(signum, frame):
        raise KeyboardInterrupt("SIGTERM received")

    def geometry(x, y):
        dx, dy = x - common["start"]["x_m"], y - common["start"]["y_m"]
        along = dx * common["forward_unit"][0] + dy * common["forward_unit"][1]
        cross = -dx * common["forward_unit"][1] + dy * common["forward_unit"][0]
        outside = not (envelope["along_m"][0] <= along <= envelope["along_m"][1]
                       and envelope["cross_m"][0] <= cross <= envelope["cross_m"][1])
        return {"goal_distance_m": math.hypot(x - common["goal"]["x_m"], y - common["goal"]["y_m"]),
                "along_m": along, "cross_track_m": abs(cross), "outside_envelope": outside}

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rclpy.signals import SignalHandlerOptions
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float64

        class Recorder(Node):
            def __init__(self):
                super().__init__("p5_vrx_waypoint")
                qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                self.left_pub = self.create_publisher(Float64, config["topics"]["left_thrust"], 10)
                self.right_pub = self.create_publisher(Float64, config["topics"]["right_thrust"], 10)
                self.clock_sub = self.create_subscription(Clock, config["topics"]["clock"], self.on_clock, qos)
                self.odom_sub = self.create_subscription(Odometry, config["topics"]["odometry"], self.on_odom, qos)
                self.was_outside = False

            def on_clock(self, msg):
                now = msg.clock.sec + msg.clock.nanosec * 1e-9
                writers["clock"].writerow({"wall_elapsed_s": time.monotonic() - started_wall,
                                           "phase": state["phase"], "sec": msg.clock.sec,
                                           "nanosec": msg.clock.nanosec, "clock_s": now})
                state["counts"]["clock"] += 1
                if state["clock"] is not None and now < state["clock"] - 1e-9:
                    if common is not None:
                        raise RuntimeError("Simulation clock moved backwards during the task")
                    window.clear()
                    state.update(latest=None, clock_advances=0)
                if state["clock"] is None or now > state["clock"]:
                    state["clock_advances"] += 1
                    state["clock_wall"] = time.monotonic()
                state["clock"] = now

            def on_odom(self, msg):
                pose, twist = msg.pose.pose, msg.twist.twist
                p, q = pose.position, pose.orientation
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                row = {"wall_elapsed_s": time.monotonic() - started_wall, "phase": state["phase"],
                       "clock_s": state["clock"], "stamp_sec": msg.header.stamp.sec,
                       "stamp_nanosec": msg.header.stamp.nanosec, "stamp_s": stamp,
                       "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id,
                       "x_m": p.x, "y_m": p.y, "z_m": p.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
                       "twist_linear_x": twist.linear.x, "twist_linear_y": twist.linear.y,
                       "twist_linear_z": twist.linear.z, "twist_angular_x": twist.angular.x,
                       "twist_angular_y": twist.angular.y, "twist_angular_z": twist.angular.z,
                       "pose_covariance_json": json.dumps(list(msg.pose.covariance)),
                       "twist_covariance_json": json.dumps(list(msg.twist.covariance)),
                       "left_thrust_n": state["left"], "right_thrust_n": state["right"], "usable": False}
                state["counts"]["odometry"] += 1
                norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z, norm, stamp)) or norm < 1e-9:
                    writers["odometry"].writerow(row)
                    raise RuntimeError("Nonfinite odometry pose or invalid quaternion")
                qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
                yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                previous = state["latest"]
                row.update(yaw_rad=yaw, usable=True)
                if common is not None:
                    row.update(geometry(p.x, p.y), elapsed_sim_s=stamp-common["dispatch_clock_s"])
                if previous is not None and stamp <= previous["stamp_s"]:
                    row["usable"] = False
                    writers["odometry"].writerow(row)
                    if common is not None and stamp < previous["stamp_s"] - 1e-9:
                        raise RuntimeError("Odometry stamp moved backwards during the task")
                    return
                if common is not None and (msg.header.frame_id != common["coordinate_frame"]
                                           or msg.child_frame_id != common["child_frame"]):
                    row["usable"] = False
                    writers["odometry"].writerow(row)
                    raise RuntimeError("Odometry frame changed during the task")
                speed = 0.0
                gap = stamp - previous["stamp_s"] if previous else None
                if previous is not None:
                    raw_speed = ((p.x-previous["x_m"])*math.cos(yaw)
                                 + (p.y-previous["y_m"])*math.sin(yaw)) / gap
                    alpha = control["velocity_filter_alpha"]
                    speed = alpha*raw_speed + (1-alpha)*previous["speed_mps"]
                row["estimated_forward_speed_mps"] = speed
                writers["odometry"].writerow(row)
                sample = {"stamp_s": stamp, "x_m": p.x, "y_m": p.y, "yaw_rad": yaw,
                          "speed_mps": speed, "wall": time.monotonic(),
                          "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id}
                state["latest"] = sample
                if common is None:
                    if gap is not None and gap > startup["maximum_observation_gap_sim_s"]:
                        window.clear()
                    window.append(sample)
                    while len(window) > 1 and stamp-window[1]["stamp_s"] >= startup["stable_window_sim_s"]:
                        window.popleft()
                elif state["phase"] == "task":
                    if gap is not None and gap > startup["maximum_observation_gap_sim_s"]:
                        raise RuntimeError("Odometry gap exceeds the predefined observed-hold limit")
                    if row["elapsed_sim_s"] >= 0:
                        vrx["maximum_cross_track_m"] = max(vrx["maximum_cross_track_m"] or 0, row["cross_track_m"])
                        if row["outside_envelope"] and not self.was_outside:
                            vrx["boundary_violation_events"] += 1
                        self.was_outside = row["outside_envelope"]
                        state["boundary_stop"] = row["outside_envelope"] and envelope["stop_on_violation"]
                        if row["goal_distance_m"] <= task["arrival_radius_m"] + 1e-9:
                            if state["hold_start"] is None:
                                state["hold_start"] = stamp
                            if (stamp-state["hold_start"] >= task["arrival_hold_sim_s"] - 1e-9
                                    and row["elapsed_sim_s"] <= task["sim_budget_s"] + 1e-9):
                                state["completed"] = True
                                vrx["completion_time_s"] = row["elapsed_sim_s"]
                        else:
                            state["hold_start"] = None

            def publish_thrust(self, left, right, reason):
                self.left_pub.publish(Float64(data=float(left)))
                self.right_pub.publish(Float64(data=float(right)))
                state.update(left=left, right=right)
                row = {"wall_elapsed_s": time.monotonic()-started_wall, "phase": state["phase"],
                       "clock_s": state["clock"], "left_thrust_n": left, "right_thrust_n": right,
                       "reason": reason}
                latest = state["latest"]
                if latest is not None:
                    row.update(odom_stamp_s=latest["stamp_s"], x_m=latest["x_m"], y_m=latest["y_m"],
                               yaw_rad=latest["yaw_rad"], estimated_forward_speed_mps=latest["speed_mps"])
                    if common is not None:
                        measures = geometry(latest["x_m"], latest["y_m"])
                        row.update(elapsed_sim_s=state["clock"]-common["dispatch_clock_s"],
                                   goal_distance_m=measures["goal_distance_m"], cross_track_m=measures["cross_track_m"])
                writers["commands"].writerow(row)
                state["counts"]["commands"] += 1

        signal.signal(signal.SIGTERM, on_terminate)
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = Recorder()
        node.publish_thrust(0, 0, "startup")
        last_command_clock, last_wait_zero_wall = None, started_wall
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            wall = time.monotonic()
            if wall-started_wall >= task["wall_budget_s"]:
                status = "WALL_TIMEOUT"
                break
            latest, clock = state["latest"], state["clock"]
            if common is None:
                if wall-last_wait_zero_wall >= 1:
                    node.publish_thrust(0, 0, "waiting_input")
                    last_wait_zero_wall = wall
                if (latest is None or clock is None or len(window) < startup["minimum_odometry_samples"]
                        or state["clock_advances"] < startup["minimum_advancing_clock_samples"]
                        or latest["stamp_s"]-window[0]["stamp_s"] < startup["stable_window_sim_s"]-1e-9
                        or not -0.1 <= clock-latest["stamp_s"] <= startup["maximum_observation_age_sim_s"]):
                    continue
                first = window[0]
                if any(math.hypot(p["x_m"]-first["x_m"], p["y_m"]-first["y_m"]) > startup["maximum_window_drift_m"]
                       or abs(math.atan2(math.sin(p["yaw_rad"]-first["yaw_rad"]),
                                         math.cos(p["yaw_rad"]-first["yaw_rad"]))) > startup["maximum_window_yaw_change_rad"]
                       or p["frame_id"] != latest["frame_id"] or p["child_frame_id"] != latest["child_frame_id"]
                       for p in window):
                    continue
                forward = [math.cos(latest["yaw_rad"]), math.sin(latest["yaw_rad"])]
                common = {"task_id": task["id"], "coordinate_frame": latest["frame_id"],
                          "child_frame": latest["child_frame_id"],
                          "coordinate_definition": "Native Odometry XY metres; yaw radians CCW from its +X toward +Y",
                          "start": {"x_m": latest["x_m"], "y_m": latest["y_m"], "yaw_rad": latest["yaw_rad"],
                                    "odom_stamp_s": latest["stamp_s"]},
                          "forward_unit": forward,
                          "goal": {"x_m": latest["x_m"]+task["forward_distance_m"]*forward[0],
                                   "y_m": latest["y_m"]+task["forward_distance_m"]*forward[1]},
                          "dispatch_clock_s": clock, "initial_odometry_age_s": clock-latest["stamp_s"],
                          "elapsed_time_definition": "Simulation time since dispatch_clock_s; only post-dispatch odometry enters task metrics; ideal 2D starts at the same elapsed zero",
                          "task": task, "operating_envelope": envelope,
                          "hold_rule": "Consecutive received poses inside the radius spanning hold_sim_s; gap limited by frozen startup config",
                          "reference_model": config["reference"], "config_sha256": metadata["config_sha256"],
                          "script_sha256": metadata["script_sha256"], "source_id": metadata["source_id"]}
                state["common"] = common
                (out / "common_inputs.json").write_text(json.dumps(common, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
                for stream in streams.values():
                    stream.flush()
                x, y, t, hold_start, was_outside = latest["x_m"], latest["y_m"], 0.0, None, False
                dt = config["reference"]["sample_dt_s"]
                reference.update(status="SIM_TIMEOUT", maximum_cross_track_m=0.0)
                while t <= task["sim_budget_s"] + 1e-9:
                    measures = geometry(x, y)
                    inside = measures["goal_distance_m"] <= task["arrival_radius_m"] + 1e-9
                    hold_start = (t if hold_start is None else hold_start) if inside else None
                    hold = t-hold_start if hold_start is not None else 0.0
                    speed = 0.0 if inside else task["target_speed_mps"]
                    writers["reference_2d"].writerow(dict(measures, elapsed_sim_s=t, x_m=x, y_m=y,
                                                         yaw_rad=latest["yaw_rad"], speed_mps=speed, inside_hold_s=hold))
                    state["counts"]["reference_2d"] += 1
                    reference["maximum_cross_track_m"] = max(reference["maximum_cross_track_m"], measures["cross_track_m"])
                    if measures["outside_envelope"] and not was_outside:
                        reference["boundary_violation_events"] += 1
                    was_outside = measures["outside_envelope"]
                    if inside and hold >= task["arrival_hold_sim_s"] - 1e-9:
                        reference.update(status="COMPLETED", completion_time_s=t)
                        break
                    if was_outside and envelope["stop_on_violation"]:
                        reference["status"] = "BOUNDARY_STOP"
                        break
                    step = min(speed*dt, max(0.0, measures["goal_distance_m"]-task["arrival_radius_m"]))
                    x += step*forward[0]
                    y += step*forward[1]
                    t = round(t+dt, 10)
                state["phase"] = "task"
                vrx["status"], status = "RUNNING", "RUNNING"
            if state["completed"]:
                status = "COMPLETED"
                break
            if state["boundary_stop"]:
                status = "BOUNDARY_STOP"
                break
            if clock-common["dispatch_clock_s"] >= task["sim_budget_s"]:
                status = "SIM_TIMEOUT"
                break
            if (wall-latest["wall"] > startup["feedback_wall_timeout_s"]
                    or wall-state["clock_wall"] > startup["feedback_wall_timeout_s"]
                    or not -0.1 <= clock-latest["stamp_s"] <= startup["maximum_observation_age_sim_s"]):
                status = "STALE_FEEDBACK"
                break
            if last_command_clock is None or clock-last_command_clock >= control["period_sim_s"]-1e-9:
                measures = geometry(latest["x_m"], latest["y_m"])
                inside = measures["goal_distance_m"] <= task["arrival_radius_m"]+1e-9
                wanted_speed = 0.0 if inside else min(task["target_speed_mps"], control["distance_speed_gain_per_s"]
                                                      * max(0.0, measures["goal_distance_m"]-task["arrival_radius_m"]*0.5))
                wanted_yaw = math.atan2(common["goal"]["y_m"]-latest["y_m"], common["goal"]["x_m"]-latest["x_m"])
                heading_error = math.atan2(math.sin(wanted_yaw-latest["yaw_rad"]), math.cos(wanted_yaw-latest["yaw_rad"]))
                base = control["speed_feedforward_n_per_mps"]*wanted_speed + control["speed_gain_n_per_mps"]*(wanted_speed-latest["speed_mps"])
                turn = 0.0 if inside else max(-control["maximum_differential_n"],
                                            min(control["maximum_differential_n"], control["heading_gain_n_per_rad"]*heading_error))
                left = max(control["minimum_thrust_n"], min(control["maximum_thrust_n"], base-turn))
                right = max(control["minimum_thrust_n"], min(control["maximum_thrust_n"], base+turn))
                node.publish_thrust(left, right, "arrival_braking" if inside else "waypoint_control")
                last_command_clock = clock
                for stream in streams.values():
                    stream.flush()
        else:
            status = "ROS_SHUTDOWN"
    except KeyboardInterrupt as exc:
        status, error = "INTERRUPTED", str(exc) or "KeyboardInterrupt"
    except Exception as exc:
        status, error = "ERROR", f"{type(exc).__name__}: {exc}"
        (out / "error.txt").write_text(traceback.format_exc(), encoding="utf-8")
    finally:
        state["phase"] = "shutdown"
        if node is not None:
            try:
                if rclpy.ok():
                    for _ in range(3):
                        node.publish_thrust(0, 0, "finally_stop")
                        rclpy.spin_once(node, timeout_sec=0.05)
                else:
                    shutdown_error = "ROS context already stopped; zero-thrust publication unavailable"
            except Exception as exc:
                shutdown_error = f"Zero-thrust cleanup failed: {type(exc).__name__}: {exc}"
            finally:
                node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term_handler)
        for stream in streams.values():
            stream.close()
        vrx["status"] = status
        if shutdown_error:
            status = "STOP_PUBLICATION_FAILED"
        rows = [vrx, reference]
        with (out / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(vrx))
            writer.writeheader()
            writer.writerows(rows)
        summary = {"status": status, "task_completed": state["completed"] and common is not None,
                   "role": role, "framework_version": framework_version, "source_id": config["source_id"],
                   "error": error, "stop_publication_error": shutdown_error,
                   "wall_elapsed_s": time.monotonic()-started_wall, "received_or_published_rows": state["counts"],
                   "common_inputs_obtained": common is not None, "metrics": rows,
                   "course_acceptance": "NOT_AUTOMATICALLY_ASSESSED",
                   "scope": "One actual ROS/VRX task plus a separately executed ideal 2D model; sampled coordinate envelope only; no collision, energy or hardware-validation claim"}
        (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        lines = ["# P5 VRX 单航点运行摘要", "", f"状态：{status}；角色：{role}；版本：{framework_version}。", "",
                 "| 系统 | 状态 | 完成时间（模拟秒） | 最大横向偏差（米） | 坐标包络违规事件 |",
                 "|---|---|---:|---:|---:|"]
        for row in rows:
            completion = "null" if row["completion_time_s"] is None else row["completion_time_s"]
            cross_track = "null" if row["maximum_cross_track_m"] is None else row["maximum_cross_track_m"]
            lines.append(f"| {row['system']} | {row['status']} | {completion} | {cross_track} | {row['boundary_violation_events']} |")
        lines += ["", "未完成的完成时间为 null；2D 是独立执行的理想点运动学参考。VRX 指标只依据本节点实际收到的原生里程计。",
                  "", "坐标包络已在配置中预定义，随观测起点/朝向冻结；违规只代表越出该包络。到达保持依据连续收到的有效采样及预定最大间隔。",
                  "", "所有收到的时钟与里程计、每次推力发布保存在 CSV。推力发布不代表仿真端已确认接收。本人理解与 P5 整阶段验收由主端另行登记。"]
        if error or shutdown_error:
            lines += ["", f"异常：{error or shutdown_error}"]
        (out / "摘要.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    print(out / "摘要.md")
    return 0 if status == "COMPLETED" else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True, help="Absolute output directory that does not exist yet")
    parser.add_argument("--role", required=True, choices=("teacher", "learner"))
    parser.add_argument("--framework-version", required=True)
    arguments = parser.parse_args()
    raise SystemExit(run(arguments.config, arguments.out, arguments.role, arguments.framework_version))
