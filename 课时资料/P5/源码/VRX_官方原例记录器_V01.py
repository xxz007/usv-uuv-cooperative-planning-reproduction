"""Record the unmodified VRX WAM-V example through externally configured bridges.

--model-frame matches TransformStamped.child_frame_id exactly. The external
launcher must check that this is the original WAM-V model, not a sensor/link.
This script owns only its ROS node; it never starts or stops Gazebo.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import signal
import time
import traceback


def run(out, role, pose_topic, model_frame, image_topic):
    out = Path(out)
    if not out.is_absolute():
        raise ValueError("--out must be an absolute new directory")
    if not model_frame or not pose_topic.startswith("/") or not image_topic.startswith("/"):
        raise ValueError("Use a nonempty exact child frame and absolute ROS topic names")
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    metadata = {
        "role": role, "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "pose_topic": pose_topic, "model_child_frame_exact_match": model_frame, "image_topic": image_topic,
        "clock_topic": "/clock", "thrust_topics": ["/wamv/thrusters/left/thrust", "/wamv/thrusters/right/thrust"],
        "protocol": {"zero_baseline_sim_s": 3.0, "equal_thrust_sim_s": 8.0, "each_thrust_n": 100.0,
                     "wall_budget_s": 240.0, "feedback_wall_timeout_s": 10.0,
                     "maximum_model_receive_age_sim_s": 0.5},
        "bridge_dependency": "External Pose_V -> TFMessage bridge for /world/sydney_regatta/pose/info; launcher must verify exact model child_frame_id and native parent frame before running",
        "observation_qos": "BEST_EFFORT/VOLATILE; clock/pose depth 1000 record received samples, Image depth 1 records the first frame only; no lossless-transport claim",
        "boundary": "ROS publications, actual model-pose differences and a received rendered Image are separate evidence; no general causal, energy, collision or real-vessel claim"
    }
    (out / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    columns = {
        "clock": ["wall_elapsed_s", "phase", "sec", "nanosec", "clock_s"],
        "model_pose": ["wall_elapsed_s", "phase", "clock_s", "elapsed_sim_s", "stamp_sec", "stamp_nanosec",
                       "parent_frame", "child_frame", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "usable"],
        "commands": ["wall_elapsed_s", "phase", "clock_s", "elapsed_sim_s", "left_thrust_n", "right_thrust_n", "reason"],
        "image": ["wall_elapsed_s", "phase", "clock_s", "stamp_sec", "stamp_nanosec", "frame_id", "encoding",
                  "width", "height", "step", "is_bigendian", "data_bytes", "data_sha256", "data_file"]
    }
    streams = {name: (out / f"{name}.csv").open("w", newline="", encoding="utf-8") for name in columns}
    writers = {name: csv.DictWriter(streams[name], fieldnames=fields) for name, fields in columns.items()}
    for writer in writers.values():
        writer.writeheader()
    state = {"phase": "waiting_input", "clock": None, "clock_wall": None, "clock_advances": 0,
             "latest_pose_wall": None, "latest_pose_clock_s": None, "parent_frame": None, "protocol_start_clock_s": None,
             "thrust_start_clock_s": None, "thrust_stop_clock_s": None, "image": None,
             "nonzero_publication_pairs": 0,
             "counts": {name: 0 for name in columns}}
    segments = {phase: {"samples": 0, "first": None, "last": None} for phase in ("zero_baseline", "equal_thrust")}
    rclpy, node, error, stop_error, status = None, None, None, None, "WAITING_INPUT"
    old_term = signal.getsignal(signal.SIGTERM)

    def on_terminate(signum, frame):
        raise KeyboardInterrupt("SIGTERM received")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from tf2_msgs.msg import TFMessage
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float64

        class Recorder(Node):
            def __init__(self):
                super().__init__("p5_vrx_original_example_recorder")
                qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
                self.left = self.create_publisher(Float64, metadata["thrust_topics"][0], 10)
                self.right = self.create_publisher(Float64, metadata["thrust_topics"][1], 10)
                self.clock_sub = self.create_subscription(Clock, "/clock", self.on_clock, qos)
                self.pose_sub = self.create_subscription(TFMessage, pose_topic, self.on_pose, qos)
                image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                       durability=DurabilityPolicy.VOLATILE)
                self.image_sub = self.create_subscription(Image, image_topic, self.on_image, image_qos)

            def on_clock(self, msg):
                now = msg.clock.sec + msg.clock.nanosec*1e-9
                writers["clock"].writerow({"wall_elapsed_s": time.monotonic()-started, "phase": state["phase"],
                                           "sec": msg.clock.sec, "nanosec": msg.clock.nanosec, "clock_s": now})
                state["counts"]["clock"] += 1
                if state["clock"] is not None and now < state["clock"]:
                    raise RuntimeError("Simulation clock moved backwards")
                if state["clock"] is None or now > state["clock"]:
                    state["clock_advances"] += 1
                    state["clock_wall"] = time.monotonic()
                state["clock"] = now

            def on_pose(self, msg):
                matches = [transform for transform in msg.transforms if transform.child_frame_id == model_frame]
                if len(matches) > 1:
                    raise RuntimeError("Ambiguous model child frame occurs more than once in a TFMessage")
                for transform in matches:
                    p, q = transform.transform.translation, transform.transform.rotation
                    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
                    usable = all(math.isfinite(value) for value in values) and sum(value*value for value in values[3:]) > 1e-12
                    row = {"wall_elapsed_s": time.monotonic()-started, "phase": state["phase"],
                           "clock_s": state["clock"], "elapsed_sim_s": None if state["protocol_start_clock_s"] is None
                           else state["clock"]-state["protocol_start_clock_s"],
                           "stamp_sec": transform.header.stamp.sec, "stamp_nanosec": transform.header.stamp.nanosec,
                           "parent_frame": transform.header.frame_id, "child_frame": transform.child_frame_id,
                           "x_m": p.x, "y_m": p.y, "z_m": p.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
                           "usable": usable}
                    writers["model_pose"].writerow(row)
                    state["counts"]["model_pose"] += 1
                    if not usable:
                        raise RuntimeError("Selected model transform has an invalid pose")
                    if state["parent_frame"] is not None and transform.header.frame_id != state["parent_frame"]:
                        raise RuntimeError("Selected model parent frame changed")
                    state["parent_frame"] = transform.header.frame_id
                    state["latest_pose_wall"] = time.monotonic()
                    state["latest_pose_clock_s"] = state["clock"]
                    if state["phase"] in segments:
                        segment = segments[state["phase"]]
                        segment["samples"] += 1
                        if segment["first"] is None:
                            segment["first"] = row
                        segment["last"] = row

            def on_image(self, msg):
                if state["image"] is not None:
                    return
                data = bytes(msg.data)
                row = {"wall_elapsed_s": time.monotonic()-started, "phase": state["phase"], "clock_s": state["clock"],
                       "stamp_sec": msg.header.stamp.sec, "stamp_nanosec": msg.header.stamp.nanosec,
                       "frame_id": msg.header.frame_id, "encoding": msg.encoding, "width": msg.width,
                       "height": msg.height, "step": msg.step, "is_bigendian": msg.is_bigendian,
                       "data_bytes": len(data), "data_sha256": hashlib.sha256(data).hexdigest(),
                       "data_file": "first_image.data.bin"}
                (out / row["data_file"]).write_bytes(data)
                (out / "first_image_header.json").write_text(json.dumps(row, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
                writers["image"].writerow(row)
                state["counts"]["image"] += 1
                state["image"] = row

            def publish(self, thrust, reason):
                self.left.publish(Float64(data=float(thrust)))
                self.right.publish(Float64(data=float(thrust)))
                writers["commands"].writerow({"wall_elapsed_s": time.monotonic()-started, "phase": state["phase"],
                                              "clock_s": state["clock"], "elapsed_sim_s": None
                                              if state["protocol_start_clock_s"] is None else state["clock"]-state["protocol_start_clock_s"],
                                              "left_thrust_n": thrust, "right_thrust_n": thrust, "reason": reason})
                state["counts"]["commands"] += 1
                if thrust != 0:
                    state["nonzero_publication_pairs"] += 1

        signal.signal(signal.SIGTERM, on_terminate)
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = Recorder()
        node.publish(0, "startup_zero")
        last_publish_clock, last_wait_zero_wall = None, started
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            wall, clock = time.monotonic(), state["clock"]
            if wall-started >= 240:
                status = "WALL_TIMEOUT"
                break
            if state["protocol_start_clock_s"] is None:
                if wall-last_wait_zero_wall >= 1:
                    node.publish(0, "waiting_for_real_pose_and_clock")
                    last_wait_zero_wall = wall
                if (state["latest_pose_clock_s"] is None or state["clock_advances"] < 2
                        or wall-state["latest_pose_wall"] > 10 or wall-state["clock_wall"] > 10
                        or clock-state["latest_pose_clock_s"] > 0.5):
                    continue
                state["protocol_start_clock_s"] = clock
                state["phase"] = "zero_baseline"
            if (wall-state["latest_pose_wall"] > 10 or wall-state["clock_wall"] > 10
                    or clock-state["latest_pose_clock_s"] > 0.5):
                status = "STALE_FEEDBACK"
                break
            if state["thrust_start_clock_s"] is not None and clock-state["thrust_start_clock_s"] >= 8:
                state["thrust_stop_clock_s"] = clock
                state["phase"] = "post_thrust_zero"
                node.publish(0, "eight_sim_seconds_thrust_finished")
                status = "PROTOCOL_FINISHED"
                break
            changed = state["phase"] == "zero_baseline" and clock-state["protocol_start_clock_s"] >= 3
            if changed:
                if segments["zero_baseline"]["samples"] < 2:
                    status = "INSUFFICIENT_BASELINE_OBSERVATIONS"
                    break
                state["phase"] = "equal_thrust"
                state["thrust_start_clock_s"] = clock
            phase = state["phase"]
            if changed or last_publish_clock is None or clock-last_publish_clock >= 0.1:
                node.publish(100 if phase == "equal_thrust" else 0, phase)
                last_publish_clock = clock
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
                        node.publish(0, "finally_zero")
                        rclpy.spin_once(node, timeout_sec=0.05)
                else:
                    stop_error = "ROS context stopped before zero-thrust cleanup"
            except Exception as exc:
                stop_error = f"{type(exc).__name__}: {exc}"
            finally:
                node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)
        for stream in streams.values():
            stream.close()
        for segment in segments.values():
            first, last = segment["first"], segment["last"]
            segment["horizontal_displacement_m"] = None if segment["samples"] < 2 else math.hypot(last["x_m"]-first["x_m"], last["y_m"]-first["y_m"])
            segment["three_dimensional_displacement_m"] = None if segment["samples"] < 2 else math.sqrt(sum((last[key]-first[key])**2 for key in ("x_m", "y_m", "z_m")))
        image = state["image"]
        rendered = bool(image and image["width"] > 0 and image["height"] > 0 and image["step"] > 0
                        and image["data_bytes"] == image["height"]*image["step"])
        complete = status == "PROTOCOL_FINISHED" and all(value["samples"] >= 2 for value in segments.values()) and rendered and not stop_error
        if status == "PROTOCOL_FINISHED":
            status = "RECORDING_COMPLETE" if complete else "INCOMPLETE_OBSERVATIONS"
        if stop_error:
            status = "STOP_PUBLICATION_FAILED"
        summary = {"status": status, "role": role, "recording_complete": complete, "error": error,
                   "stop_publication_error": stop_error, "wall_elapsed_s": time.monotonic()-started,
                   "counts": state["counts"], "model_parent_frame": state["parent_frame"],
                   "nonzero_publication_pairs": state["nonzero_publication_pairs"],
                   "protocol_start_clock_s": state["protocol_start_clock_s"],
                   "thrust_start_clock_s": state["thrust_start_clock_s"], "thrust_stop_clock_s": state["thrust_stop_clock_s"],
                   "segments": segments, "first_image": image, "rendered_buffer_received": rendered,
                   "script_sha256": metadata["script_sha256"], "course_acceptance": "NOT_AUTOMATICALLY_ASSESSED"}
        (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        lines = ["# VRX 官方原例记录摘要", "", f"状态：{status}；角色：{role}。", "",
                 "| 段 | 实际模型姿态采样数 | 首末水平位移（米） |", "|---|---:|---:|"]
        for phase, segment in segments.items():
            distance = "null" if segment["horizontal_displacement_m"] is None else str(segment["horizontal_displacement_m"])
            lines.append(f"| {phase} | {segment['samples']} | {distance} |")
        image_note = (f"首帧原始 Image 缓冲已收到且尺寸匹配：{rendered}；原始数据和 SHA 见 first_image_header.json。"
                      if image is not None else "未收到 Image，不生成图像数据文件，也不记为渲染帧证据。")
        lines += ["", image_note,
                  "", "推力 CSV 是本节点实际 ROS 发布记录，不是仿真端确认。两段位移分别来自其实际收到的指定模型姿态首末采样，不插值、不把传感器/链节姿态当船体，也不把基线漂移自动归因于推力。",
                  "", "模型通过 child_frame_id 精确匹配，正确性依赖运行前对外部世界 Pose_V→TFMessage 桥及坐标帧的核对。clock 与模型姿态未就绪时只发零推力；结束只清理本节点，不停止 Gazebo。",
                  "", "渲染图像、姿态位移和指令发布是分开的观察证据，不构成动力学普遍因果证明、实艇验证或 P5 自动通过。"]
        if error or stop_error:
            lines += ["", f"异常：{error or stop_error}"]
        (out / "摘要.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    print(out / "摘要.md")
    return 0 if complete else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--role", required=True, choices=("teacher", "learner"))
    parser.add_argument("--pose-topic", required=True)
    parser.add_argument("--model-frame", required=True, help="Exact TransformStamped.child_frame_id of the WAM-V model")
    parser.add_argument("--image-topic", required=True)
    args = parser.parse_args()
    raise SystemExit(run(args.out, args.role, args.pose_topic, args.model_frame, args.image_topic))
