"""Real MOOS processes, one scheduler task, feedback and an explicit timeout.

The headless configuration is derived from the installed s1_alpha example.
No prerecorded trajectory is sent to the simulator. Only this run's child
processes are stopped. Original MOOS sources, missions and evidence stay intact.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time

from usv_uuc_cooperation.config import PACKAGE_ROOT, PROJECT_ROOT, FRAMEWORK_VERSION, new_run_directory
from usv_uuc_cooperation.scheduling.greedy_scheduler import greedy


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_rows(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def goal_error(goal, envelope):
    if not isinstance(goal, list) or len(goal) != 2:
        return "goal must contain two ENU coordinates"
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in goal):
        return "goal coordinates must be finite numbers"
    if not all(envelope[axis][0] <= value <= envelope[axis][1] for axis, value in zip(("x", "y"), goal)):
        return "goal is outside the declared operating envelope"
    return None


def query(binary, mission, folder, condition, wait_sim_s, warp):
    folder.mkdir(parents=True, exist_ok=False)
    variables = ("NAV_X", "NAV_Y", "NAV_SPEED", "NAV_HEADING", "DB_UPTIME", "P5_TASK_ID", "P5_TASK_DONE", "WPT_STAT")
    command = [str(binary), str(mission), f"--condition={condition}", f"--wait={wait_sim_s}", "--csv",
               *[f"--check_var={v}" for v in variables]]
    returned = subprocess.run(command, cwd=folder, text=True, capture_output=True,
                              timeout=max(15, wait_sim_s / warp + 15))
    (folder / "command.json").write_text(json.dumps(command, ensure_ascii=False, indent=2) + "\n")
    (folder / "stdout.txt").write_text(returned.stdout + returned.stderr)
    values = {}
    checkvars = folder / ".checkvars"
    if checkvars.is_file():
        for line in checkvars.read_text().splitlines():
            name, _, value = line.partition(",")
            values[name.strip()] = value.strip()
    return {"returncode": returned.returncode, "values": values, "command": command}


def alog_evidence(folder, task_id, start, goal, envelope):
    """Reconstruct pose pairs from this run's uSimMarine-stamped raw alog."""
    paths = sorted(folder.rglob("*.alog"))
    if not paths:
        raise RuntimeError(f"pLogger produced no raw alog: {folder}")
    nav, markers, complete, wpt = {}, [], [], []
    trajectory = []
    for path in paths:
        for line in path.read_text(errors="replace").splitlines():
            if not line.strip() or line.lstrip().startswith("%"):
                continue
            fields = line.split(None, 3)
            if len(fields) != 4:
                continue
            stamp, variable, source, value = fields
            value = value.strip()
            if not re.fullmatch(r"\d+(?:\.\d+)?", stamp):
                continue
            stamp = float(stamp)
            if variable in ("NAV_X", "NAV_Y") and source.startswith("uSimMarine"):
                nav[variable] = (stamp, float(value))
                if variable == "NAV_Y" and "NAV_X" in nav and abs(nav["NAV_X"][0] - stamp) < 0.002:
                    x, y = nav["NAV_X"][1], nav["NAV_Y"][1]
                    dx, dy = goal[0] - start[0], goal[1] - start[1]
                    length = math.hypot(dx, dy)
                    cross_track = abs(dy * (x - start[0]) - dx * (y - start[1])) / length if length else math.hypot(x-start[0], y-start[1])
                    trajectory.append({"alog_time_s": stamp, "x_m": x, "y_m": y,
                                       "goal_distance_m": math.hypot(x-goal[0], y-goal[1]),
                                       "cross_track_m": cross_track})
            elif variable == "P5_DISPATCH" and value == task_id:
                markers.append(stamp)
            elif variable == "P5_TASK_DONE" and value == task_id and source.startswith("pHelmIvP"):
                complete.append(stamp)
            elif variable == "WPT_STAT" and "completed" in value and source.startswith("pHelmIvP"):
                wpt.append({"alog_time_s": stamp, "value": value})
    if not trajectory:
        raise RuntimeError(f"No paired uSimMarine NAV_X/Y samples in {folder}")
    dispatch = markers[0] if markers else None
    completed_at = complete[0] if complete else None
    for row in trajectory:
        row["time_from_dispatch_s"] = row["alog_time_s"] - dispatch if dispatch is not None else None
    write_rows(folder / "trajectory.csv", trajectory)
    post_dispatch = [r for r in trajectory if (dispatch is None or r["alog_time_s"] >= dispatch)
                     and (completed_at is None or r["alog_time_s"] <= completed_at)]
    prior = [r for r in trajectory if completed_at is not None and r["alog_time_s"] <= completed_at]
    actual_at_completion = prior[-1] if prior else None
    return {"raw_alogs": [str(p.relative_to(folder)) for p in paths], "sample_count": len(trajectory),
            "dispatch_alog_time_s": dispatch, "complete_alog_time_s": completed_at,
            "completion_time_s": completed_at - dispatch if completed_at is not None and dispatch is not None else None,
            "completion_pose": actual_at_completion,
            "completion_pose_age_s": completed_at - actual_at_completion["alog_time_s"] if actual_at_completion is not None else None,
            "behavior_completion_messages": wpt,
            "actual_distance_m": sum(math.hypot(b["x_m"]-a["x_m"], b["y_m"]-a["y_m"]) for a,b in zip(post_dispatch, post_dispatch[1:])),
            "maximum_cross_track_m": max(r["cross_track_m"] for r in post_dispatch),
            "operating_envelope_violations": sum(not (envelope["x"][0] <= r["x_m"] <= envelope["x"][1] and envelope["y"][0] <= r["y_m"] <= envelope["y"][1]) for r in post_dispatch),
            "metric_time_scope": "dispatch through completion event, or through last observed pose if not completed",
            "first_pose": trajectory[0], "last_pose": trajectory[-1]}


def run_case(folder, moos_root, config, task, example=False, timeout=False):
    folder.mkdir(parents=True, exist_ok=False)
    binaries = moos_root / "bin"
    original = moos_root / "ivp/missions/s1_alpha"
    mission_text = (original / "alpha.moos").read_text()
    warp = config["time_warp"]
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    mission_text = re.sub(r"(?m)^ServerHost\s*=.*$", "ServerHost = localhost", mission_text)
    mission_text = re.sub(r"(?m)^ServerPort\s*=.*$", f"ServerPort = {port}", mission_text)
    mission_text = re.sub(r"(?m)^MOOSTimeWarp\s*=.*$", f"MOOSTimeWarp = {warp}", mission_text)
    mission_text = re.sub(r"(?m)^\s*Run\s*=\s*pMarineViewer.*\n", "", mission_text)
    start = [config["initial_pose"]["x_m"], config["initial_pose"]["y_m"]]
    if example:
        behavior_text = (original / "alpha.bhv").read_text()
        goal = [60.0, -40.0]
    else:
        pose = config["initial_pose"]
        mission_text = re.sub(r"(?m)^\s*start_pos\s*=.*$", f"  start_pos = x={pose['x_m']}, y={pose['y_m']}, heading={pose['heading_deg']}, speed={pose['speed_mps']}", mission_text)
        goal = task["goal_m"]
        behavior_text = f"""// P5 local-ENU waypoint task; derived interface: s1_alpha and BHV_Waypoint.
initialize DEPLOY = false
initialize MOOS_MANUAL_OVERRIDE = false
Behavior = BHV_Waypoint
{{
  name = p5_task
  pwt = 100
  condition = DEPLOY = true
  condition = P5_TASK_ID = {task['id']}
  updates = WPT_UPDATE
  perpetual = false
  endflag = P5_TASK_DONE = {task['id']}
  endflag = DEPLOY = false
  speed = {config['target_speed_mps']}
  capture_radius = {config['arrival_tolerance_m']}
  slip_radius = {config['arrival_tolerance_m']}
  capture_line = false
  points = 190,-190
  repeat = 0
}}
"""
    mission = folder / "alpha.moos"
    mission.write_text(mission_text)
    (folder / "alpha.bhv").write_text(behavior_text)
    save(folder / "task.json", {"task": task, "example": example, "timeout_case": timeout, "port": port, "frame": config["coordinate_frame"]})
    environment = {**os.environ, "PATH": str(binaries) + os.pathsep + os.environ.get("PATH", "")}
    processes, handles, commands = [], [], []
    started = time.monotonic()
    try:
        for app in config["core_apps"]:
            handle = (folder / f"{app}.stdout.txt").open("w")
            handles.append(handle)
            command = [str(binaries / app), str(mission)]
            if app in ("uSimMarineV22", "pMarinePIDV22", "pHelmIvP"):
                command.append(f"--alias={app}")
            processes.append(subprocess.Popen(command, cwd=folder, env=environment, stdout=handle, stderr=subprocess.STDOUT))
            commands.append({"app": app, "pid": processes[-1].pid, "command": command})
            time.sleep(0.15)
        ready = query(binaries/"uQueryDB", mission, folder/"queries/ready", "NAV_X > -100000", 30, warp)
        save(folder / "ready.json", ready)
        if ready["returncode"] != 0:
            raise RuntimeError(f"MOOS NAV readiness failed: {folder}")
        pokes = [f"P5_TASK_ID:={task['id']}", f"P5_DISPATCH:={task['id']}"]
        if not example:
            pokes.append(f"WPT_UPDATE:=points={goal[0]},{goal[1]}")
        pokes += ["RETURN=false", "DEPLOY=true", "MOOS_MANUAL_OVERRIDE=false"]
        command = [str(binaries/"uPokeDB"), str(mission), *pokes]
        sent = subprocess.run(command, cwd=folder, env=environment, text=True, capture_output=True, timeout=20)
        (folder/"dispatch.stdout.txt").write_text(sent.stdout+sent.stderr)
        save(folder/"dispatch.json", {"command": command, "returncode": sent.returncode})
        if sent.returncode:
            raise RuntimeError(f"MOOS dispatch failed: {folder}")
        deadline = config["example_observation_sim_s"] if example else config["timeout_feedback_wait_sim_s"] if timeout else config["success_feedback_wait_sim_s"]
        condition = "NAV_X > 100000" if example else f"P5_TASK_DONE = {task['id']}"
        feedback = query(binaries/"uQueryDB", mission, folder/"queries/feedback", condition, deadline, warp)
        save(folder/"feedback.json", feedback)
        # Let pLogger receive and flush the final task/pose messages before shutdown.
        time.sleep(0.5)
    finally:
        # PID ownership is explicit; do not stop another user's MOOS mission.
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for handle in handles:
            handle.close()
        save(folder/"owned_processes.json", {"commands": commands,
             "returncodes": [p.returncode for p in processes], "all_stopped": all(p.poll() is not None for p in processes),
             "wall_elapsed_s": time.monotonic()-started})
    evidence = alog_evidence(folder, task["id"], start, goal, config["operating_envelope_m"])
    moved = math.hypot(evidence["last_pose"]["x_m"]-evidence["first_pose"]["x_m"], evidence["last_pose"]["y_m"]-evidence["first_pose"]["y_m"])
    reported = evidence["completion_pose"]
    complete = (feedback["returncode"] == 0 and evidence["completion_time_s"] is not None
                and reported is not None and reported["goal_distance_m"] < config["arrival_tolerance_m"]
                and any("behavior-name=p5_task,completed" in r["value"] for r in evidence["behavior_completion_messages"]))
    evidence.update(task_id=task["id"], status="EXAMPLE_MOVEMENT" if example and moved>1 else "COMPLETED" if complete else "TIMEOUT",
                    completed=complete, movement_displacement_m=moved, goal_m=goal, query_returncode=feedback["returncode"])
    save(folder/"result.json", evidence)
    return evidence


def run(run_id, moos_root, role):
    from usv_uuc_cooperation.main import assert_frozen_core
    assert_frozen_core()
    config_path = PACKAGE_ROOT / "configs/P5-MOOS-V01.json"
    config = json.loads(config_path.read_text())
    moos_root = Path(moos_root).expanduser().resolve()
    binaries = moos_root / "bin"
    required = [*config["core_apps"], "uPokeDB", "uQueryDB"]
    missing = [name for name in required if not (binaries/name).is_file()]
    if missing:
        raise FileNotFoundError(f"MOOS binaries missing from {binaries}: {missing}")
    for task in [*config["tasks"], config["timeout_task"]]:
        error = goal_error(task["goal_m"], config["operating_envelope_m"])
        if error:
            raise ValueError(f"{task['id']}: {error}")
    start = [config["initial_pose"]["x_m"], config["initial_pose"]["y_m"]]
    problem = {"boats": config["boats"], "tasks": [t["id"] for t in config["tasks"]], "capacity": [1],
               "reachable": [[True for _ in config["tasks"]]],
               "cost_m": [[math.dist(start, t["goal_m"]) for t in config["tasks"]]]}
    assigned = greedy(problem)
    if not assigned["feasible"] or len(assigned["trace"]) != 1:
        raise ValueError("P5 requires one feasible scheduler-generated task")
    chosen = next(t for t in config["tasks"] if t["id"] == assigned["trace"][0]["task"])
    folder = new_run_directory("P5", run_id)
    save(folder/"config.json", config)
    save(folder/"scheduler_problem.json", problem)
    save(folder/"scheduler_assignment.json", assigned)
    commit = subprocess.run(["git", "-C", str(moos_root), "rev-parse", "HEAD"], text=True, capture_output=True)
    save(folder/"metadata.json", {"framework": FRAMEWORK_VERSION, "config": config["config_version"], "operator_role": role,
         "learner_explanation_status": "NOT_COLLECTED_BY_PROGRAM", "moos_source_commit": commit.stdout.strip() if commit.returncode==0 else None,
         "binary_build_commit": "UNKNOWN; binary SHA recorded independently", "moos_root": str(moos_root),
         "binary_sha256": {name:hashlib.sha256((binaries/name).read_bytes()).hexdigest() for name in required},
         "original_example_sha256": {name:hashlib.sha256((moos_root/"ivp/missions/s1_alpha"/name).read_bytes()).hexdigest() for name in ("alpha.moos","alpha.bhv")}})
    print(f"MOOS original-example movement: {folder}", flush=True)
    example = run_case(folder/"example", moos_root, config, {"id":"P5-EXAMPLE","goal_m":[60,-40]}, example=True)
    print("Scheduler -> MOOS task and completion feedback", flush=True)
    success = run_case(folder/"success", moos_root, config, chosen)
    print("Real short feedback observation window: completion must remain false", flush=True)
    timed_out = run_case(folder/"timeout", moos_root, config, config["timeout_task"], timeout=True)
    invalid = config["invalid_task"]
    error = goal_error(invalid["goal_m"], config["operating_envelope_m"])
    rejected = {"task_id":invalid["id"],"status":"REJECTED" if error else "UNEXPECTEDLY_VALID","completed":False,
                "reason":error,"published_to_moos":False}
    save(folder/"invalid_target.json", rejected)
    # Execute the same task in the explicitly simpler 2D kinematic reference.
    goal = chosen["goal_m"]
    length = math.dist(start,goal)
    rows=[]
    for step in range(math.ceil(config["success_feedback_wait_sim_s"]/config["sample_dt_s"])+1):
        stamp=step*config["sample_dt_s"]
        distance=min(length,config["target_speed_mps"]*stamp)
        x=start[0]+(goal[0]-start[0])*distance/length
        y=start[1]+(goal[1]-start[1])*distance/length
        remaining=math.hypot(x-goal[0],y-goal[1])
        rows.append({"time_s":stamp,"x_m":x,"y_m":y,"goal_distance_m":remaining,"cross_track_m":0.0})
        if remaining <= config["arrival_tolerance_m"]:
            break
    write_rows(folder/"reference_2d.csv",rows)
    paired={"task_id":chosen["id"],"start_m":start,"goal_m":goal,"nominal_speed_mps":config["target_speed_mps"],
            "arrival_tolerance_m":config["arrival_tolerance_m"],
            "feedback_wait_semantics":"uQueryDB MOOS-time window from its own startup, not a strict dispatch-to-completion deadline",
            "initial_heading_deg":config["initial_pose"]["heading_deg"],
            "kinematic_2d":{"completion_time_s":rows[-1]["time_s"],"max_cross_track_m":0.0,"operating_envelope_violations":0},
            "moos":{"completion_time_s":success["completion_time_s"],"max_cross_track_m":success["maximum_cross_track_m"],"operating_envelope_violations":success["operating_envelope_violations"]},
            "limits":config["comparison_boundary"],"energy":"not measured; no real propulsion/battery claim",
            "violation_definition":"Recorded NAV pose outside frozen x/y operating envelope; no obstacle or inter-vessel collision claim"}
    save(folder/"paired_metrics.json",paired)
    checks={"example_actual_movement":example["status"]=="EXAMPLE_MOVEMENT",
            "scheduler_task_actual_completion":success["completed"],
            "timeout_no_false_completion":not timed_out["completed"] and timed_out["complete_alog_time_s"] is None,
            "invalid_goal_rejected_before_publish":rejected["status"]=="REJECTED",
            "actual_trajectory_recorded":all(c["sample_count"]>2 for c in (example,success,timed_out)),
            "operating_envelope":success["operating_envelope_violations"]==0}
    result={"engineering_status":"PASS" if all(checks.values()) else "FAIL","operator_role":role,"checks":checks,
            "completed_task_count":int(success["completed"]),"completion_time_s":success["completion_time_s"],
            "timeout_task_status":timed_out["status"],"invalid_task_status":rejected["status"],
            "learner_course_status":"REQUIRES_SEPARATE_USER_EXECUTION_AND_EXPLANATION_RECORD","vrx_status":"NOT_EXECUTED"}
    save(folder/"result.json",result)
    text=["# P5 MOOS real platform-simulator execution", "", f"Engineering status: **{result['engineering_status']}**; operator role: **{role}**.","",
          "| Task | Status | Completion time, simulated s |", "|---|---|---:|",
          f"| {chosen['id']} | {success['status']} | {success['completion_time_s']} |",
          f"| {timed_out['task_id']} | {timed_out['status']} | null |",
          f"| {rejected['task_id']} | {rejected['status']} | null |", "",
          "[Structured checks](result.json) · [Paired metrics](paired_metrics.json) · [Scheduler output](scheduler_assignment.json)","",
          "Real MOOSDB, uSimMarineV22, pMarinePIDV22, pHelmIvP and pLogger ran. Raw alog and per-app stdout are in each case folder.",
          "Completion requires the matching behavior event and actual goal proximity. Received target alone is not completion.","",
          config["comparison_boundary"],"",
          "Teacher engineering checks, learner execution and learner explanations are distinct. This program does not mark course explanations passed."]
    (folder/"摘要.md").write_text("\n".join(text)+"\n")
    print(json.dumps({"output":str(folder),**result},ensure_ascii=False),flush=True)
    return folder


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id",required=True)
    parser.add_argument("--moos-root",type=Path,default=PROJECT_ROOT/"moos-ivp")
    parser.add_argument("--role",choices=("teacher","learner"),default="teacher")
    args=parser.parse_args()
    run(args.run_id,args.moos_root,args.role)
