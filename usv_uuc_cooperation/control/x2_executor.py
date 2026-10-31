"""X2 observed-graph extension of the frozen V08 integrated executor.

Original P4 executor remains byte-identical; movement/service/relay rules copied
from that accepted source. X2 changes only graph observation and planning calls.
"""
import copy
import math
import time

from usv_uuc_cooperation.control.current_executor import waypoint_candidate
from usv_uuc_cooperation.env.acoustic_comm import communication_graph
from usv_uuc_cooperation.env.obstacle import forbidden_nodes, unsafe_reason
from usv_uuc_cooperation.metrics.evaluator import metrics
from usv_uuc_cooperation.planning.continuous_routes import nearest_order, rebuild

STRATEGIES = ("STATIC_FROZEN", "DYNAMIC_REPLAN")


def current_at(config, time_s):
    t = min(time_s, config["cutoff_time_s"] - 1)
    for row in config["current_schedule"]:
        if row["start_s"] <= t < row["end_s"]:
            return list(row["vector_m_per_s"])
    raise ValueError(f"No explicit flow at {t}")


def simulate(config, assignment, strategy, planner, *, rebuild_routes=None):
    if strategy not in STRATEGIES:
        raise ValueError(strategy)
    rebuild_routes = rebuild if rebuild_routes is None else rebuild_routes
    config = copy.deepcopy(config)
    assert config["dt_s"] == 1 and config["cutoff_time_s"] == config["observation_duration_s"]
    ids, horizon = config["uuc_ids"], config["cutoff_time_s"]
    assert all(math.hypot(*current_at(config, t)) <= .2 + 1e-12 for t in range(horizon))
    blocked = forbidden_nodes(config)
    # The environment observes scheduled events; planning never receives these tables.
    physical = {k: config[k] for k in ("map", "uuc_ids", "starts_m", "tasks_m", "energy",
                                      "capacity_per_uuc", "service_s_per_task")}
    states = {b: {"position_m": list(config["starts_m"][b]), "energy_NEU": 0.0, "distance_m": 0.0,
                  "budget_NEU": config["energy"]["usv_budget_NEU" if b == "USV" else "uuc_budget_NEU"],
                  "failure": None} for b in ids + ["USV"]}

    def new_task(position, released, owner=None):
        assert unsafe_reason(position, position, config, blocked) is None
        return {"position_m": list(position), "release_s": released, "boat_id": owner,
                "initial_boat_id": owner, "status": "pending", "arrival_s": None,
                "service_progress_s": 0, "completion_s": None}

    tasks = {task: new_task(config["tasks_m"][task], 0, b) for b in ids for task in assignment[b]}
    assert sorted(tasks) == sorted(config["tasks_m"])
    accepted = {b: set(assignment[b]) for b in ids}
    queues = copy.deepcopy(config['x2']['initial_queues'])
    assert all(set(queues[b]) == set(assignment[b]) and len(queues[b]) == len(assignment[b]) for b in ids)
    initial_queues = copy.deepcopy(queues)
    planner.context(0, 'INITIAL')
    queues, plans, initial = rebuild_routes(physical, blocked, states, tasks, accepted, queues, 0, "INITIAL", False, planner)
    if 'scan_ordering' in initial:
        initial_queues = copy.deepcopy(queues)
    replans, history, steps, links, event_log = [initial], [], [], [], []
    previous, ages = {b: True for b in ids}, {b: 0 for b in ids}
    observed = set()
    graph_version = 0
    permanent_blocked = set(blocked)

    def snap():
        return {"boats": copy.deepcopy(states), "tasks": copy.deepcopy(tasks), "queues": copy.deepcopy(queues),
                "accepted_task_ids": {b: sorted(v) for b, v in accepted.items()},
                "remaining_plans": copy.deepcopy(plans)}

    for t in range(horizon + 1):
        due = sorted((e for e in config["events"] if e["time_s"] == t),
                     key=lambda e: (0 if e["type"] == "UUV_FAILURE" else 1, e["event_id"]))
        before = snap() if due else None
        old_plans = None
        if due:
            for event in due:
                assert event["event_id"] not in observed
                observed.add(event["event_id"])
                detail = {}
                if event["type"] == "TASK_ARRIVAL":
                    assert event["task_id"] not in tasks
                    tasks[event["task_id"]] = new_task(event["position_m"], t)
                elif event["type"] == "UUV_FAILURE":
                    boat = event["boat_id"]
                    assert boat in ids and not states[boat]["failure"]
                    states[boat]["failure"] = {"reason": "task_capability_fault", "time_s": t,
                                               "hold_policy": "ideal_station_keeping_compensation"}
                    interrupted = []
                    for task, item in tasks.items():
                        if item["boat_id"] == boat and item["status"] != "completed":
                            interrupted.append({"task_id": task, "previous_progress_s": item["service_progress_s"],
                                                "previous_status": item["status"], "reset_progress_s": 0})
                            item.update(status="orphaned", arrival_s=None, service_progress_s=0)
                    queues[boat], plans[boat] = [], []
                    detail = {"interrupted_tasks": interrupted, "position_m": list(states[boat]["position_m"]),
                              "retained_energy_NEU": states[boat]["energy_NEU"]}
                elif event['type'] == 'MAP_UPDATE':
                    added, removed = set(map(tuple, event['add_cells'])), set(map(tuple, event['remove_cells']))
                    assert not (added & blocked) and removed <= (blocked - permanent_blocked)
                    blocked = (blocked | added) - removed
                    assert all(unsafe_reason(s['position_m'], s['position_m'], config, blocked) is None for s in states.values()), 'Map update would newly occupy a boat'
                    graph_version += 1
                    planner.observe(physical, blocked, graph_version)
                    detail = {'graph_version': graph_version, 'graph_sha256': planner.graph.sha256}
                else:
                    raise ValueError(event["type"])
                event_log.append({**event, **detail, "observed_at_s": t, "processed_at_s": t})
            if strategy == "DYNAMIC_REPLAN":
                old_plans = before["remaining_plans"]
                stamp = time.perf_counter()
                inputs = snap()
                reason = '+'.join(e['event_id'] for e in due)
                planner.context(t, reason)
                assign_new = any(e['type'] != 'MAP_UPDATE' for e in due)
                queues, plans, decision = rebuild_routes(physical, blocked, states, tasks, accepted, queues,
                                                  t, reason, assign_new, planner)
                decision.update(input_state=inputs, wall_time_s=time.perf_counter() - stamp,
                                old_plans=old_plans, protected_services=[task for task, item in before["tasks"].items()
                                    if item["service_progress_s"] > 0 and item["status"] != "completed"
                                    and not states[item["boat_id"]]["failure"]])
                decision.update(graph_version=graph_version, graph_sha256=planner.graph.sha256)
                replans.append(decision)
        positions = {b: list(states[b]["position_m"]) for b in ids}
        usv = list(states["USV"]["position_m"])
        graph, distances = communication_graph(usv, positions, config["communication"]["radius_m"])
        connected = {b: graph.has_edge("USV", b) for b in ids}
        current = current_at(config, t)
        history.append({"time_s": t, **snap(), "before_events": before, "connected": connected,
                        "current_m_per_s": current, "event_ids": [e["event_id"] for e in due],
                        "old_plans": old_plans, "replanning_count": len(replans) - 1})
        history[-1].update(graph_version=graph_version, observed_blocked_cells=[list(p) for p in sorted(blocked)])
        if t == horizon:
            break
        for b in ids:
            ages[b] = 0 if connected[b] or previous[b] else ages[b] + 1
            links.append({"time_s": t, "interval_end_s": t + 1, "boat_id": b,
                          "distance_m": distances[b], "connected": connected[b], "outage_age_s": ages[b]})
        lost = [b for b in ids if not connected[b]]
        relay_boat = min(lost, key=lambda b: (-ages[b], distances[b], b)) if lost else None
        target = positions[relay_boat] if relay_boat else usv
        for b in ids + ["USV"]:
            state = states[b]
            start, task, waypoint = list(state["position_m"]), None, None
            count_before = len(plans[b]) if b in plans else None
            if state["failure"]:
                action, candidate = "failed_hold", start
            elif b == "USV":
                candidate = waypoint_candidate(start, target, current, config["speed_m_per_s"][b], False)
                action = "relay_move" if candidate != start else "relay_hold"
            elif plans[b]:
                op = plans[b][0]
                action, task, waypoint = op["action"], op["task_id"], list(op["end"])
                if action == "move":
                    candidate = waypoint_candidate(start, waypoint, current, config["speed_m_per_s"][b])
                else:
                    assert start == waypoint == tasks[task]["position_m"], "Actual arrival required before service"
                    candidate = start
            else:
                action, candidate = "finished_hold", start
            coefficient = config["energy"]["usv_coefficient_NEU_per_m" if b == "USV" else "uuc_coefficient_NEU_per_m"]
            energy_before = state["energy_NEU"]
            predicted = energy_before + coefficient * math.dist(start, candidate)
            safe_error = unsafe_reason(start, candidate, config, blocked)
            budget_ok = predicted <= state["budget_NEU"] + 1e-9
            rejection, event_end = None, None
            progress_before = tasks[task]["service_progress_s"] if task else None
            if safe_error:
                end, action, rejection = start, "safety_rejected", safe_error
            elif not budget_ok:
                end, action, rejection = start, "budget_rejected", "insufficient_movement_budget"
            else:
                end = list(candidate)
                if task:
                    item = tasks[task]
                    assert item["boat_id"] == b and item["status"] != "completed"
                    if action == "move" and end == waypoint:
                        plans[b].pop(0)
                        if end == item["position_m"]:
                            item.update(status="arrived", arrival_s=t + 1)
                            event_end = "arrived"
                    elif action == "service":
                        if item["arrival_s"] is None:
                            item["arrival_s"] = t
                        item["service_progress_s"] += 1
                        item["status"] = "servicing"
                        plans[b].pop(0)
                        if item["service_progress_s"] == config["service_s_per_task"]:
                            item.update(status="completed", completion_s=t + 1)
                            assert queues[b].pop(0) == task
                            event_end = "completed"
            length = math.dist(start, end)
            ground = [end[k] - start[k] for k in (0, 1)]
            water = [ground[k] - current[k] for k in (0, 1)]
            assert math.hypot(*water) <= config["speed_m_per_s"][b] + 1e-9
            assert unsafe_reason(start, end, config, blocked) is None
            state.update(position_m=end, distance_m=state["distance_m"] + length,
                         energy_NEU=energy_before + coefficient * length)
            assert state["energy_NEU"] <= state["budget_NEU"] + 1e-9
            steps.append({"time_s": t, "next_time_s": t + 1, "boat_id": b, "action": action,
                          "task_id": task, "start_m": start, "candidate_m": candidate, "end_m": end,
                          "waypoint_m": waypoint, "plan_count_before": count_before,
                          "plan_count_after": len(plans[b]) if b in plans else None,
                          "current_m_per_s": current, "water_m_per_s": water, "ground_m_per_s": ground,
                          "actual_distance_m": length, "energy_before_NEU": energy_before,
                          "predicted_total_NEU": predicted, "delta_energy_NEU": coefficient * length,
                          "energy_NEU": state["energy_NEU"], "remaining_NEU": state["budget_NEU"] - state["energy_NEU"],
                          "budget_NEU": state["budget_NEU"], "budget_check_passed": budget_ok,
                          "service_progress_before_s": progress_before,
                          "service_progress_after_s": tasks[task]["service_progress_s"] if task else None,
                          "rejection_reason": rejection, "event_at_end": event_end,
                          "relay_target_boat": relay_boat if b == "USV" else None})
        previous = connected
    completions = [x["completion_s"] for x in tasks.values() if x["status"] == "completed"]
    per_boat = {b: metrics([r for r in links if r["boat_id"] == b]) for b in ids}
    summary = {"status": "SUCCESS" if len(completions) == len(tasks) else "TASKS_INCOMPLETE",
               "completed_task_count": len(completions), "known_task_count": len(tasks),
               "team_completion_time_s": max(completions) if len(completions) == len(tasks) else None,
               "cutoff_time_s": horizon, "total_energy_NEU": sum(s["energy_NEU"] for s in states.values()),
               "energy_observation_scope": f"all six boats over [0,{horizon}), including relay after task completion",
               "boat_totals": states, "accepted_task_ids": {b: sorted(v) for b, v in accepted.items()},
               "replanning_count": len(replans) - 1, "unassigned_or_orphaned": sorted(t for t, x in tasks.items()
                   if x["status"] != "completed" and (x["boat_id"] is None or states[x["boat_id"]]["failure"])),
               "communication": {"observed_boat_seconds": len(ids) * horizon,
                                 "connected_boat_seconds": sum(r["connected_boat_seconds"] for r in per_boat.values()),
                                 "retention_rate": sum(r["connected_boat_seconds"] for r in per_boat.values()) / (len(ids) * horizon),
                                 "per_boat": per_boat},
               "actual_global_dynamic_optimality_claim": None, "global_infeasibility_proved": False,
               "boat_collision_modeled": False}
    initial.update(graph_version=0, graph_sha256=planner.graphs[0]['sha256'])
    return {"implementation": "X2-OBSERVED-EXECUTOR-V01", "strategy": strategy, "config": config,
            "planner_config_keys": sorted(physical), "initial_queues": initial_queues, "summary": summary, "tasks": tasks,
            "steps": steps, "links": links, "snapshots": history, "event_log": event_log, "replans": replans,
            "algorithm": planner.algorithm, "observed_graphs": planner.graphs}
