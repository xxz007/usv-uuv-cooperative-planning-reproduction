"""Incremental checks for the combined executor; saved artifacts, no solver rerun."""
from collections import deque
import copy
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes


def safe_segment(a, b, config, blocked):
    # Independent slab intersection, including boundary contact.
    grid = config["map"]
    if any(not grid["x_range"][0] <= p[0] <= grid["x_range"][1]
           or not grid["y_range"][0] <= p[1] <= grid["y_range"][1] for p in (a, b)):
        return False
    for cell in blocked:
        lo, hi = 0.0, 1.0
        for k in (0, 1):
            delta = b[k] - a[k]
            lower, upper = cell[k] - .5, cell[k] + .5
            if abs(delta) < 1e-12:
                if a[k] < lower - 1e-12 or a[k] > upper + 1e-12:
                    hi = -1.0
                    break
            else:
                x, y = (lower - a[k]) / delta, (upper - a[k]) / delta
                lo, hi = max(lo, min(x, y)), min(hi, max(x, y))
        if lo <= hi + 1e-12:
            return False
    return True


def check_initial(config, problem, solved):
    assert solved["has_feasible_solution"] and solved["variables_loaded"]
    assert len(problem["boats"]) == 5 and len(problem["tasks"]) == 20
    assert problem["capacity"] == [config["capacity_per_uuc"]] * 5
    assert problem["tasks_m"] == config["tasks_m"] and problem["map"] == config["map"]
    blocked = forbidden_nodes(config)
    grid = config["map"]
    for i, boat in enumerate(problem["boats"]):
        start = tuple(config["starts_m"][boat]); distances = {start: 0}; work = deque([start])
        while work:
            x, y = work.popleft()
            for node in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if node in distances or node in blocked:
                    continue
                if grid["x_range"][0] <= node[0] <= grid["x_range"][1] and grid["y_range"][0] <= node[1] <= grid["y_range"][1]:
                    distances[node] = distances[(x, y)] + 1
                    work.append(node)
        for j, task in enumerate(problem["tasks"]):
            length = distances.get(tuple(config["tasks_m"][task]))
            assert problem["reachable"][i][j] == (length is not None)
            if length is not None:
                assert problem["cost_m"][i][j] == length
                path = problem["start_task_routes"][f"{boat}->{task}"]["path"]
                assert path[0] == list(start) and path[-1] == config["tasks_m"][task]
                assert len(path) - 1 == length and all(safe_segment(a, b, config, blocked) for a, b in zip(path, path[1:]))
    matrix = solved["matrix"]
    assert len(matrix) == 5 and all(len(row) == 20 for row in matrix)
    assert all(min(abs(x), abs(x - 1)) < 1e-7 for row in matrix for x in row)
    assert all(abs(sum(row[j] for row in matrix) - 1) < 1e-7 for j in range(20))
    assert all(sum(row) <= config["capacity_per_uuc"] for row in matrix)
    assert all(x <= int(r) for row, flags in zip(matrix, problem["reachable"]) for x, r in zip(row, flags))
    total = sum(x * cost for row, costs in zip(matrix, problem["cost_m"]) for x, cost in zip(row, costs))
    assert math.isclose(total, solved["objective_m"], abs_tol=1e-8)
    if solved["status"] == "OPTIMAL":
        assert solved["termination_condition"] == "optimal" and math.isclose(total * 1000, solved["objective_bound_int"], abs_tol=1e-7)
    assert sorted(t for v in solved["assignment"].values() for t in v) == problem["tasks"]
    return {"status": "PASS", "independent_BFS_cost_reach_pairs": 100,
            "all_binary_cover_capacity_reach_objective_checks": "PASS", "objective_m": total,
            "scope": "static start-to-task assignment only; no exhaustive5^20 or route optimality claim"}


def check_result(result):
    config, snaps = result["config"], result["snapshots"]
    blocked = forbidden_nodes(config)
    ids, horizon = config["uuc_ids"] + ["USV"], config["cutoff_time_s"]
    assert [s["time_s"] for s in snaps] == list(range(horizon + 1))
    assert len(result["steps"]) == len(ids) * horizon
    cumulative = {b: 0.0 for b in ids}
    completions = {}
    nonzero, partial_move, connectors = 0, 0, 0
    for row in result["steps"]:
        t, b = row["time_s"], row["boat_id"]
        assert row["start_m"] == snaps[t]["boats"][b]["position_m"]
        assert row["end_m"] == snaps[t + 1]["boats"][b]["position_m"]
        assert safe_segment(row["start_m"], row["end_m"], config, blocked)
        assert row["current_m_per_s"] == snaps[t]["current_m_per_s"]
        length = math.dist(row["start_m"], row["end_m"])
        assert math.isclose(length, row["actual_distance_m"], abs_tol=1e-9)
        for k in (0, 1):
            assert math.isclose(row["water_m_per_s"][k] + row["current_m_per_s"][k], row["ground_m_per_s"][k], abs_tol=1e-9)
            assert math.isclose(row["ground_m_per_s"][k], row["end_m"][k] - row["start_m"][k], abs_tol=1e-9)
        assert math.hypot(*row["water_m_per_s"]) <= config["speed_m_per_s"][b] + 1e-9
        coefficient = config["energy"]["usv_coefficient_NEU_per_m" if b == "USV" else "uuc_coefficient_NEU_per_m"]
        assert math.isclose(row["energy_before_NEU"], cumulative[b], abs_tol=1e-8)
        cumulative[b] += coefficient * length
        assert math.isclose(row["energy_NEU"], cumulative[b], abs_tol=1e-8) and cumulative[b] <= row["budget_NEU"] + 1e-8
        if row["action"] == "move" and row["end_m"] != row["waypoint_m"]:
            assert row["plan_count_before"] == row["plan_count_after"]
            partial_move += 1
        if row["action"] == "service":
            item = snaps[t]["tasks"][row["task_id"]]
            assert row["start_m"] == row["end_m"] == item["position_m"]
            assert row["service_progress_before_s"] == item["service_progress_s"]
            assert row["service_progress_after_s"] == row["service_progress_before_s"] + 1
        if row["event_at_end"] == "completed":
            task = row["task_id"];assert task not in completions
            same = [r for r in result["steps"] if r["boat_id"] == b and r["task_id"] == task and r["action"] == "service" and t - config["service_s_per_task"] + 1 <= r["time_s"] <= t]
            assert [r["time_s"] for r in same] == list(range(t - config["service_s_per_task"] + 1, t + 1))
            completions[task] = t + 1
        if row["current_m_per_s"] != [0, 0]:
            nonzero += 1
    for t, snap in enumerate(snaps):
        expected = set(config["tasks_m"]) | {e["task_id"] for e in config["events"] if e["type"] == "TASK_ARRIVAL" and e["time_s"] <= t}
        assert set(snap["tasks"]) == expected
        for b in config["uuc_ids"]:
            accepted = set(snap["accepted_task_ids"][b]);assert len(accepted) <= config["capacity_per_uuc"]
            if t:
                assert set(snaps[t - 1]["accepted_task_ids"][b]) <= accepted
        for b in config["uuc_ids"]:
            distance = math.dist(snap["boats"][b]["position_m"], snap["boats"]["USV"]["position_m"])
            assert snap["connected"][b] == (distance <= config["communication"]["radius_m"])
    for decision in result["replans"]:
        for leg in decision["legs"]:
            path = leg["path"]
            assert all(safe_segment(a, b, config, blocked) for a, b in zip(path, path[1:]))
            assert math.isclose(sum(math.dist(a, b) for a, b in zip(path, path[1:])), leg["distance_m"], abs_tol=1e-8)
            if leg["connector_m"]:
                connectors += 1
        for boat, projected in decision["projected_total_NEU"].items():
            assert projected <= config["energy"]["uuc_budget_NEU"] + 1e-8
        for task in decision.get("protected_services", []):
            assert decision["input_state"]["tasks"][task]["service_progress_s"] > 0
            before = snaps[decision["time_s"]]["before_events"]["tasks"][task]
            after = snaps[decision["time_s"]]["tasks"][task]
            assert before["boat_id"] == after["boat_id"] and before["service_progress_s"] == after["service_progress_s"]
    for e in result["event_log"]:
        assert e["observed_at_s"] == e["processed_at_s"] == e["time_s"]
        if e["type"] == "UUV_FAILURE":
            t, boat = e["time_s"], e["boat_id"]
            assert all(s["boats"][boat]["position_m"] == snaps[t]["boats"][boat]["position_m"] for s in snaps[t:])
            for item in e["interrupted_tasks"]:
                assert item["reset_progress_s"] == 0
            for task, item in snaps[t]["before_events"]["tasks"].items():
                if item["status"] == "completed":
                    assert snaps[t]["tasks"][task] == item
    summary = result["summary"]
    assert summary["completed_task_count"] == len(completions) and summary["known_task_count"] == len(snaps[-1]["tasks"])
    assert summary["team_completion_time_s"] == (max(completions.values()) if len(completions) == summary["known_task_count"] else None)
    assert math.isclose(sum(cumulative.values()), summary["total_energy_NEU"], abs_tol=1e-8)
    assert summary["communication"]["observed_boat_seconds"] == len(config["uuc_ids"]) * horizon
    return {"status": "PASS", "actual_step_rows": len(result["steps"]), "nonzero_current_rows": nonzero,
            "partial_moves_with_no_plan_pop": partial_move, "safe_connector_legs": connectors,
            "known_tasks": summary["known_task_count"], "completed": len(completions),
            "scope": "combined actual state/velocity/segment/ledger/quota/event/service checks; no prior lesson retest"}


def combination_probes(config):
    from usv_uuc_cooperation.control.integrated_executor import simulate
    from usv_uuc_cooperation.planning.continuous_routes import connected_route
    c = copy.deepcopy(config)
    c.update(uuc_ids=["U1"], cutoff_time_s=5, observation_duration_s=5, tasks_m={"P": [3, 2]}, events=[])
    c["starts_m"] = {"U1": [2, 2], "USV": [2, 3]}
    c["current_schedule"] = [{"start_s": 0, "end_s": 5, "vector_m_per_s": [-.1, 0]}]
    r = simulate(c, {"U1": ["P"]}, "DYNAMIC_REPLAN")
    assert r["snapshots"][1]["boats"]["U1"]["position_m"] == [2.9, 2.0]
    assert r["snapshots"][1]["tasks"]["P"]["service_progress_s"] == 0
    assert r["tasks"]["P"]["completion_s"] == 4
    c = copy.deepcopy(config)
    c.update(uuc_ids=["U1", "U2"], cutoff_time_s=8, observation_duration_s=8, tasks_m={"P": [2, 2], "Q": [2, 3]},
             events=[{"event_id": "PROBE_FAULT", "time_s": 1, "type": "UUV_FAILURE", "boat_id": "U1"}])
    c["starts_m"] = {"U1": [2, 2], "U2": [2, 3], "USV": [3, 3]}
    c["current_schedule"] = [{"start_s": 0, "end_s": 8, "vector_m_per_s": [.1, 0]}]
    r = simulate(c, {"U1": ["P"], "U2": ["Q"]}, "DYNAMIC_REPLAN")
    assert r["event_log"][0]["interrupted_tasks"][0]["previous_progress_s"] == 1
    assert r["tasks"]["P"]["boat_id"] == "U2" and r["tasks"]["Q"]["completion_s"] == 2
    assert r["tasks"]["P"]["completion_s"] == 6 and set(r["summary"]["accepted_task_ids"]["U1"]) == {"P"}
    route = connected_route([4.2, 13], [9, 11], config, forbidden_nodes(config))
    assert route and route["path"][0] == [4.2, 13] and route["connector_m"] > 0
    assert all(safe_segment(a, b, config, forbidden_nodes(config)) for a, b in zip(route["path"], route["path"][1:]))
    return {"status": "PASS", "label": "teacher incremental component probes, not extra main comparison groups",
            "checks": ["adverse flow partial waypoint does not pop or service early", "nonzero-flow partial service reset/receiver full2s",
                       "healthy service preserved during event", "continuous safe connector starts at actual non-grid position", "lifetime slot retained after transfer"]}
