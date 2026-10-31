"""Safe connectors from actual continuous positions to the accepted A* grid."""
import math
from usv_uuc_cooperation.planning.astar import astar
from usv_uuc_cooperation.env.obstacle import unsafe_reason


def connected_route(start, goal, config, blocked, planner=None):
    start, goal = list(start), list(goal)
    if unsafe_reason(start, start, config, blocked) or unsafe_reason(goal, goal, config, blocked):
        return None
    if any(float(x) != int(x) for x in goal):
        raise ValueError("Task destinations must lie on the frozen 1m grid")
    if start == goal and planner is None:
        result = {"path": [start], "distance_m": 0.0, "connector_m": 0.0}
        if "adjacency" in config["map"]:
            result.update(adjacency=config["map"]["adjacency"], grid_path=[start], grid_distance_m=0.0)
        return result
    candidates = []
    for x in range(math.floor(start[0]), math.ceil(start[0]) + 1):
        for y in range(math.floor(start[1]), math.ceil(start[1]) + 1):
            node = [x, y]
            if unsafe_reason(start, node, config, blocked):
                continue
            if planner is not None:
                grid = planner(node, goal, config, blocked)
            elif "adjacency" in config["map"]:
                from usv_uuc_cooperation.planning.grid_planner import plan
                grid = plan(node, goal, config, blocked, adjacency=config["map"]["adjacency"])
            else:
                grid = astar(node, goal, config["map"], blocked)
            if grid is not None:
                distance = math.dist(start, node)
                candidates.append((distance + grid["distance_m"], distance, x, y, grid))
    if not candidates:
        return None
    length, connector, _, _, grid = min(candidates, key=lambda v: v[:4])
    path = [start] + (grid["path"] if start != grid["path"][0] else grid["path"][1:])
    assert all(unsafe_reason(a, z, config, blocked) is None for a, z in zip(path, path[1:]))
    result = {"path": path, "distance_m": length, "connector_m": connector}
    if "adjacency" in config["map"]:
        result.update(adjacency=config["map"]["adjacency"], grid_path=grid["path"], grid_distance_m=grid["distance_m"])
    if planner is not None:
        result['search_query_id'] = grid['search_query_id']
    return result


def nearest_order(boat, assigned, config, blocked):
    tail, remaining, order = list(config["starts_m"][boat]), set(assigned), []
    while remaining:
        candidates = []
        for task in sorted(remaining):
            route = connected_route(tail, config["tasks_m"][task], config, blocked)
            if route is not None:
                candidates.append((route["distance_m"], task))
        if not candidates:
            raise ValueError(f"No safe route for {boat}: {sorted(remaining)}")
        _, task = min(candidates)
        order.append(task)
        remaining.remove(task)
        tail = config["tasks_m"][task]
    return order


def rebuild(config, blocked, states, tasks, accepted, queues, time_s, reason, assign_new, planner=None,
            *, allow_planning_holds=False):
    """Only observed tasks/state; no future events or flow schedule are accepted."""
    if "events" in config or "current_schedule" in config:
        raise ValueError("Planner must not receive future event/flow tables")
    coefficient = config["energy"]["uuc_coefficient_NEU_per_m"]
    plans, output, tails, projected = {}, {}, {}, {}
    assignments, legs, candidates = [], [], []
    holds = {}

    def append(boat, task, route):
        item = tasks[task]
        old = item["boat_id"]
        if old != boat:
            item.update(boat_id=boat, status="pending", arrival_s=None, service_progress_s=0)
        accepted[boat].add(task)
        output[boat].append(task)
        for point in route["path"][1:]:
            plans[boat].append({"action": "move", "task_id": task, "end": list(point)})
        plans[boat].extend({"action": "service", "task_id": task, "end": list(item["position_m"])}
                           for _ in range(config["service_s_per_task"] - item["service_progress_s"]))
        projected[boat] += coefficient * route["distance_m"]
        tails[boat] = list(item["position_m"])
        legs.append({"boat_id": boat, "task_id": task, **route})
        assignments.append({"task_id": task, "boat_id": boat, "old_owner": old,
                            "service_progress_s": item["service_progress_s"],
                            "accepted_count": len(accepted[boat]), "projected_total_NEU": projected[boat]})

    for boat in config["uuc_ids"]:
        plans[boat], output[boat] = [], []
        tails[boat] = list(states[boat]["position_m"])
        projected[boat] = states[boat]["energy_NEU"]
        if states[boat]["failure"]:
            continue
        retained = [t for t in queues[boat] if tasks[t]['status'] != 'completed']
        for index, task in enumerate(retained):
            item = tasks[task]
            if item["status"] == "completed":
                continue
            assert item["boat_id"] == boat
            if hasattr(planner, 'route_context'):
                planner.route_context(boat, task, 'ACTUAL_HEAD' if not output[boat] else 'LOOKAHEAD_TAIL')
            route = connected_route(tails[boat], item["position_m"], config, blocked, planner)
            if allow_planning_holds and (route is None or projected[boat] + coefficient * route['distance_m'] > states[boat]['budget_NEU'] + 1e-9):
                holds[boat] = {'first_task_id': task, 'held_task_ids': list(retained[index:]),
                    'reason': 'NO_FEASIBLE_ROUTE_WITHIN_SEARCH_BUDGET' if route is None else 'PREFIX_ENERGY_BUDGET',
                    'executable_task_ids': list(output[boat]), 'global_infeasibility_proved': False}
                output[boat].extend(retained[index:])
                break
            if route is None:
                raise ValueError(f"Existing queue cannot safely connect: {boat}/{task}")
            if item["service_progress_s"]:
                assert tails[boat] == item["position_m"] and not output[boat]
            append(boat, task, route)
        assert len(accepted[boat]) <= config["capacity_per_uuc"]
        if projected[boat] > states[boat]["budget_NEU"] + 1e-9:
            raise ValueError(f"Complete retained route exceeds budget: {boat}")

    pending = {t for t, v in tasks.items() if v["status"] != "completed"
               and (v["boat_id"] is None or states[v["boat_id"]]["failure"])}
    if assign_new:
        while pending:
            eligible = []
            for boat in config["uuc_ids"]:
                if states[boat]["failure"] or boat in holds:
                    continue
                for task in sorted(pending):
                    if hasattr(planner, 'route_context'):
                        planner.route_context(boat, task, 'DISPATCH_CANDIDATE')
                    route = connected_route(tails[boat], tasks[task]["position_m"], config, blocked, planner)
                    cap_ok = len(accepted[boat] | {task}) <= config["capacity_per_uuc"]
                    predicted = projected[boat] + coefficient * route["distance_m"] if route else None
                    budget_ok = predicted is not None and predicted <= states[boat]["budget_NEU"] + 1e-9
                    candidates.append({"boat_id": boat, "task_id": task, "from_m": list(tails[boat]),
                                       "capacity_ok": cap_ok, "budget_ok": budget_ok,
                                       "route_distance_m": route["distance_m"] if route else None,
                                       "projected_before_NEU": projected[boat], "predicted_total_NEU": predicted})
                    if cap_ok and budget_ok:
                        eligible.append((route["distance_m"], boat, task, route))
            if not eligible:
                break
            _, boat, task, route = min(eligible, key=lambda v: v[:3])
            append(boat, task, route)
            pending.remove(task)
    decision = {"time_s": time_s, "reason": reason, "assign_new": assign_new,
                           "observed_task_ids": sorted(tasks), "assignments": assignments,
                           "candidate_checks": candidates, "legs": legs,
                           "output_queues": output, "projected_total_NEU": projected,
                           "unassigned_tasks": sorted(pending), "global_infeasibility_proved": False}
    if allow_planning_holds:
        decision['planning_holds'] = holds
        decision['budget_scope'] = 'executable prefix only; held tails are unplanned and retain ownership'
    return output, plans, decision
