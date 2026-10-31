"""P3's original fixed heuristic and discrete executor, using modular core functions."""
import copy
import time

from usv_uuc_cooperation.config import PACKAGE_ROOT, FRAMEWORK_VERSION, read_json
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.scheduling.joint_scheduler import schedule
from usv_uuc_cooperation.agents.uuv import task_plan
from usv_uuc_cooperation.control.discrete_executor import simulate


def run(case_id):
    config = read_json(PACKAGE_ROOT / "configs/P3-V01.json")
    suite = read_json(PACKAGE_ROOT / "configs/P3-V01_cases.json")
    case = next((c for c in suite["cases"] if c["case_id"] == case_id), None)
    if case is None:
        raise ValueError(f"Unknown P3 case: {case_id}")
    effective = copy.deepcopy(config)
    blocked = forbidden_nodes(effective)
    stamp = time.perf_counter()
    sequences, decisions, failure = schedule(effective, blocked)
    if failure:
        raise ValueError(f"Planning failed under this fixed heuristic: {failure}")
    plans, legs = {}, {}
    for boat, sequence in sequences.items():
        plans[boat], legs[boat] = task_plan(boat, sequence, effective, blocked)
        if plans[boat] is None:
            raise ValueError(f"Selected route became unreachable: {boat}")
    planning_wall_time = time.perf_counter() - stamp
    budgets = {b: effective["energy"]["uuc_budget_NEU"] for b in effective["uuc_ids"]}
    budgets["USV"] = effective["energy"]["usv_budget_NEU"]
    budgets.update(case["budget_overrides_NEU"])
    summary, tasks, steps, links, snapshots = simulate(effective, sequences, plans, blocked, budgets)
    for boat in effective["uuc_ids"]:
        completed = [tasks[t]["completion_s"] for t in sequences[boat] if tasks[t]["status"] == "completed"]
        summary["boat_totals"][boat]["completion_time_s"] = (
            max(completed) if len(completed) == len(sequences[boat]) else None
        )
    summary["mission_status"] = (
        "ALL_TASKS_ON_TIME" if summary["deadline_met_task_count"] == len(tasks)
        and not summary["boat_totals"]["USV"]["failure"] else "RESTRICTED_OR_FAILED"
    )
    summary["total_uuc_distance_m"] = sum(summary["boat_totals"][b]["distance_m"] for b in effective["uuc_ids"])
    summary["total_all_boat_distance_m"] = sum(v["distance_m"] for v in summary["boat_totals"].values())
    summary["total_all_boat_energy_NEU"] = sum(v["energy_NEU"] for v in summary["boat_totals"].values())
    result = {
        "source_version": f"{FRAMEWORK_VERSION}/P3-V01",
        "mode": "1USV+5UUV+20tasks",
        "case": case, "config": effective, "effective_budgets_NEU": budgets,
        "planning_identity": "DETERMINISTIC_HEURISTIC", "optimality_claim": None,
        "sequences": sequences, "scheduling_decisions": decisions, "legs": legs,
        "tasks": tasks, "summary": summary, "snapshots": snapshots,
        "run_metadata": {"planning_wall_time_s": planning_wall_time,
            "planning_time_scope": "greedy candidate A* calls plus final route/service plans; excludes simulation and output",
            "role": "framework reproduction; does not create new learner acceptance"},
    }
    return result, steps, links
