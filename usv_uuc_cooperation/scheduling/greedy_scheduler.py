"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P2/P2-L2/源码/p2_l2_baselines.py
"""
import math
from itertools import product



def check(owners, case):
    """owners 的四个位置依次表示 A/B/C/D 的负责艇下标。"""
    coverage = {task: int(owner in range(len(case["boats"])))
                for task, owner in zip(case["tasks"], owners)}
    counts = [owners.count(i) for i in range(len(case["boats"]))]
    capacity_ok = [n <= cap for n, cap in zip(counts, case["capacity"])]
    reachable_ok = all(owner is not None and case["reachable"][owner][j]
                       for j, owner in enumerate(owners))
    valid = (len(owners) == len(case["tasks"]) and
             all(n == 1 for n in coverage.values()) and
             all(capacity_ok) and reachable_ok)
    cost = sum(case["cost_m"][owner][j] for j, owner in enumerate(owners)) if valid else None
    return {"owners": owners, "coverage": coverage, "boat_task_counts": counts,
            "capacity_ok": capacity_ok, "reachable_ok": reachable_ok,
            "feasible": valid, "cost_m": cost}


def greedy(case):
    owners = [None] * len(case["tasks"])
    remaining = case["capacity"].copy()
    trace = []
    while None in owners:
        candidates = [(case["cost_m"][i][j], j, i)
                      for j, owner in enumerate(owners) if owner is None
                      for i in range(len(case["boats"]))
                      if remaining[i] > 0 and case["reachable"][i][j]]
        if not candidates:
            break
        cost, j, i = min(candidates)  # 依次按成本、任务、艇排序。
        owners[j] = i
        remaining[i] -= 1
        trace.append({"boat": case["boats"][i], "task": case["tasks"][j],
                      "cost_m": cost, "remaining_capacity": remaining.copy()})
    return {"trace": trace, **check(owners, case)}


def enumerate_all(case):
    # 每任务独立选择一艇，覆盖全部 2^4 个归属向量，再过滤容量与可达性。
    candidates = [check(list(owners), case) for owners in
                  product(range(len(case["boats"])), repeat=len(case["tasks"]))]
    feasible = [row for row in candidates if row["feasible"]]
    best = min((row["cost_m"] for row in feasible), default=None)
    optimal = [row for row in feasible if math.isclose(row["cost_m"], best, rel_tol=0, abs_tol=1e-9)]
    return {"candidate_count": len(candidates), "feasible_count": len(feasible),
            "candidates": candidates, "feasible_assignments": feasible,
            "optimal_cost_m": best, "optimal_assignments": optimal}


def describe(row, case):
    return "; ".join(f"{boat}={','.join(task for task, owner in zip(case['tasks'], row['owners']) if owner == i) or '空'}"
                     for i, boat in enumerate(case["boats"]))

