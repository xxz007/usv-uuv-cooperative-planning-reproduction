"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P4/P4-L4/源码/p4_l4_audit.py
"""
from itertools import product
import importlib.metadata as metadata
from ortools.sat.python import cp_model



def enumeration(case):
    feasible = []
    candidates = list(product(range(len(case["boats"])), repeat=len(case["tasks"])))
    for owners in candidates:
        if any(owners.count(a) > case["capacity"][a] for a in range(len(case["boats"]))):
            continue
        if not all(case["reachable"][a][b] for b, a in enumerate(owners)):
            continue
        feasible.append({"owners": list(owners), "objective_int": sum(case["cost_int"][a][b] for b, a in enumerate(owners))})
    best = min((v["objective_int"] for v in feasible), default=None)
    return {"candidate_count": len(candidates), "feasible_count": len(feasible), "feasible_assignments": feasible, "optimal_objective_int": best, "optimal_assignments": [v for v in feasible if v["objective_int"] == best]}


def ortools_reference(case):
    model = cp_model.CpModel()
    rows, columns = range(len(case["boats"])), range(len(case["tasks"]))
    x = [[model.new_bool_var(f"x_{a}_{b}") for b in columns] for a in rows]
    for b in columns:
        model.add(sum(x[a][b] for a in rows) == 1)
    for a in rows:
        model.add(sum(x[a]) <= case["capacity"][a])
        for b in columns:
            model.add(x[a][b] <= int(case["reachable"][a][b]))
    model.minimize(sum(case["cost_int"][a][b] * x[a][b] for a in rows for b in columns))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 10.0
    solver.parameters.num_search_workers = 1
    solver.parameters.random_seed = 0
    code = solver.solve(model)
    feasible = code in [cp_model.OPTIMAL, cp_model.FEASIBLE]
    return {"version": metadata.version("ortools"), "status": solver.status_name(code), "objective_int": solver.objective_value if feasible else None, "matrix": [[solver.value(v) for v in row] for row in x] if feasible else None, "actual_solve": True}

