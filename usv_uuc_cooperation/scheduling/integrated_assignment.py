"""Real initial 5x20 Pyomo assignment; future events never enter the model."""
import argparse
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

from usv_uuc_cooperation.config import PACKAGE_ROOT
from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.metrics.logger import digest
from usv_uuc_cooperation.planning.astar import astar


def initial_problem(config):
    blocked = forbidden_nodes(config)
    ids, tasks = config["uuc_ids"], sorted(config["tasks_m"])
    costs, reach, routes = [], [], {}
    for boat in ids:
        row, flags = [], []
        for task in tasks:
            path = astar(config["starts_m"][boat], config["tasks_m"][task], config["map"], blocked)
            flags.append(path is not None)
            row.append(path["distance_m"] if path else 0)
            routes[f"{boat}->{task}"] = path
        costs.append(row)
        reach.append(flags)
    return {"boats": ids, "tasks": tasks, "capacity": [config["capacity_per_uuc"]] * len(ids),
            "cost_m": costs, "cost_int": [[int(x * 1000) for x in r] for r in costs],
            "reachable": reach, "scale": 1000, "start_task_routes": routes,
            "map": config["map"], "starts_m": {b: config["starts_m"][b] for b in ids},
            "tasks_m": config["tasks_m"],
            "scope": "t0 known tasks only; minimum sum of start-to-task A* distances, not route/makespan/energy optimum"}


def solve_initial(config_path, output):
    import pyomo.environ as pyo
    from pyomo.contrib.appsi.solvers import Highs
    from usv_uuc_cooperation.scheduling.pyomo_scheduler import build_model
    config_path, output = Path(config_path), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    problem = initial_problem(json.loads(config_path.read_text()))
    def save(name, obj):
        (output / name).write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n")
    save("初始问题.json", problem)
    model = build_model(problem)
    model.write(str(output / "初始模型.lp"), io_options={"symbolic_solver_labels": True})
    solver = Highs()
    if not solver.available():
        raise RuntimeError("HiGHS unavailable; do not generate trajectories")
    solver.config.load_solution = False
    solver.config.time_limit = 30
    solver.config.mip_gap = 0
    solver.config.logfile = str(output / "HiGHS.log")
    solver.highs_options.update(threads=1, random_seed=0, mip_abs_gap=0)
    stamp = time.perf_counter()
    returned = solver.solve(model)
    term = returned.termination_condition.name
    feasible = returned.best_feasible_objective
    has_solution = feasible is not None and math.isfinite(feasible)
    result = {"implementation": "P4-INTEGRATED-ASSIGNMENT-V01", "problem_sha256": digest(output / "初始问题.json"),
              "interface": "pyomo.contrib.appsi.solvers.Highs", "termination_condition": term,
              "status": ("OPTIMAL" if term == "optimal" else "FEASIBLE_NOT_PROVEN_OPTIMAL") if has_solution
                        else ("INFEASIBLE" if term == "infeasible" else "NO_FEASIBLE_SOLUTION"),
              "has_feasible_solution": has_solution, "variables_loaded": False,
              "assignment": None, "objective_m": None, "matrix": None,
              "objective_bound_int": returned.best_objective_bound,
              "versions": {"python": sys.version.split()[0], "pyomo": importlib.metadata.version("pyomo"),
                           "highspy": importlib.metadata.version("highspy"), "HiGHS": list(solver.version())},
              "solve_wall_s": time.perf_counter() - stamp, "available": bool(solver.available()),
              "model_counts": {"binary": len(model.x), "cover": len(model.cover), "capacity": len(model.capacity), "reach": len(model.reach)},
              "source_sha256": {str(p.relative_to(PACKAGE_ROOT.parent)): digest(p) for p in
                                [Path(__file__), PACKAGE_ROOT / "scheduling/pyomo_scheduler.py", PACKAGE_ROOT / "planning/astar.py", PACKAGE_ROOT / "env/obstacle.py"]},
              "official_interface": "https://pyomo.readthedocs.io/en/6.10.1/_modules/pyomo/contrib/appsi/solvers/highs.html",
              "decision_input": "初始问题.json; no future event table or current schedule", "command": sys.argv,
              "python": sys.executable, "output": str(output.resolve())}
    if has_solution:
        returned.solution_loader.load_vars()
        matrix = [[pyo.value(model.x[b, t]) for t in problem["tasks"]] for b in problem["boats"]]
        assert all(min(abs(x), abs(x - 1)) < 1e-7 for row in matrix for x in row)
        assert all(abs(sum(row[j] for row in matrix) - 1) < 1e-7 for j in range(len(problem["tasks"])))
        assert all(sum(row) <= q + 1e-7 for row, q in zip(matrix, problem["capacity"]))
        assert all(x <= int(r) + 1e-7 for row, flags in zip(matrix, problem["reachable"]) for x, r in zip(row, flags))
        independent = sum(x * c for row, costs in zip(matrix, problem["cost_int"]) for x, c in zip(row, costs))
        assert math.isclose(independent, pyo.value(model.total), abs_tol=1e-7)
        if term == "optimal":
            assert math.isclose(independent, returned.best_objective_bound, abs_tol=1e-7)
        result.update(variables_loaded=True, matrix=matrix, objective_int=independent, objective_m=independent / 1000,
                      assignment={b: [t for t, x in zip(problem["tasks"], row) if x > .5]
                                  for b, row in zip(problem["boats"], matrix)}, constraint_check="PASS")
    save("初始分配.json", result)
    save("约束核查.json", {"status": "PASS" if has_solution else "NO_EXECUTABLE_ASSIGNMENT",
                          "binary_variables": result["model_counts"]["binary"], "all_coverage_capacity_reach_and_objective_checked": has_solution,
                          "scope": problem["scope"], "real_solver_log": "HiGHS.log"})
    (output / "摘要.md").write_text(f"# P4 初始实际分配\n\n{result['status']}；起点到任务A*代理目标 {result['objective_m']} m。\n\n这是分配代理目标，不是完整航程最优。两组复用本解，不读取未来事件。\n")
    (output / "SHA256.txt").write_text(''.join(f"{digest(p)}  {p.name}\n" for p in sorted(output.iterdir()) if p.is_file()))
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(solve_initial(args.config, args.output) / "摘要.md")
