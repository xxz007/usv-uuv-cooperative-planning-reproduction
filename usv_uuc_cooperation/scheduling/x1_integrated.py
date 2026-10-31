"""X1 initial problem; isolated Pyomo worker reads only saved t0 data."""
import argparse
from decimal import Decimal, ROUND_HALF_UP
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

from usv_uuc_cooperation.metrics.logger import digest


def build_problem(config):
    from usv_uuc_cooperation.env.obstacle import forbidden_nodes
    from usv_uuc_cooperation.planning.grid_planner import plan
    boats, tasks, scale = config['uuc_ids'], sorted(config['tasks_m']), config['x1']['scale']
    adjacency = config['map']['adjacency']
    blocked = forbidden_nodes(config)
    routes = {f'{b}->{t}': plan(config['starts_m'][b], config['tasks_m'][t], config, blocked, adjacency=adjacency)
              for b in boats for t in tasks}
    costs = [[routes[f'{b}->{t}']['distance_m'] if routes[f'{b}->{t}'] else None for t in tasks] for b in boats]
    ints = [[int((Decimal(str(c)) * scale).quantize(Decimal('1'), rounding=ROUND_HALF_UP))
             if c is not None else 0 for c in row] for row in costs]
    return {'boats': boats, 'tasks': tasks, 'capacity': [config['capacity_per_uuc']] * len(boats),
            'adjacency': adjacency, 'cost_m': costs, 'cost_int': ints, 'scale': scale,
            'reachable': [[c is not None for c in row] for row in costs],
            'rounding': 'ROUND_HALF_UP', 'per_coefficient_error_bound_m': .5 / scale,
            'selected_sum_error_bound_m': len(tasks) * .5 / scale,
            'start_task_routes': routes, 'map': config['map'],
            'starts_m': {b: config['starts_m'][b] for b in boats}, 'tasks_m': config['tasks_m'],
            'scope': 't0 known tasks only; minimum scaled start-task sum, no route/makespan/energy optimum; route budget preflight separately'}


def solve_saved(problem_path):
    # Only this worker uses the unchanged P4-L4 Pyomo/HiGHS environment.
    import pyomo.environ as pyo
    from pyomo.contrib.appsi.solvers import Highs
    from usv_uuc_cooperation.scheduling.pyomo_scheduler import build_model
    from usv_uuc_cooperation.main import assert_frozen_core, save
    assert_frozen_core()
    problem_path = Path(problem_path).resolve()
    folder, problem = problem_path.parent, json.loads(problem_path.read_text())
    if (folder / '初始分配.json').exists():
        raise FileExistsError(folder / '初始分配.json')
    model = build_model(problem)
    model.write(str(folder / '初始模型.lp'), io_options={'symbolic_solver_labels': True})
    solver = Highs()
    if not solver.available():
        raise RuntimeError('HiGHS unavailable')
    solver.config.load_solution = False
    solver.config.time_limit = 30
    solver.config.mip_gap = 0
    solver.config.logfile = str(folder / 'HiGHS.log')
    solver.highs_options.update(threads=1, random_seed=0, mip_abs_gap=0)
    stamp = time.perf_counter()
    returned = solver.solve(model)
    term = returned.termination_condition.name
    feasible = returned.best_feasible_objective
    has_solution = feasible is not None and math.isfinite(feasible)
    result = {'interface': 'pyomo.contrib.appsi.solvers.Highs', 'adjacency': problem['adjacency'],
              'problem_sha256': digest(problem_path), 'termination_condition': term,
              'status': ('OPTIMAL' if term == 'optimal' else 'FEASIBLE_NOT_PROVEN_OPTIMAL') if has_solution
                        else ('INFEASIBLE' if term == 'infeasible' else 'NO_FEASIBLE_SOLUTION'),
              'has_feasible_solution': has_solution, 'variables_loaded': False, 'assignment': None,
              'objective_bound_int': returned.best_objective_bound, 'solve_wall_s': time.perf_counter() - stamp,
              'python': sys.executable, 'command': sys.argv,
              'versions': {k: importlib.metadata.version(k) for k in ('pyomo', 'highspy')},
              'optimality_scope': 'scaled start-task assignment only'}
    if has_solution:
        returned.solution_loader.load_vars()
        matrix = [[pyo.value(model.x[b, t]) for t in problem['tasks']] for b in problem['boats']]
        integer = sum(x * c for row, costs in zip(matrix, problem['cost_int']) for x, c in zip(row, costs))
        actual = math.fsum(c for row, costs in zip(matrix, problem['cost_m']) for x, c in zip(row, costs) if x > .5)
        assert math.isclose(integer, pyo.value(model.total), abs_tol=1e-7)
        result.update(variables_loaded=True, matrix=matrix, objective_int=integer,
                      scaled_objective_m=integer / problem['scale'], float_start_task_sum_m=actual,
                      selected_sum_error_m=integer / problem['scale'] - actual,
                      assignment={b: [t for t, x in zip(problem['tasks'], row) if x > .5]
                                  for b, row in zip(problem['boats'], matrix)})
    save(folder / '初始分配.json', result)
    print(f"邻接{problem['adjacency']} HiGHS {result['status']}", flush=True)
    if not has_solution:
        raise RuntimeError('No feasible initial assignment; simulation forbidden')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--problem', type=Path, required=True)
    solve_saved(parser.parse_args().problem)
