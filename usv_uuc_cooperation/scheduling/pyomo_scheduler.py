"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P4/P4-L4/源码/p4_l4_model.py
"""
import json
from pathlib import Path
import pyomo.environ as pyo
from usv_uuc_cooperation.config import PACKAGE_ROOT

CASE_PATH=PACKAGE_ROOT/"configs/P4-L4_V01.json"

def load_case(configuration, path=CASE_PATH):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    case = data["case"]
    case["capacity"] = data["configurations"][configuration]["capacity"]
    case["cost_int"] = data["scaling"]["cost_int"]
    case["scale"] = data["scaling"]["scale"]
    return case


def build_model(case):
    m = pyo.ConcreteModel()
    m.I = pyo.Set(initialize=case["boats"], ordered=True)
    m.J = pyo.Set(initialize=case["tasks"], ordered=True)
    pairs = [(i, j) for i in m.I for j in m.J]
    positions = {(i, j): (a, b) for a, i in enumerate(m.I) for b, j in enumerate(m.J)}
    m.q = pyo.Param(m.I, initialize=dict(zip(m.I, case["capacity"])))
    m.c = pyo.Param(m.I, m.J, initialize={
        (i, j): case["cost_int"][positions[i, j][0]][positions[i, j][1]]
        for i, j in pairs
    })
    m.r = pyo.Param(m.I, m.J, initialize={
        (i, j): int(case["reachable"][positions[i, j][0]][positions[i, j][1]])
        for i, j in pairs
    })
    m.x = pyo.Var(m.I, m.J, domain=pyo.Binary)
    m.cover = pyo.Constraint(m.J, rule=lambda m, j: sum(m.x[i, j] for i in m.I) == 1)
    m.capacity = pyo.Constraint(m.I, rule=lambda m, i: sum(m.x[i, j] for j in m.J) <= m.q[i])
    m.reach = pyo.Constraint(m.I, m.J, rule=lambda m, i, j: m.x[i, j] <= m.r[i, j])
    m.total = pyo.Objective(expr=sum(m.c[i, j] * m.x[i, j] for i, j in pairs), sense=pyo.minimize)
    return m

