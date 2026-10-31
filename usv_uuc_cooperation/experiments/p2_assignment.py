"""P2 assignment cost, separate from multi-task route length and execution time."""
from usv_uuc_cooperation.config import PACKAGE_ROOT, read_json
from usv_uuc_cooperation.scheduling.greedy_scheduler import greedy, enumerate_all


def run(case_id):
    case = read_json(PACKAGE_ROOT / "configs/P2-V01.json")[case_id]
    return {"case_id": case_id, "case": case, "greedy": greedy(case), "enumeration": enumerate_all(case)}
