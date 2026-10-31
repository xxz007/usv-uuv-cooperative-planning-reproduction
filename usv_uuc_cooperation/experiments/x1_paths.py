"""X1-L1 paired geometry-only planning, with an immutable P4 map reference."""
import sys

from usv_uuc_cooperation.config import (PACKAGE_ROOT, PROJECT_ROOT, FRAMEWORK_VERSION,
                                       new_run_directory, read_json)
from usv_uuc_cooperation.env.obstacle import forbidden_nodes, unsafe_reason
from usv_uuc_cooperation.metrics.logger import digest
from usv_uuc_cooperation.planning.grid_planner import plan


def run(run_id):
    from usv_uuc_cooperation.main import save, assert_frozen_core
    from usv_uuc_cooperation.visualization.grid_paths import render
    assert_frozen_core()
    config_path = PACKAGE_ROOT / "configs/X1-L1_V01.json"
    config = read_json(config_path)
    source = PROJECT_ROOT / config["source"]["path"]
    if digest(source) != config["source"]["sha256"]:
        raise ValueError("P4 source config differs from the frozen X1 reference")
    original = read_json(source)
    if (config["map"] != original["map"] or config["start_m"] != original["starts_m"]["U1"]
            or config["goal_m"] != original["tasks_m"]["T03"]):
        raise ValueError("X1 paired map/endpoints differ from the recorded P4 source")
    blocked = forbidden_nodes(config)
    routes = {str(mode): plan(config["start_m"], config["goal_m"], config, blocked, adjacency=mode)
              for mode in (4, 8)}
    if any(route is None for route in routes.values()):
        raise ValueError("Main case must have two reachable paths")
    for route in routes.values():
        if any(unsafe_reason(a, b, config, blocked) for a, b in zip(route["path"], route["path"][1:])):
            raise ValueError("Unsafe planned segment")
    if routes["8"]["diagonal_edges"] == 0 or routes["8"]["distance_m"] >= routes["4"]["distance_m"]:
        raise ValueError("Main case must contain legal diagonal benefit")
    result = {"scope": "geometric planning only", "framework": FRAMEWORK_VERSION,
              "config": config, "blocked_nodes": sorted(map(list, blocked)), "routes": routes,
              "geometric_reduction_m": routes["4"]["distance_m"] - routes["8"]["distance_m"]}
    folder = new_run_directory("X1", run_id)
    save(folder / "路径.json", result)
    render(result, folder / "路径叠图.png")
    rows = "\n".join(f"| {mode} | {r['distance_m']:.9f} | {r['step_count']} | {r['straight_edges']} | {r['diagonal_edges']} |"
                     for mode, r in routes.items())
    text = (f"# X1-L1 四/八邻接路径对照\n\n版本：{FRAMEWORK_VERSION} / X1-L1_V01；仅几何规划。\n\n"
            f"同P4地图、同禁行闭方格与1m缓冲；U1 {config['start_m']} → T03 {config['goal_m']}。\n\n"
            "| 邻接 | distance_m 航程/m | step_count 节点步数 | 直边 | 斜边 |\n|---|---:|---:|---:|---:|\n"
            f"{rows}\n\n整段安全：两组均通过沿用检查器；八邻接另检查两侧正交格。\n"
            f"几何航程减少 {result['geometric_reduction_m']:.9f} m。系统执行时间尚未评价。\n\n"
            f"![路径叠图]({folder / '路径叠图.png'})\n\n"
            f"[原始路径]({folder / '路径.json'}) · [运行身份]({folder / '运行元数据.json'})\n\n"
            "关注：distance_m、step_count、diagonal_edges。教师运行不构成本人课堂验收。\n")
    (folder / "摘要.md").write_text(text, encoding="utf-8")
    save(folder / "运行元数据.json", {"framework": FRAMEWORK_VERSION, "lesson": "X1-L1",
        "scope": "standalone geometry; no dynamic execution", "command": sys.argv,
        "python": sys.executable, "config_sha256": digest(config_path), "source_sha256": digest(source),
        "output": str(folder), "status": "COMPLETE"})
    (folder / "SHA256.txt").write_text("\n".join(f"{digest(p)}  {p.name}" for p in sorted(folder.iterdir())
                                                if p.is_file()) + "\n")
    return folder
