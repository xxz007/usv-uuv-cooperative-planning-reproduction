"""One entry for stage discovery and versioned framework reproductions."""
import argparse
import json
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from usv_uuc_cooperation.config import (PACKAGE_ROOT, PROJECT_ROOT, DOCS_ROOT, VERSIONS_ROOT,
    PROFILES, FRAMEWORK_VERSION, artifact_path, stage_catalog, new_run_directory)
from usv_uuc_cooperation.metrics.logger import digest, write_csv


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def assert_frozen_core():
    manifest = VERSIONS_ROOT / FRAMEWORK_VERSION / "源码配置_SHA256.txt"
    for line in manifest.read_text().splitlines():
        expected, name = line.split("  ", 1)
        if digest(PROJECT_ROOT / name) != expected:
            raise ValueError(f"{FRAMEWORK_VERSION} source/config changed: {name}; prepare a new version before running")


def run_stage(stage, case, run_id, render=False, initial_result=None):
    if stage == "P4":
        if case != "ALL" or initial_result is None:
            raise ValueError("P4 requires --case ALL and --initial-result ABSOLUTE_INITIAL_JSON; runs exactly the two responses")
        from usv_uuc_cooperation.experiments.p4_integrated import run_pair
        return run_pair(run_id, initial_result, render)
    if stage not in ("P1", "P2", "P3"):
        raise ValueError(f"P4 showcase is reserved for the P4-L4 chat; use {DOCS_ROOT / 'P4阶段效果展示_续会话提示词.md'}")
    if render and stage != "P3":
        raise ValueError("Only the verified P3 snapshot renderer is connected; P1/P2 use existing static figures")
    allowed = {"P1": {"ALL"}, "P2": {"ALL", "baseline", "counterexample"},
               "P3": {"ALL", "SUCCESS", "RESTRICTED"}}
    if case not in allowed[stage]:
        raise ValueError(f"Unknown {stage} case {case}; choices: {sorted(allowed[stage])}")
    assert_frozen_core()
    folder = new_run_directory(stage, run_id)
    try:
        if stage == "P1":
            from usv_uuc_cooperation.experiments.p1_pathfinding import run
            results = run()
            save(folder / "结果.json", results)
            text = "\n".join(f"- {k}：{v['status']}，A* 长度 {v['astar_length']} m" for k, v in results.items())
        elif stage == "P2":
            from usv_uuc_cooperation.experiments.p2_assignment import run
            cases = ["baseline", "counterexample"] if case == "ALL" else [case]
            results = {c: run(c) for c in cases}
            for c, result in results.items():
                save(folder / f"{c}_结果.json", result)
            text = "\n".join(f"- {c}：贪心 {r['greedy']['cost_m']} m，枚举最优 {r['enumeration']['optimal_cost_m']} m"
                             for c, r in results.items())
            text += "\n\n单位是固定起点分配成本，不是完整航程或完成时间。"
        else:
            from usv_uuc_cooperation.experiments.p3_baseline import run
            cases = ["SUCCESS", "RESTRICTED"] if case == "ALL" else [case]
            rows = []
            for c in cases:
                result, steps, links = run(c)
                save(folder / f"{c}_结果.json", result)
                write_csv(folder / f"{c}_逐步任务能量.csv", steps)
                write_csv(folder / f"{c}_逐时通信.csv", links)
                write_csv(folder / f"{c}_任务状态.csv", [{"task_id": t, **s} for t, s in sorted(result["tasks"].items())])
                if render:
                    from usv_uuc_cooperation.env.obstacle import forbidden_nodes
                    from usv_uuc_cooperation.visualization.animation import render as render_p3
                    render_p3(result, forbidden_nodes(result["config"]), folder / c)
                s = result["summary"]
                rows.append({"case": c, "completed": f"{s['completed_task_count']}/{s['task_count']}",
                    "team_completion_time_s": s["team_completion_time_s"],
                    "total_energy_NEU": s["total_all_boat_energy_NEU"],
                    "communication_retention_rate": s["communication"]["retention_rate"]})
            save(folder / "指标.json", rows)
            text = "| 案例 | 完成任务 | 成功时间 s | 实际能耗 NEU |\n|---|---|---|---|\n" + "\n".join(
                f"| {r['case']} | {r['completed']} | {r['team_completion_time_s'] if r['team_completion_time_s'] is not None else 'null'} | {r['total_energy_NEU']} |" for r in rows)
            text += "\n\n观察窗 [0,120)。未完成时间保持 null；较低失败能耗不表示完成同任务更节能。"
        (folder / "摘要.md").write_text(f"# {stage} {PROFILES[stage]} 框架运行\n\n{text}\n\n输出目录：`{folder}`\n"
            "\n课堂学习验收状态沿用原课时证据；本次运行是框架复现。\n", encoding="utf-8")
        save(folder / "运行元数据.json", {"framework": FRAMEWORK_VERSION, "stage": stage,
            "stage_version": PROFILES[stage], "command": sys.argv, "python": sys.executable,
            "output": str(folder), "render_requested": render, "status": "COMPLETE"})
        files = sorted(p for p in folder.iterdir() if p.is_file())
        (folder / "SHA256.txt").write_text("\n".join(f"{digest(p)}  {p.name}" for p in files) + "\n")
    except Exception as exc:
        save(folder / "失败.json", {"status": "FAILED", "error": str(exc), "output": str(folder)})
        raise
    return folder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("list", help="List stage and showcase status")
    show = sub.add_parser("show", help="Locate an existing showcase without rerunning simulation")
    show.add_argument("--stage", required=True)
    planning = sub.add_parser("plan", help="X1-L1 standalone paired four/eight-neighbor geometric paths")
    planning.add_argument("--case", choices=["X1-L1"], required=True)
    planning.add_argument("--run-id", required=True)
    execution = sub.add_parser("execute", help="X1-L2 saved L1 paths: paired no-flow then P4-flow execution")
    execution.add_argument("--case", choices=["X1-L2"], required=True)
    execution.add_argument("--run-id", required=True)
    assignment = sub.add_parser("assign", help="X1-L3: fixed queues first, then scaled CP-SAT reassignment")
    assignment.add_argument("--case", choices=["X1-L3"], required=True)
    assignment.add_argument("--phase", choices=["fixed", "reassign"], required=True)
    assignment.add_argument("--run-id", required=True)
    integration = sub.add_parser("integrate", help="X1-L4: each adjacency solves initial Pyomo assignment, then same P4 dynamic strategy")
    integration.add_argument("--case", choices=["X1-L4"], required=True)
    integration.add_argument("--run-id", required=True)
    comparison = sub.add_parser('compare', help='X2 complete extension: shared observed input, existing baselines, original D* or scan ordering plus A*')
    comparison.add_argument('--case', choices=['X2-L1', 'X2-L2', 'X2-Dstar', 'X2-Scan', 'X2-PSO'], required=True)
    comparison.add_argument('--run-id', required=True)
    run = sub.add_parser("run", help="New isolated stage run; P4 uses one initial solution for both strategies")
    run.add_argument("--stage", required=True, choices=["P1", "P2", "P3", "P4"])
    run.add_argument("--case", default="ALL")
    run.add_argument("--run-id", required=True)
    run.add_argument("--render", action="store_true", help="P3/P4: render from this run's saved actual snapshots")
    run.add_argument("--initial-result", type=Path, help="P4 actual initial assignment JSON, from the L4 Pyomo environment")
    solve = sub.add_parser("solve", help="P4 initial5x20 Pyomo solve; use the L4 isolated interpreter")
    solve.add_argument("--stage", required=True, choices=["P4"])
    solve.add_argument("--run-id", required=True)
    display = sub.add_parser("render", help="P4 render saved logs without simulation or solver reruns")
    display.add_argument("--stage", required=True, choices=["P4", "X1", "X2"])
    display.add_argument("--run", type=Path, required=True)
    display.add_argument('--revision', help='X2 only: save a new figure revision such as V02 without overwriting earlier figures')
    args = parser.parse_args()
    try:
        if args.action == "list":
            for stage, row in stage_catalog().items():
                print(f"{stage} | {row['title']} | 课程 {row['course_status']} | 效果展示 {row['visual_status']}")
            print(f"入口：{PACKAGE_ROOT / 'README.md'}")
        elif args.action == "show":
            row = stage_catalog()[args.stage]
            print(f"{args.stage}: {row['visual_status']}")
            for path in row["assets"]:
                print(artifact_path(path))
            print(DOCS_ROOT / "阶段成果总览.md")
            if args.stage == "P4":
                print(DOCS_ROOT / "P4统一案例实现与展示_V01.md")
        elif args.action == "plan":
            from usv_uuc_cooperation.experiments.x1_paths import run as run_paths
            folder = run_paths(args.run_id)
            print(f"摘要：{folder / '摘要.md'}")
            print(f"输出目录：{folder}")
        elif args.action == "execute":
            from usv_uuc_cooperation.experiments.x1_execution import run as run_execution
            folder = run_execution(args.run_id)
            print(f"摘要：{folder / '摘要.md'}")
            print(f"输出目录：{folder}")
        elif args.action == "assign":
            from usv_uuc_cooperation.experiments.x1_assignment import run as run_assignment
            folder = run_assignment(args.run_id, args.phase)
            print(f"摘要：{folder / '摘要.md'}")
            print(f"输出目录：{folder}")
        elif args.action == "solve":
            from usv_uuc_cooperation.scheduling.integrated_assignment import solve_initial
            from usv_uuc_cooperation.config import RESULTS_ROOT
            # Validate the run-id with the shared helper, then let the solver create it.
            import re
            if not re.fullmatch(r"[\w\-]+", args.run_id):
                raise ValueError("run-id must be a single name")
            folder = solve_initial(PACKAGE_ROOT / "configs/P4-integrated-V01.json",
                                   RESULTS_ROOT / "P4" / PROFILES["P4"] / "runs" / args.run_id)
            print(f"摘要：{folder / '摘要.md'}")
        elif args.action == 'compare':
            if args.case == 'X2-PSO':
                from usv_uuc_cooperation.experiments.x2_pso import run as run_pso
                folder = run_pso(args.run_id)
            elif args.case == 'X2-Scan':
                from usv_uuc_cooperation.experiments.x2_grid_scan import run as run_scan
                folder = run_scan(args.run_id)
            elif args.case == 'X2-Dstar':
                from usv_uuc_cooperation.experiments.x2_original_dstar import run as run_original_dstar
                folder = run_original_dstar(args.run_id)
            elif args.case == 'X2-L2':
                from usv_uuc_cooperation.experiments.x2_dstar_lite import run as run_dstar
                folder = run_dstar(args.run_id)
            else:
                from usv_uuc_cooperation.experiments.x2_comparison import run_pair
                folder = run_pair(args.run_id)
            print(f"摘要：{folder / '摘要.md'}")
            print(f"输出目录：{folder}")
        elif args.action == "integrate":
            from usv_uuc_cooperation.experiments.x1_integrated import run_pair
            folder = run_pair(args.run_id)
            print(f"摘要：{folder / '摘要.md'}")
            print(f"输出目录：{folder}")
        elif args.action == "render":
            if args.stage == 'X2':
                metadata = json.loads((args.run / '运行元数据.json').read_text())
                if metadata['lesson'] in ('X2-L2', 'X2-Dstar', 'X2-Scan', 'X2-PSO'):
                    from usv_uuc_cooperation.visualization.x2_dstar_effects import render
                    print(render(args.run, args.revision) / '摘要.md')
                else:
                    from usv_uuc_cooperation.visualization.x2_effects import render_pair
                    print(render_pair(args.run, args.revision) / '摘要.md')
                return
            elif args.stage == "X1":
                from usv_uuc_cooperation.visualization.x1_integrated import render_pair
            else:
                from usv_uuc_cooperation.visualization.integrated_animation import render_pair
            if args.revision is not None:
                raise ValueError('Render revisions are currently connected only for X2')
            print(render_pair(args.run) / "摘要.md")
        else:
            folder = run_stage(args.stage, args.case, args.run_id, args.render, args.initial_result)
            print(f"摘要：{folder / '摘要.md'}")
            print(f"输出目录：{folder}")
    except (ValueError, KeyError, FileExistsError, FileNotFoundError) as exc:
        parser.exit(2, f"无法执行：{exc}\n")


if __name__ == "__main__":
    main()
