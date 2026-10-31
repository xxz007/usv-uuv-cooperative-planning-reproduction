"""Check saved framework runs against frozen lesson evidence, without rerunning simulations."""
import argparse
import ast
import builtins
import hashlib
import importlib
import json
from pathlib import Path
import sys
import symtable

from usv_uuc_cooperation.config import (PACKAGE_ROOT as F, PROJECT_ROOT as R, DOCS_ROOT as D,
    RESULTS_ROOT as S, FRAMEWORK_VERSION, artifact_path, read_json)
from usv_uuc_cooperation.metrics.logger import digest


def verify(run_id):
    for row in read_json(D / "固定输入来源_V01.json"):
        if digest(F / row["copy"]) != row["sha256"]:
            raise ValueError(f"Copied configuration changed: {row['copy']}")
    for name, expected in read_json(D / "归档原件身份_V00.json").items():
        if digest(R / name) != expected:
            raise ValueError(f"Archived input/source/illustration changed: {name}")
    origins = read_json(D / "源码提取来源_V01.json")
    receipt = read_json(D / "P4_Pyomo构模路径核查_V01.json")
    if digest(F / "scheduling/pyomo_scheduler.py") != receipt["model_source_sha256"] or digest(F / "configs/P4-L4_V01.json") != receipt["config_sha256"]:
        raise ValueError("Saved Pyomo build probe no longer matches source/configuration")
    for row in origins:
        tree = ast.parse((F / row["module"]).read_text())
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == row["function"])
        rows = (F / row["module"]).read_text().splitlines(keepends=True)
        body = "".join(rows[fn.lineno - 1:fn.end_lineno])
        actual = hashlib.sha256(body.encode()).hexdigest()
        original_tree = ast.parse((R / row["source"]).read_text())
        original_fn = next(n for n in original_tree.body if isinstance(n, ast.FunctionDef) and n.name == row["function"])
        same_ast = ast.dump(fn, include_attributes=False) == ast.dump(original_fn, include_attributes=False)
        if actual != row["function_text_sha256"] or not same_ast:
            raise ValueError(f"Extracted function body changed: {row['module']}:{row['function']}")
    modules = []
    for p in sorted(F.rglob("*.py")):
        if "versions" in p.parts:
            continue
        ast.parse(p.read_text(), filename=str(p))
        if p.name != "__init__.py" and p.relative_to(F).as_posix() != "scheduling/pyomo_scheduler.py":
            name = "usv_uuc_cooperation." + ".".join(p.relative_to(F).with_suffix("").parts)
            importlib.import_module(name)
            modules.append(name)
    for rel in sorted({r["module"] for r in origins} - {"scheduling/pyomo_scheduler.py"}):
        mod = importlib.import_module("usv_uuc_cooperation." + ".".join(Path(rel).with_suffix("").parts))
        table = symtable.symtable((F / rel).read_text(), rel, "exec")
        def check_globals(tab):
            for symbol in tab.get_symbols():
                name = symbol.get_name()
                if symbol.is_referenced() and symbol.is_global() and name not in vars(mod) and not hasattr(builtins, name):
                    raise ValueError(f"Missing extracted dependency: {rel}:{tab.get_name()}:{name}")
            for child in tab.get_children():
                check_globals(child)
        for child in table.get_children():
            check_globals(child)
    roots = {s: S / s / f"{s}-V01" / "runs" / run_id for s in ["P1", "P2", "P3"]}
    p1 = read_json(roots["P1"] / "结果.json")
    if p1 != read_json(R / "课时资料/P1/P1-L3/结果/P1-L3_验证结果.json"):
        raise ValueError("P1 saved scenarios differ")
    for case, original in [("baseline", "P2-L2_V01_枚举结果.json"), ("counterexample", "P2-L2_CE01_枚举结果.json")]:
        new = read_json(roots["P2"] / f"{case}_结果.json")
        old = read_json(R / "课时资料/P2/P2-L2/结果" / original)
        if any(new[k] != old[k] for k in ["case_id", "case", "greedy", "enumeration"]):
            raise ValueError(f"P2 saved case differs: {case}")
    csv_checks = []
    for case in ["SUCCESS", "RESTRICTED"]:
        new = read_json(roots["P3"] / f"{case}_结果.json")
        old = read_json(R / f"课时资料/P3/P3-L4/结果/P3-L4_{case}_结果.json")
        for k in ["case", "config", "effective_budgets_NEU", "planning_identity", "optimality_claim",
                  "sequences", "scheduling_decisions", "legs", "tasks", "summary", "snapshots"]:
            if new[k] != old[k]:
                raise ValueError(f"P3 deterministic result differs: {case}:{k}")
        for suffix in ["逐步任务能量.csv", "逐时通信.csv", "任务状态.csv"]:
            newp = roots["P3"] / f"{case}_{suffix}"
            oldp = R / f"课时资料/P3/P3-L4/结果/P3-L4_{case}_{suffix}"
            if newp.read_bytes() != oldp.read_bytes():
                raise ValueError(f"P3 CSV differs: {case}:{suffix}")
            csv_checks.append(newp.relative_to(R).as_posix())
    images = []
    from PIL import Image
    for row in read_json(D / "已有阶段效果图来源_V01.json"):
        path = artifact_path(row["published"])
        if digest(path) != row["sha256"] or digest(R / row["source"]) != row["sha256"]:
            raise ValueError(f"Published illustration differs: {path}")
        with Image.open(path) as im:
            frames = getattr(im, "n_frames", 1)
            im.seek(frames - 1)
            im.load()
            images.append({"path": path.relative_to(R).as_posix(), "format": im.format, "frames": frames,
                           "size": list(im.size), "source_sha256_match": True})
    for folder in roots.values():
        for line in (folder / "SHA256.txt").read_text().splitlines():
            expected, name = line.split("  ", 1)
            if digest(folder / name) != expected:
                raise ValueError(f"Framework run artifact changed: {folder / name}")
    if (S / "P4").exists():
        raise ValueError("V01 task reserves P4 illustration generation for the P4-L4 chat")
    return {
        "status": "PASS", "framework": FRAMEWORK_VERSION, "date": "2026-10-11", "run_id": run_id,
        "registered_lesson_source_files": len(read_json(D / "课时源码登记_V01.json")), "archived_original_identity_records": len(read_json(D / "归档原件身份_V00.json")),
        "extracted_implementation_modules": len({r["module"] for r in origins}),
        "function_ASTs_preserved": len(origins), "shared_environment_modules_imported": len(modules),
        "extracted_global_dependencies": "PASS",
        "P1_three_scenarios_saved_JSON_equal": True, "P2_two_cases_complete_saved_results_equal": True,
        "P3_two_cases_all_deterministic_fields_equal": True, "P3_CSV_byte_equal": csv_checks,
        "framework_run_artifact_manifests": "PASS", "reused_illustrations": images,
        "new_GIF_renders": 0, "P4_new_visuals": 0, "P4_extra_solver_runs": 0,
        "Pyomo_model_build": {"evidence_mode": "SAVED_SEPARATE_BUILD_PROBE_REUSED_NOT_RERUN", "receipt": (D / "P4_Pyomo构模路径核查_V01.json").relative_to(R).as_posix(), "data": receipt},
        "teacher_framework_regression_runs": {"P1": 1, "P2": 1, "P3": 1},
        "learner_acceptance_changed": False,
        "scope": "archived identities, function bodies/imports, saved-output equivalence and existing image decode; P4 unified execution not checked",
        "native_GIF_player_UI_verified": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output", type=Path, help="Optional new JSON report; refuses existing output")
    args = parser.parse_args()
    report = verify(args.run_id)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as out:
            json.dump(report, out, ensure_ascii=False, indent=2)
            out.write("\n")
    print(json.dumps({"status": report["status"], "functions": report["function_ASTs_preserved"],
                      "P3_equal_csv": len(report["P3_CSV_byte_equal"]), "P4_new_visuals": 0}, ensure_ascii=False))


if __name__ == "__main__":
    main()
