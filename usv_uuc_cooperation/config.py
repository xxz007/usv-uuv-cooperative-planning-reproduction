"""Paths and explicit stage profiles; no working-directory-dependent inputs."""
from pathlib import Path
import json
import re

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
OUTPUT_ROOT = PROJECT_ROOT / "输出文件"
RESULTS_ROOT = OUTPUT_ROOT / "阶段结果"
DOCS_ROOT = OUTPUT_ROOT / "说明文档"
VERSIONS_ROOT = OUTPUT_ROOT / "版本快照"
FRAMEWORK_VERSION = "FRAMEWORK-V20"
PROFILES = {
    "P1": "P1-V01", "P2": "P2-V01", "P3": "P3-V01", "P4": "P4-V01", "X1": "X1-V01", "X2": "X2-V01", "P5": "P5-V01"
}


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def stage_catalog():
    return read_json(VERSIONS_ROOT / "阶段版本索引.json")["stages"]


def artifact_path(reference):
    """Resolve current project paths and recorded pre-migration artifact paths."""
    path = Path(reference)
    if path.is_absolute():
        roots = ((PROJECT_ROOT, PROJECT_ROOT),
                 (Path("/Users/omo/Downloads/Project/Pyomo"), PROJECT_ROOT),
                 (Path("/Users/omo/Downloads/Project/VScode-Project/moos-ivp"), PROJECT_ROOT / "moos-ivp"))
        for previous, current in roots:
            if path.is_relative_to(previous):
                path = (current / path.relative_to(previous)).relative_to(PROJECT_ROOT)
                break
        else:
            return path
    for old_name, directory in (("outputs", RESULTS_ROOT), ("docs", DOCS_ROOT),
                                ("versions", VERSIONS_ROOT)):
        for prefix in (Path("usv_uuc_cooperation") / old_name, Path(old_name)):
            if path.is_relative_to(prefix):
                return directory / path.relative_to(prefix)
    return PROJECT_ROOT / path


def new_run_directory(stage, run_id):
    if stage not in PROFILES:
        raise ValueError(f"No runnable profile for {stage}")
    if not re.fullmatch(r"[\w\-]+", run_id):
        raise ValueError("run-id must be a single name containing letters, numbers, underscores or hyphens")
    path = RESULTS_ROOT / stage / PROFILES[stage] / "runs" / run_id
    path.mkdir(parents=True, exist_ok=False)
    return path
