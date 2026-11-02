"""Paths and explicit stage profiles; no working-directory-dependent inputs."""
from pathlib import Path
import json
import re

PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
OUTPUT_ROOT = PROJECT_ROOT
RESULTS_ROOT = PROJECT_ROOT / "results"
DOCS_ROOT = PROJECT_ROOT / "docs"
VERSIONS_ROOT = PROJECT_ROOT / "data/provenance"
FRAMEWORK_VERSION = "FRAMEWORK-V22"
PROFILES = {
    "P1": "P1-V01", "P2": "P2-V01", "P3": "P3-V01", "P4": "P4-V01", "X1": "X1-V01", "X2": "X2-V01", "P5": "P5-V01"
}

STAGE_DIRS = {'P1': 'p1_pathfinding', 'P2': 'p2_assignment', 'P3': 'p3_baseline', 'P4': 'p4_dynamic', 'X1': 'x1_adjacency', 'X2': 'x2_algorithms', 'P5': 'p5_platforms'}


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def stage_catalog():
    return read_json(VERSIONS_ROOT / "stages.json")["stages"]


def artifact_path(reference):
    """Resolve catalog references relative to this portable checkout."""
    path = Path(reference)
    return path if path.is_absolute() else PROJECT_ROOT / path


def new_run_directory(stage, run_id):
    if stage not in PROFILES:
        raise ValueError(f"No runnable profile for {stage}")
    if not re.fullmatch(r"[\w\-]+", run_id):
        raise ValueError("run-id must be a single name containing letters, numbers, underscores or hyphens")
    path = RESULTS_ROOT / STAGE_DIRS[stage] / "runs" / run_id
    path.mkdir(parents=True, exist_ok=False)
    return path
