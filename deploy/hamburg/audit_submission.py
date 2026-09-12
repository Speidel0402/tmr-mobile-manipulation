#!/usr/bin/env python3
"""Reproduce the Hamburg portability audit against a Task 3 checkout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re


DEFAULT_ROOT = Path(__file__).resolve().parents[2]
TEXT_SUFFIXES = {".py", ".sh", ".ps1", ".md", ".yaml", ".yml", ".json"}
RULES = {
    "remote_or_split_host_execution": re.compile(
        r"\bssh\b|172\.16\.0\.(?:50|100)|DEFAULT_(?:BASE|ARM)_HOST", re.I
    ),
    "mixed_or_non_humble_ros": re.compile(r"\bJazzy\b|/opt/ros/jazzy", re.I),
    "shanghai_dds_override": re.compile(
        r"ROS_DOMAIN_ID[^\n]*(?:97|1)|rmw_cyclonedds_cpp|CYCLONEDDS_URI", re.I
    ),
    "legacy_head_or_wrist_camera_topic": re.compile(
        r"/head_camera/zed/(?!zed_node)|/wrist_camera_(?:left|right)/color/image_raw"
    ),
    "undocumented_hamburg_motion_interface": re.compile(
        r"/left_ik/|/right_ik/|/action_server/ptp_motion|"
        r"robotiq_gripper_controller/gripper_cmd|/franka_spine_node/|"
        r"/tmr_cycle/mission_cmd_vel"
    ),
    "ros_participant_created_in_phase_script": re.compile(r"rclpy\.init\s*\("),
    "runtime_process_spawn": re.compile(r"\bsubprocess\b|Popen\s*\("),
}


def iter_sources(root: Path):
    ignored = {".git", ".pytest_cache", "__pycache__"}
    for path in root.rglob("*"):
        if not path.is_file() or any(part in ignored for part in path.parts):
            continue
        if path.suffix.lower() in TEXT_SUFFIXES or path.name.startswith("Dockerfile"):
            if path.is_relative_to(root / "deploy" / "hamburg"):
                continue
            yield path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.repo_root.resolve()
    findings = {name: [] for name in RULES}
    scanned = 0
    for path in iter_sources(root):
        scanned += 1
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        relative = path.relative_to(root).as_posix()
        for name, pattern in RULES.items():
            if pattern.search(text):
                findings[name].append(relative)
    findings = {name: sorted(paths) for name, paths in findings.items()}
    report = {
        "schema_version": 1,
        "repo_root": str(root),
        "scanned_text_files": scanned,
        "direct_hamburg_compatibility": "blocked",
        "reason": (
            "The submitted Shanghai runtime must not be used as the Hamburg "
            "entrypoint; use deploy/hamburg and port motion to one startup participant."
        ),
        "findings": findings,
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
