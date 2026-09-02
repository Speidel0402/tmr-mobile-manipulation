#!/usr/bin/env python3
"""Deliver an already-held plate from the pickup table to letter B."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
MISSION_SCRIPT = REPO_ROOT / "mission" / "scripts" / "run_letter_delivery_competition.py"
PLATE_CONFIG = REPO_ROOT / "mission" / "config" / "held_plate_to_b.json"


def build_command(args: argparse.Namespace) -> list[str]:
    command = [sys.executable, str(MISSION_SCRIPT)]
    if args.execute:
        command.append("--execute")
    resume_flag = (
        "--resume-from-search-start-confirmed"
        if args.resume_from_search_start_confirmed
        else "--resume-from-object-held-confirmed"
    )
    command.extend(
        [
            resume_flag,
            "--stop-after-place",
            "--target-letter",
            "B",
            "--row",
            args.row,
            "--delivery-config",
            str(PLATE_CONFIG),
            "--base-host",
            args.base_host,
            "--base-root",
            args.base_root,
            "--arm-root",
            str(args.arm_root.resolve()),
            "--arm-env",
            args.arm_env,
            "--base-phase-timeout-s",
            f"{args.base_phase_timeout_s:.1f}",
            "--search-timeout-s",
            f"{args.search_timeout_s:.1f}",
            "--place-timeout-s",
            f"{args.place_timeout_s:.1f}",
            "--checkpoint",
            str(args.checkpoint),
            "--log-dir",
            str(args.log_dir),
        ]
    )
    return command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--resume-from-search-start-confirmed", action="store_true")
    parser.add_argument("--row", choices=("auto", "near", "far"), default="auto")
    parser.add_argument("--base-host", default="tmr-user@172.16.0.50")
    parser.add_argument("--base-root", default="/home/tmr-user/tmr_cycle")
    parser.add_argument("--arm-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--arm-env", default="/home/aup/tmr_env.sh")
    parser.add_argument("--base-phase-timeout-s", type=float, default=160.0)
    parser.add_argument("--search-timeout-s", type=float, default=100.0)
    parser.add_argument("--place-timeout-s", type=float, default=200.0)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path.home() / ".tmr_plate_to_b" / "state.json",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path.home() / ".tmr_plate_to_b" / "logs",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return subprocess.run(build_command(args), check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
