#!/usr/bin/env python3
"""Run the complete outbound-pick-return-place competition mission.

Realtime loops stay onboard: base motion runs on 172.16.0.50 and arm motion
runs locally on 172.16.0.100.  This coordinator only performs phase hand-offs
after structured endpoint and stable-hold reports.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
import shlex
import sys
import time
import uuid


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_long_range_pick import (  # noqa: E402
    MissionError,
    MissionRunLock,
    StrategyConfig,
    atomic_write_json,
    base_report_is_stable,
    build_base_argv,
    extract_last_json_object,
    load_checkpoint,
    parse_pick_events,
    run_streamed_command,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
RIGHT_STAGE_1_M = 0.80
RIGHT_STAGE_2_M = 0.85
PLACE_DESCENT_M = 0.255


class Phase(str, Enum):
    CREATED = "CREATED"
    INITIALIZING_DUAL_ARMS = "INITIALIZING_DUAL_ARMS"
    DUAL_ARMS_READY = "DUAL_ARMS_READY"
    OUTBOUND_BASE_RUNNING = "OUTBOUND_BASE_RUNNING"
    AT_PICKUP = "AT_PICKUP"
    PICK_RUNNING = "PICK_RUNNING"
    OBJECT_HELD = "OBJECT_HELD"
    RETURN_BASE_RUNNING = "RETURN_BASE_RUNNING"
    AT_PLACE = "AT_PLACE"
    PLACE_RUNNING = "PLACE_RUNNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    INTERRUPTED = "INTERRUPTED"


@dataclass(frozen=True)
class FullConfig:
    base_host: str
    base_root: str
    arm_root: str
    arm_env: str
    init_timeout_s: float
    outbound_timeout_s: float
    pick_timeout_s: float
    return_timeout_s: float
    place_timeout_s: float
    transition_settle_s: float


def emit(event: str, **values) -> None:
    print(
        "FULL_MISSION="
        + json.dumps({"event": event, **values}, ensure_ascii=False, separators=(",", ":")),
        flush=True,
    )


def save_checkpoint(path: Path, run_id: str, phase: Phase, **values) -> dict:
    payload = {
        "version": 2,
        "run_id": run_id,
        "phase": phase.value,
        "updated_unix_s": time.time(),
        "right_segments_m": [RIGHT_STAGE_1_M, RIGHT_STAGE_2_M],
        "place_descent_m": PLACE_DESCENT_M,
        **values,
    }
    atomic_write_json(path, payload)
    emit("phase", run_id=run_id, phase=phase.value)
    return payload


def arm_argv(config: FullConfig, relative_script: str, arguments: list[str]) -> list[str]:
    root = shlex.quote(config.arm_root)
    environment = shlex.quote(config.arm_env)
    script = shlex.quote(str(Path(config.arm_root) / relative_script))
    tail = " ".join(shlex.quote(value) for value in arguments)
    command = (
        f"set -eo pipefail; source {environment}; export PYTHONUNBUFFERED=1; "
        f"cd {root}; exec python3 {script} {tail}"
    )
    return ["bash", "-lc", command]


def initialization_argv(config: FullConfig) -> list[str]:
    return arm_argv(
        config,
        "grasp/scripts/initialize_dual_arm_pick_pose.py",
        ["--execute"],
    )


def pick_argv(config: FullConfig) -> list[str]:
    return arm_argv(
        config,
        "grasp/scripts/run_streamed_live_pick_cycle.py",
        ["--force-restore-top"],
    )


def return_base_argv(config: FullConfig) -> list[str]:
    remote = (
        f"{shlex.quote(config.base_root)}/scripts/13_run_post_grasp_route.sh "
        f"--execute --fresh-start --right-first-m {RIGHT_STAGE_1_M:.2f} "
        f"--right-second-m {RIGHT_STAGE_2_M:.2f}"
    )
    return [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=5",
        "-o",
        "ServerAliveInterval=2",
        "-o",
        "ServerAliveCountMax=3",
        config.base_host,
        remote,
    ]


def place_argv(config: FullConfig) -> list[str]:
    return arm_argv(
        config,
        "grasp/scripts/run_streamed_live_place_cycle.py",
        ["--execute", "--fresh-start", "--down-m", f"{PLACE_DESCENT_M:.3f}"],
    )


def init_report_is_stable(report: dict | None) -> bool:
    return bool(
        isinstance(report, dict)
        and report.get("status") == "success"
        and report.get("both_stable_hold") is True
        and report.get("gripper_commanded") is False
        and len(report.get("reports", [])) == 2
    )


def pick_report_is_stable(output: str, returncode: int) -> tuple[bool, dict | None]:
    events = parse_pick_events(output)
    complete = [event for event in events if event.get("event") == "cycle_complete"]
    failures = [
        event
        for event in events
        if event.get("event") in {"failure", "controller_restore_failed", "operator_stop"}
    ]
    return returncode == 0 and bool(complete) and not failures, complete[-1] if complete else None


def return_report_is_stable(report: dict | None) -> bool:
    return bool(
        isinstance(report, dict)
        and report.get("status") == "complete"
        and report.get("next_stage") == 5
        and report.get("zero_command_latched") is True
        and len(report.get("reports", [])) == 5
    )


def place_report_is_stable(report: dict | None) -> bool:
    return bool(
        isinstance(report, dict)
        and report.get("status") == "complete"
        and report.get("phase") == "DONE"
        and report.get("released") is True
        and isinstance(report.get("stable_hold_recovery_attempt"), int)
    )


def strategy(config: FullConfig, checkpoint_path: Path) -> dict:
    return {
        "motion_enabled": False,
        "entrypoint": "mission/scripts/run_full_competition_cycle.py",
        "phases": [
            "dual arms -> unified pick-start pose; grippers untouched",
            "outbound: forward 0.85 m -> clockwise 90 deg -> live doorway midpoint -> door crossing 1.20 m",
            "pick: open left gripper -> force left top restore -> visual align -> descend 0.24 m -> close -> lift",
            "return: reverse 1.70 m -> counter-clockwise 180 deg -> reverse 0.25 m",
            "return: right 0.80 m -> right 0.85 m -> latch zero speed",
            "place: capture current top -> descend 0.255 m -> open -> lift -> stable hold",
        ],
        "base_host": config.base_host,
        "arm_host": "local 172.16.0.100",
        "right_segments_m": [RIGHT_STAGE_1_M, RIGHT_STAGE_2_M],
        "right_total_m": RIGHT_STAGE_1_M + RIGHT_STAGE_2_M,
        "place_descent_m": PLACE_DESCENT_M,
        "collision_guard": "disabled for outbound and return base routes",
        "checkpoint": str(checkpoint_path),
        "replay_rule": "an existing checkpoint blocks automatic phase replay",
    }


def run(args: argparse.Namespace) -> int:
    config = FullConfig(
        base_host=args.base_host,
        base_root=args.base_root,
        arm_root=str(args.arm_root.resolve()),
        arm_env=args.arm_env,
        init_timeout_s=args.init_timeout_s,
        outbound_timeout_s=args.outbound_timeout_s,
        pick_timeout_s=args.pick_timeout_s,
        return_timeout_s=args.return_timeout_s,
        place_timeout_s=args.place_timeout_s,
        transition_settle_s=args.transition_settle_s,
    )
    if not args.execute:
        print(json.dumps(strategy(config, args.checkpoint), ensure_ascii=False, indent=2))
        return 0

    required = [
        "grasp/scripts/initialize_dual_arm_pick_pose.py",
        "grasp/scripts/run_streamed_live_pick_cycle.py",
        "grasp/scripts/run_streamed_live_place_cycle.py",
    ]
    missing = [name for name in required if not Path(config.arm_root, name).is_file()]
    if missing:
        raise MissionError("missing arm-local mission files: " + ", ".join(missing))
    previous = load_checkpoint(args.checkpoint)
    if previous is not None and not args.fresh_start_confirmed:
        raise MissionError(
            f"checkpoint phase is {previous.get('phase')}; refuse automatic motion replay. "
            "Use --fresh-start-confirmed only after returning to the marked start."
        )

    lock_path = args.checkpoint.with_suffix(args.checkpoint.suffix + ".lock")
    with MissionRunLock(lock_path):
        return run_locked(args, config)


def run_locked(args: argparse.Namespace, config: FullConfig) -> int:
    run_id = uuid.uuid4().hex[:12]
    log_dir = args.log_dir / run_id
    current_phase = Phase.CREATED
    save_checkpoint(args.checkpoint, run_id, current_phase)

    try:
        current_phase = Phase.INITIALIZING_DUAL_ARMS
        save_checkpoint(args.checkpoint, run_id, current_phase)
        result = run_streamed_command(
            "dual-init",
            initialization_argv(config),
            config.init_timeout_s,
            log_dir / "dual_init.log",
        )
        report = extract_last_json_object(result.output)
        if result.returncode != 0 or not init_report_is_stable(report):
            raise MissionError("dual-arm initialization did not prove two stable holds")
        current_phase = Phase.DUAL_ARMS_READY
        save_checkpoint(args.checkpoint, run_id, current_phase, init_report=report)

        current_phase = Phase.OUTBOUND_BASE_RUNNING
        save_checkpoint(args.checkpoint, run_id, current_phase)
        outbound_config = StrategyConfig(
            base_host=config.base_host,
            base_root=config.base_root,
            base_timeout_s=config.outbound_timeout_s,
            arm_root=config.arm_root,
            arm_env=config.arm_env,
            arm_timeout_s=config.pick_timeout_s,
            transition_settle_s=config.transition_settle_s,
        )
        result = run_streamed_command(
            "outbound-base",
            build_base_argv(outbound_config, run_id),
            config.outbound_timeout_s,
            log_dir / "outbound_base.log",
        )
        report = extract_last_json_object(result.output)
        if not base_report_is_stable(report):
            raise MissionError("outbound route did not prove FINAL_STOP and zero lease")
        current_phase = Phase.AT_PICKUP
        save_checkpoint(args.checkpoint, run_id, current_phase, outbound_report=report)
        time.sleep(min(0.5, max(0.0, config.transition_settle_s)))

        current_phase = Phase.PICK_RUNNING
        save_checkpoint(args.checkpoint, run_id, current_phase)
        result = run_streamed_command(
            "pick",
            pick_argv(config),
            config.pick_timeout_s,
            log_dir / "pick.log",
        )
        stable, final_pick_event = pick_report_is_stable(result.output, result.returncode)
        if not stable:
            raise MissionError("pick did not reach cycle_complete with stable hold")
        current_phase = Phase.OBJECT_HELD
        save_checkpoint(args.checkpoint, run_id, current_phase, final_pick_event=final_pick_event)

        current_phase = Phase.RETURN_BASE_RUNNING
        save_checkpoint(args.checkpoint, run_id, current_phase)
        result = run_streamed_command(
            "return-base",
            return_base_argv(config),
            config.return_timeout_s,
            log_dir / "return_base.log",
        )
        report = extract_last_json_object(result.output)
        if not return_report_is_stable(report):
            raise MissionError("return route did not complete five stages with latched zero")
        current_phase = Phase.AT_PLACE
        save_checkpoint(args.checkpoint, run_id, current_phase, return_report=report)
        time.sleep(min(0.5, max(0.0, config.transition_settle_s)))

        current_phase = Phase.PLACE_RUNNING
        save_checkpoint(args.checkpoint, run_id, current_phase)
        result = run_streamed_command(
            "place",
            place_argv(config),
            config.place_timeout_s,
            log_dir / "place.log",
        )
        report = extract_last_json_object(result.output)
        if result.returncode != 0 or not place_report_is_stable(report):
            raise MissionError("place did not verify release, lift, and stable hold")
        current_phase = Phase.COMPLETE
        save_checkpoint(args.checkpoint, run_id, current_phase, place_report=report)
        emit(
            "complete",
            run_id=run_id,
            object_released=True,
            right_total_m=RIGHT_STAGE_1_M + RIGHT_STAGE_2_M,
            place_descent_m=PLACE_DESCENT_M,
        )
        return 0
    except KeyboardInterrupt:
        save_checkpoint(args.checkpoint, run_id, Phase.INTERRUPTED, interrupted_phase=current_phase.value)
        emit("interrupted", run_id=run_id, phase=current_phase.value)
        return 130
    except Exception as exc:
        save_checkpoint(
            args.checkpoint,
            run_id,
            Phase.FAILED,
            failed_phase=current_phase.value,
            error=f"{type(exc).__name__}: {exc}",
        )
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start-confirmed", action="store_true")
    parser.add_argument("--base-host", default="tmr-user@172.16.0.50")
    parser.add_argument("--base-root", default="/home/tmr-user/tmr_cycle")
    parser.add_argument("--arm-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--arm-env", default="/home/aup/tmr_env.sh")
    parser.add_argument("--init-timeout-s", type=float, default=180.0)
    parser.add_argument("--outbound-timeout-s", type=float, default=180.0)
    parser.add_argument("--pick-timeout-s", type=float, default=240.0)
    parser.add_argument("--return-timeout-s", type=float, default=180.0)
    parser.add_argument("--place-timeout-s", type=float, default=180.0)
    parser.add_argument("--transition-settle-s", type=float, default=0.5)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path.home() / ".tmr_full_competition" / "state.json",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path.home() / ".tmr_full_competition" / "logs",
    )
    return parser.parse_args()


def main() -> int:
    try:
        return run(parse_args())
    except Exception as exc:
        emit("aborted", error=f"{type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
