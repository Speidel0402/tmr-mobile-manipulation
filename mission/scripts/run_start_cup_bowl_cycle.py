#!/usr/bin/env python3
"""Run the start-to-table cup/bowl pick-and-return-in-place mission.

The coordinator is intended to run on the arm computer (172.16.0.100).  The
base route runs as one persistent process on 172.16.0.50.  At the table, the
left arm picks the cup, returns it to the same XY location, then independently
restores the calibrated top pose and repeats the operation for the food bowl.

Without ``--execute`` this program only prints the exact strategy.  An old
checkpoint blocks accidental route replay unless ``--fresh-start-confirmed``
is supplied after the base has physically returned to the marked start.
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

from run_full_competition_cycle import (  # noqa: E402
    SPINE_HOME_M,
    init_report_is_stable,
    spine_report_is_stable,
)
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
CUP_DESCENT_M = 0.340
BOWL_DESCENT_M = 0.360


class Phase(str, Enum):
    CREATED = "CREATED"
    INITIALIZING_DUAL_ARMS = "INITIALIZING_DUAL_ARMS"
    INITIALIZING_SPINE = "INITIALIZING_SPINE"
    READY_TO_DEPART = "READY_TO_DEPART"
    OUTBOUND_BASE_RUNNING = "OUTBOUND_BASE_RUNNING"
    AT_PICKUP_TABLE = "AT_PICKUP_TABLE"
    CUP_PICK_RUNNING = "CUP_PICK_RUNNING"
    CUP_HELD = "CUP_HELD"
    CUP_PLACE_RUNNING = "CUP_PLACE_RUNNING"
    CUP_RELEASED_AT_ORIGIN = "CUP_RELEASED_AT_ORIGIN"
    BOWL_PICK_RUNNING = "BOWL_PICK_RUNNING"
    BOWL_HELD = "BOWL_HELD"
    BOWL_PLACE_RUNNING = "BOWL_PLACE_RUNNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    INTERRUPTED = "INTERRUPTED"


@dataclass(frozen=True)
class CycleConfig:
    base_host: str
    base_root: str
    arm_root: str
    arm_env: str
    init_timeout_s: float
    outbound_timeout_s: float
    pick_timeout_s: float
    place_timeout_s: float
    transition_settle_s: float


def emit(event: str, **values) -> None:
    print(
        "CUP_BOWL_MISSION="
        + json.dumps(
            {"event": event, **values},
            ensure_ascii=False,
            separators=(",", ":"),
        ),
        flush=True,
    )


def save_checkpoint(path: Path, run_id: str, phase: Phase, **values) -> dict:
    payload = {
        "version": 1,
        "run_id": run_id,
        "phase": phase.value,
        "updated_unix_s": time.time(),
        "cup_descent_m": CUP_DESCENT_M,
        "bowl_descent_m": BOWL_DESCENT_M,
        **values,
    }
    atomic_write_json(path, payload)
    emit("phase", run_id=run_id, phase=phase.value)
    return payload


def arm_argv(config: CycleConfig, relative_script: str, arguments: list[str]) -> list[str]:
    root = shlex.quote(config.arm_root)
    environment = shlex.quote(config.arm_env)
    script = shlex.quote(str(Path(config.arm_root) / relative_script))
    tail = " ".join(shlex.quote(value) for value in arguments)
    command = (
        f"set -eo pipefail; source {environment}; export PYTHONUNBUFFERED=1; "
        f"cd {root}; exec python3 {script} {tail}"
    )
    return ["bash", "-lc", command]


def initialization_argv(config: CycleConfig) -> list[str]:
    return arm_argv(
        config,
        "grasp/scripts/initialize_dual_arm_pick_pose.py",
        ["--execute"],
    )


def spine_initialization_argv(config: CycleConfig) -> list[str]:
    return arm_argv(
        config,
        "grasp/scripts/initialize_spine_height.py",
        ["--execute", "--target-m", f"{SPINE_HOME_M:.3f}"],
    )


def outbound_argv(config: CycleConfig, run_id: str) -> list[str]:
    route_config = StrategyConfig(
        base_host=config.base_host,
        base_root=config.base_root,
        base_timeout_s=config.outbound_timeout_s,
        arm_root=config.arm_root,
        arm_env=config.arm_env,
        arm_timeout_s=config.pick_timeout_s,
        transition_settle_s=config.transition_settle_s,
    )
    return build_base_argv(route_config, run_id)


def pick_argv(config: CycleConfig, object_name: str) -> list[str]:
    scripts = {
        "cup": "grasp/scripts/run_streamed_live_pick_cycle.py",
        "bowl": "grasp/scripts/run_streamed_food_bowl_pick_cycle.py",
    }
    try:
        script = scripts[object_name]
    except KeyError as exc:
        raise ValueError(f"unsupported object: {object_name}") from exc
    return arm_argv(config, script, ["--force-restore-top"])


def place_argv(
    config: CycleConfig,
    object_name: str,
    run_id: str,
    descent_m: float,
) -> list[str]:
    state_file = f"/tmp/tmr_{object_name}_same_place_{run_id}.json"
    return arm_argv(
        config,
        "grasp/scripts/run_streamed_live_place_cycle.py",
        [
            "--execute",
            "--fresh-start",
            "--placement-row",
            "near",
            "--forward-m",
            "0",
            "--down-m",
            f"{descent_m:.3f}",
            "--state-file",
            state_file,
        ],
    )


def pick_report_is_stable(output: str, returncode: int) -> tuple[bool, dict | None]:
    events = parse_pick_events(output)
    failures = [
        event
        for event in events
        if event.get("event")
        in {"failure", "controller_restore_failed", "operator_stop"}
    ]
    successes = [event for event in events if event.get("event") == "success"]
    holds = [
        event
        for event in events
        if event.get("event") == "controller"
        and event.get("joint_impedance") == "restored_to_hold"
    ]
    complete = [
        event
        for event in events
        if event.get("event") == "cycle_complete"
        and event.get("final_state") == "DONE"
        and event.get("controller_hold") == "stable"
    ]
    stable = bool(
        returncode == 0
        and successes
        and holds
        and complete
        and not failures
    )
    return stable, complete[-1] if complete else None


def place_report_is_stable(report: dict | None) -> bool:
    if not isinstance(report, dict):
        return False
    open_report = report.get("open_report")
    down_report = report.get("down_report")
    up_report = report.get("up_report")
    return bool(
        report.get("status") == "complete"
        and report.get("phase") == "DONE"
        and report.get("released") is True
        and isinstance(report.get("stable_hold_recovery_attempt"), int)
        and isinstance(open_report, dict)
        and open_report.get("verified_open") is True
        and isinstance(down_report, dict)
        and down_report.get("label") == "down_before_open"
        and isinstance(up_report, dict)
        and up_report.get("label") == "up_after_open"
    )


def strategy(config: CycleConfig, checkpoint_path: Path) -> dict:
    return {
        "motion_enabled": False,
        "entrypoint": "mission/scripts/run_start_cup_bowl_cycle.py",
        "phases": [
            "initialize both arms: right inward parking, left calibrated pick-top hold",
            f"restore spine to {SPINE_HOME_M:.2f} m",
            "base: forward 0.85 m -> clockwise 90 deg",
            "base: live doorway midpoint -> stop 0.50 m before door -> advance 1.20 m",
            "cup: open -> restore top -> visual right-edge align -> down 0.340 m -> close -> lift",
            "cup: down 0.340 m at current XY -> open -> lift to captured top",
            "bowl: open -> restore top -> color/diameter/stability align -> down 0.360 m -> close -> lift",
            "bowl: down 0.360 m at current XY -> open -> lift to captured top",
        ],
        "base_host": config.base_host,
        "arm_host": "local 172.16.0.100",
        "cup_descent_m": CUP_DESCENT_M,
        "bowl_descent_m": BOWL_DESCENT_M,
        "base_collision_guard": "disabled, matching the verified outbound route",
        "checkpoint": str(checkpoint_path),
        "failure_rule": "a phase without explicit stable completion prevents every later phase",
        "replay_rule": "an existing checkpoint blocks route replay without an explicit start/resume confirmation",
    }


def run(args: argparse.Namespace) -> int:
    config = CycleConfig(
        base_host=args.base_host,
        base_root=args.base_root,
        arm_root=str(args.arm_root.resolve()),
        arm_env=args.arm_env,
        init_timeout_s=args.init_timeout_s,
        outbound_timeout_s=args.outbound_timeout_s,
        pick_timeout_s=args.pick_timeout_s,
        place_timeout_s=args.place_timeout_s,
        transition_settle_s=args.transition_settle_s,
    )
    if not args.execute:
        print(json.dumps(strategy(config, args.checkpoint), ensure_ascii=False, indent=2))
        return 0

    required = [
        "grasp/scripts/initialize_dual_arm_pick_pose.py",
        "grasp/scripts/initialize_spine_height.py",
        "grasp/scripts/run_streamed_live_pick_cycle.py",
        "grasp/scripts/run_streamed_food_bowl_pick_cycle.py",
        "grasp/scripts/run_streamed_live_place_cycle.py",
    ]
    missing = [name for name in required if not Path(config.arm_root, name).is_file()]
    if missing:
        raise MissionError("missing arm-local mission files: " + ", ".join(missing))
    previous = load_checkpoint(args.checkpoint)
    if args.fresh_start_confirmed and args.resume_after_init_confirmed:
        raise MissionError("choose either a complete fresh start or resume after proven initialization")
    if previous is not None and not (
        args.fresh_start_confirmed or args.resume_after_init_confirmed
    ):
        raise MissionError(
            f"checkpoint phase is {previous.get('phase')}; refuse automatic route replay. "
            "Use an explicit confirmation only after checking the corresponding physical state."
        )

    lock_path = args.checkpoint.with_suffix(args.checkpoint.suffix + ".lock")
    with MissionRunLock(lock_path):
        return run_locked(args, config)


def run_locked(args: argparse.Namespace, config: CycleConfig) -> int:
    run_id = uuid.uuid4().hex[:12]
    log_dir = args.log_dir / run_id
    current_phase = Phase.CREATED
    save_checkpoint(args.checkpoint, run_id, current_phase)

    def set_phase(phase: Phase, **values) -> None:
        nonlocal current_phase
        current_phase = phase
        save_checkpoint(args.checkpoint, run_id, phase, **values)

    def run_arm_phase(
        phase: Phase,
        label: str,
        argv: list[str],
        timeout_s: float,
    ):
        set_phase(phase)
        return run_streamed_command(label, argv, timeout_s, log_dir / f"{label}.log")

    try:
        if args.resume_after_init_confirmed:
            set_phase(
                Phase.READY_TO_DEPART,
                resume_after_init_confirmed=True,
            )
        else:
            result = run_arm_phase(
                Phase.INITIALIZING_DUAL_ARMS,
                "dual_init",
                initialization_argv(config),
                config.init_timeout_s,
            )
            init_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not init_report_is_stable(init_report):
                raise MissionError("dual-arm initialization did not prove two calibrated stable holds")

            result = run_arm_phase(
                Phase.INITIALIZING_SPINE,
                "spine_init",
                spine_initialization_argv(config),
                config.init_timeout_s,
            )
            spine_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not spine_report_is_stable(spine_report):
                raise MissionError("spine initialization did not prove the calibrated 0.70 m height")
            set_phase(
                Phase.READY_TO_DEPART,
                init_report=init_report,
                spine_report=spine_report,
            )

        set_phase(Phase.OUTBOUND_BASE_RUNNING)
        result = run_streamed_command(
            "outbound_base",
            outbound_argv(config, run_id),
            config.outbound_timeout_s,
            log_dir / "outbound_base.log",
        )
        base_report = extract_last_json_object(result.output)
        if not base_report_is_stable(base_report):
            raise MissionError("outbound route did not prove FINAL_STOP and latched zero speed")
        set_phase(Phase.AT_PICKUP_TABLE, base_report=base_report)
        time.sleep(min(0.5, max(0.0, config.transition_settle_s)))

        result = run_arm_phase(
            Phase.CUP_PICK_RUNNING,
            "cup_pick",
            pick_argv(config, "cup"),
            config.pick_timeout_s,
        )
        stable, cup_pick_report = pick_report_is_stable(result.output, result.returncode)
        if not stable:
            raise MissionError("cup pick did not prove contact, lift, and stable hold")
        set_phase(Phase.CUP_HELD, cup_pick_report=cup_pick_report)

        result = run_arm_phase(
            Phase.CUP_PLACE_RUNNING,
            "cup_same_place",
            place_argv(config, "cup", run_id, CUP_DESCENT_M),
            config.place_timeout_s,
        )
        cup_place_report = extract_last_json_object(result.output)
        if result.returncode != 0 or not place_report_is_stable(cup_place_report):
            raise MissionError("cup same-place release did not prove down-open-up and stable hold")
        set_phase(Phase.CUP_RELEASED_AT_ORIGIN, cup_place_report=cup_place_report)
        time.sleep(min(0.5, max(0.0, config.transition_settle_s)))

        result = run_arm_phase(
            Phase.BOWL_PICK_RUNNING,
            "bowl_pick",
            pick_argv(config, "bowl"),
            config.pick_timeout_s,
        )
        stable, bowl_pick_report = pick_report_is_stable(result.output, result.returncode)
        if not stable:
            raise MissionError("bowl pick did not prove contact, lift, and stable hold")
        set_phase(Phase.BOWL_HELD, bowl_pick_report=bowl_pick_report)

        result = run_arm_phase(
            Phase.BOWL_PLACE_RUNNING,
            "bowl_same_place",
            place_argv(config, "bowl", run_id, BOWL_DESCENT_M),
            config.place_timeout_s,
        )
        bowl_place_report = extract_last_json_object(result.output)
        if result.returncode != 0 or not place_report_is_stable(bowl_place_report):
            raise MissionError("bowl same-place release did not prove down-open-up and stable hold")

        set_phase(
            Phase.COMPLETE,
            cup_pick_report=cup_pick_report,
            cup_place_report=cup_place_report,
            bowl_pick_report=bowl_pick_report,
            bowl_place_report=bowl_place_report,
        )
        emit(
            "complete",
            run_id=run_id,
            base_at_table=True,
            cup_released_at_origin=True,
            bowl_released_at_origin=True,
        )
        return 0
    except KeyboardInterrupt:
        save_checkpoint(
            args.checkpoint,
            run_id,
            Phase.INTERRUPTED,
            interrupted_phase=current_phase.value,
        )
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
    parser.add_argument(
        "--resume-after-init-confirmed",
        action="store_true",
        help="skip dual-arm/spine initialization only when this same run already proved it",
    )
    parser.add_argument("--base-host", default="tmr-user@172.16.0.50")
    parser.add_argument("--base-root", default="/home/tmr-user/tmr_cycle")
    parser.add_argument("--arm-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--arm-env", default="/home/aup/tmr_env.sh")
    parser.add_argument("--init-timeout-s", type=float, default=180.0)
    parser.add_argument("--outbound-timeout-s", type=float, default=180.0)
    parser.add_argument("--pick-timeout-s", type=float, default=300.0)
    parser.add_argument("--place-timeout-s", type=float, default=180.0)
    parser.add_argument("--transition-settle-s", type=float, default=0.5)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path.home() / ".tmr_cup_bowl_cycle" / "state.json",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path.home() / ".tmr_cup_bowl_cycle" / "logs",
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
