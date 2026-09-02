#!/usr/bin/env python3
"""Deliver cup, bowl, and plate from the marked start to parameterized letters.

Default assignment: cup -> B, bowl -> A, plate -> D.  Execution is strictly
ordered.  The first two objects return to the pickup table using the measured
letter-search distance; the final plate placement stops at its target.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
from pathlib import Path, PurePosixPath
import shlex
import sys
import time
import uuid


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_full_competition_cycle import (  # noqa: E402
    FullConfig,
    init_report_is_stable,
    pick_report_is_stable,
    place_report_is_stable,
    spine_report_is_stable,
)
from run_letter_delivery_competition import (  # noqa: E402
    DEFAULT_DELIVERY_CONFIG,
    DeliveryPlan,
    extract_return_report,
    load_plan,
    placement_values,
    preparation_report_ok,
    prepare_search_argv,
    return_argv,
    return_report_ok,
    search_argv,
    search_report_ok,
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
    run_streamed_command,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ARM_HOST = "aup@172.16.0.100"


@dataclass(frozen=True)
class ObjectDelivery:
    name: str
    letter: str
    pick_script: str
    pick_descent_m: float
    plan: DeliveryPlan


OBJECT_SPECS = (
    ("cup", "grasp/scripts/run_streamed_live_pick_cycle.py", 0.340),
    ("bowl", "grasp/scripts/run_streamed_food_bowl_pick_cycle.py", 0.360),
    ("plate", "grasp/scripts/run_streamed_plate_pick_cycle.py", 0.375),
)


def emit(event: str, **values) -> None:
    print(
        "THREE_OBJECT_MISSION="
        + json.dumps({"event": event, **values}, ensure_ascii=False, separators=(",", ":")),
        flush=True,
    )


def normalize_letter(value: str, label: str) -> str:
    letter = value.strip().upper()
    if len(letter) != 1 or not ("A" <= letter <= "Z"):
        raise MissionError(f"{label} must be one A-Z letter")
    return letter


def build_tasks(
    template: DeliveryPlan,
    cup_letter: str,
    bowl_letter: str,
    plate_letter: str,
) -> list[ObjectDelivery]:
    letters = [
        normalize_letter(cup_letter, "cup letter"),
        normalize_letter(bowl_letter, "bowl letter"),
        normalize_letter(plate_letter, "plate letter"),
    ]
    if len(set(letters)) != 3:
        raise MissionError("cup, bowl, and plate letters must be distinct")
    # Classify against every label used at the venue before checking the
    # requested target.  Restricting this to ABD forced a visible E card to be
    # reported as A and authorized an early placement.
    venue_alphabet = "ABCDE"
    alphabet = "".join(dict.fromkeys(venue_alphabet + "".join(letters)))
    tasks = []
    for (name, pick_script, pick_descent_m), letter in zip(OBJECT_SPECS, letters):
        plan = replace(
            template,
            target_letter=letter,
            requested_row="auto",
            alphabet=alphabet,
            near_down_m=pick_descent_m,
            far_down_m=pick_descent_m,
        )
        tasks.append(
            ObjectDelivery(
                name=name,
                letter=letter,
                pick_script=pick_script,
                pick_descent_m=pick_descent_m,
                plan=plan,
            )
        )
    return tasks


def remote_arm_argv(
    config: FullConfig,
    relative_script: str,
    arguments: list[str],
    arm_host: str = DEFAULT_ARM_HOST,
) -> list[str]:
    root = shlex.quote(config.arm_root)
    environment = shlex.quote(config.arm_env)
    script = shlex.quote(str(PurePosixPath(config.arm_root) / relative_script))
    tail = " ".join(shlex.quote(value) for value in arguments)
    payload = (
        f"set -eo pipefail; source {environment}; export PYTHONUNBUFFERED=1; "
        f"cd {root}; exec python3 {script} {tail}"
    )
    # The competition coordinator is normally launched on the arm computer.
    # In that case execute arm-local phases directly; requiring the robot to
    # SSH back into itself caused initialization to fail on hosts without a
    # self-authorized key.  Windows/Codex orchestration still uses SSH.
    if (
        sys.platform != "win32"
        and arm_host == DEFAULT_ARM_HOST
        and Path(config.arm_root).resolve() == REPO_ROOT.resolve()
    ) or arm_host in {"local", "localhost"}:
        return ["bash", "-lc", payload]
    return [
        "ssh",
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=5",
        "-o", "ServerAliveInterval=2",
        "-o", "ServerAliveCountMax=3",
        arm_host,
        "bash -lc " + shlex.quote(payload),
    ]


def initialization_argv(
    config: FullConfig, arm_host: str = DEFAULT_ARM_HOST
) -> list[str]:
    return remote_arm_argv(
        config,
        "grasp/scripts/initialize_dual_arm_pick_pose.py",
        ["--execute"],
        arm_host,
    )


def spine_initialization_argv(
    config: FullConfig, arm_host: str = DEFAULT_ARM_HOST
) -> list[str]:
    return remote_arm_argv(
        config,
        "grasp/scripts/initialize_spine_height.py",
        ["--execute", "--target-m", "0.700"],
        arm_host,
    )


def object_pick_argv(
    config: FullConfig,
    task: ObjectDelivery,
    arm_host: str = DEFAULT_ARM_HOST,
) -> list[str]:
    return remote_arm_argv(
        config,
        task.pick_script,
        ["--force-restore-top"],
        arm_host,
    )


def letter_place_argv(
    config: FullConfig,
    task: ObjectDelivery,
    row: str,
    run_id: str,
    arm_host: str = DEFAULT_ARM_HOST,
) -> list[str]:
    forward, down = placement_values(task.plan, row)
    if task.name == "bowl":
        forward = 0.07
    return remote_arm_argv(
        config,
        "grasp/scripts/run_streamed_live_place_cycle.py",
        [
            "--execute",
            "--fresh-start",
            "--placement-row", row,
            "--forward-m", f"{forward:.3f}",
            "--down-m", f"{down:.3f}",
            "--state-file", f"/tmp/tmr_letter_place_{run_id}.json",
        ],
        arm_host,
    )


def outbound_argv(config: FullConfig, run_id: str) -> list[str]:
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


def control_mode_argv(config: FullConfig, mode: str) -> list[str]:
    if mode not in {"mission", "teleop"}:
        raise MissionError(f"invalid base control mode: {mode}")
    runner = shlex.quote(f"{config.base_root}/scripts/17_control_mode.sh")
    ensure = shlex.quote(f"{config.base_root}/scripts/19_ensure_navigation_stack.sh")
    command = f"{runner} {mode}"
    if mode == "mission":
        # A fresh competition invocation must be self-sufficient when the
        # base stack is absent, yet reuse a healthy stack without restarting
        # hardware or creating duplicate command adapters.
        command = f"{ensure} && {command}"
    return [
        "ssh",
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=5",
        "-o", "ServerAliveInterval=2",
        "-o", "ServerAliveCountMax=3",
        config.base_host,
        command,
    ]


def control_mode_report_ok(report: dict | None, mode: str) -> bool:
    return bool(
        isinstance(report, dict)
        and report.get("status") == "success"
        and report.get("mode") == mode
        and report.get("teleop_velocity_enabled") is (mode == "teleop")
        and report.get("mission_lease") is (mode == "mission")
    )


def assignment(tasks: list[ObjectDelivery]) -> dict[str, str]:
    return {task.name: task.letter for task in tasks}


def strategy(config: FullConfig, tasks: list[ObjectDelivery], checkpoint: Path) -> dict:
    phases = [
        "disable teleop velocity and acquire the exclusive mission lease",
        "initialize both arms and spine at the marked start",
        "start -> pickup table through the verified doorway route",
    ]
    for index, task in enumerate(tasks):
        phases.extend(
            [
                f"{task.name}: RGB visual align -> down {task.pick_descent_m:.3f} m -> close once -> lift",
                f"{task.name}: retreat 1.70 m -> CCW 180 deg -> backward 0.25 m",
                f"{task.name}: search {task.letter} right <= {task.plan.maximum_right_m:.2f} m and center",
                f"{task.name}: auto near/far place at {task.letter}, down {task.pick_descent_m:.3f} m",
            ]
        )
        if index < len(tasks) - 1:
            phases.append(
                f"{task.name}: measured left + 0.20 m -> CW 180 deg -> verify doorway -> pickup table"
            )
    phases.append("plate placement complete -> final zero hold -> restore Xbox teleoperation")
    return {
        "status": "dry_run",
        "motion_enabled": False,
        "assignment": assignment(tasks),
        "recognition_alphabet": tasks[0].plan.alphabet,
        "phases": phases,
        "checkpoint": str(checkpoint),
        "base_host": config.base_host,
    }


def save_checkpoint(
    path: Path,
    run_id: str,
    phase: str,
    tasks: list[ObjectDelivery],
    **values,
) -> None:
    payload = {
        "version": 1,
        "run_id": run_id,
        "phase": phase,
        "updated_unix_s": time.time(),
        "assignment": assignment(tasks),
        **values,
    }
    atomic_write_json(path, payload)
    emit("phase", run_id=run_id, phase=phase, **{k: v for k, v in values.items() if k in {"object", "letter"}})


def run(args: argparse.Namespace) -> int:
    template = load_plan(args.delivery_config, None, None)
    tasks = build_tasks(template, args.cup_letter, args.bowl_letter, args.plate_letter)
    config = FullConfig(
        base_host=args.base_host,
        base_root=args.base_root,
        arm_root=args.arm_remote_root,
        arm_env=args.arm_env,
        init_timeout_s=args.init_timeout_s,
        outbound_timeout_s=args.outbound_timeout_s,
        pick_timeout_s=args.pick_timeout_s,
        return_timeout_s=args.return_timeout_s,
        place_timeout_s=args.place_timeout_s,
        transition_settle_s=args.transition_settle_s,
    )
    if not args.execute:
        print(json.dumps(strategy(config, tasks, args.checkpoint), ensure_ascii=False, indent=2))
        return 0
    start_modes = (
        bool(args.fresh_start_confirmed),
        bool(args.resume_at_pickup_confirmed),
        bool(getattr(args, "resume_after_cup_held_confirmed", False)),
        bool(getattr(args, "resume_object_at_pickup_confirmed", None)),
    )
    if sum(start_modes) > 1:
        raise MissionError("choose exactly one confirmed start/resume mode")
    if not any(start_modes):
        raise MissionError(
            "execution requires a confirmed fresh, pickup-table, held-cup, "
            "or object-at-pickup start"
        )
    previous = load_checkpoint(args.checkpoint)
    if previous is not None:
        emit("checkpoint_replaced", previous_phase=previous.get("phase"))

    required = [
        "grasp/scripts/initialize_dual_arm_pick_pose.py",
        "grasp/scripts/initialize_spine_height.py",
        "grasp/scripts/run_streamed_live_pick_cycle.py",
        "grasp/scripts/run_streamed_food_bowl_pick_cycle.py",
        "grasp/scripts/run_streamed_plate_pick_cycle.py",
        "grasp/scripts/run_streamed_live_place_cycle.py",
    ]
    missing = [name for name in required if not Path(REPO_ROOT, name).is_file()]
    if missing:
        raise MissionError("missing arm-local mission files: " + ", ".join(missing))

    with MissionRunLock(args.checkpoint.with_suffix(args.checkpoint.suffix + ".lock")):
        return run_locked(args, config, tasks)


def run_locked(
    args: argparse.Namespace,
    config: FullConfig,
    tasks: list[ObjectDelivery],
) -> int:
    run_id = uuid.uuid4().hex[:12]
    logs = args.log_dir / run_id
    phase = "CREATED"
    resume_object = getattr(args, "resume_object_at_pickup_confirmed", None)
    resume_index = 0
    if resume_object is not None:
        resume_index = next(
            index for index, task in enumerate(tasks) if task.name == resume_object
        )
    completed: list[str] = [task.name for task in tasks[:resume_index]]
    deliveries: dict[str, dict] = {
        task.name: {
            "letter": task.letter,
            "resumed_as_already_delivered": True,
        }
        for task in tasks[:resume_index]
    }
    save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)

    def execute(label: str, argv: list[str], timeout_s: float):
        return run_streamed_command(label, argv, timeout_s, logs / f"{label}.log")

    def settle() -> None:
        time.sleep(max(0.0, float(config.transition_settle_s)))

    restore_teleop = True
    try:
        phase = "LOCKING_MISSION_CONTROL"
        save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
        result = execute(
            "control-mission",
            control_mode_argv(config, "mission"),
            getattr(args, "base_startup_timeout_s", 150.0),
        )
        control_report = extract_last_json_object(result.output)
        if result.returncode != 0 or not control_mode_report_ok(control_report, "mission"):
            raise MissionError("failed to disable teleop and acquire mission control")

        arm_host = getattr(args, "arm_host", DEFAULT_ARM_HOST)
        resume_at_pickup = (
            getattr(args, "resume_at_pickup_confirmed", False)
            or resume_object is not None
        )
        resume_after_cup_held = getattr(args, "resume_after_cup_held_confirmed", False)
        if not resume_at_pickup and not resume_after_cup_held:
            phase = "INITIALIZING_DUAL_ARMS"
            save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
            result = execute(
                "dual-init", initialization_argv(config, arm_host), config.init_timeout_s
            )
            init_ok = result.returncode == 0 and init_report_is_stable(
                extract_last_json_object(result.output)
            )
            if not init_ok:
                emit("dual_init_transient_retry", run_id=run_id)
                time.sleep(1.0)
                result = execute(
                    "dual-init-retry",
                    initialization_argv(config, arm_host),
                    config.init_timeout_s,
                )
                init_ok = result.returncode == 0 and init_report_is_stable(
                    extract_last_json_object(result.output)
                )
            if not init_ok:
                raise MissionError("dual-arm initialization did not prove stable targets")
            settle()

            phase = "INITIALIZING_SPINE"
            save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
            result = execute(
                "spine-init", spine_initialization_argv(config, arm_host), config.init_timeout_s
            )
            spine_ok = result.returncode == 0 and spine_report_is_stable(
                extract_last_json_object(result.output)
            )
            if not spine_ok:
                emit("spine_init_transient_retry", run_id=run_id)
                time.sleep(0.8)
                result = execute(
                    "spine-init-retry",
                    spine_initialization_argv(config, arm_host),
                    config.init_timeout_s,
                )
                spine_ok = result.returncode == 0 and spine_report_is_stable(
                    extract_last_json_object(result.output)
                )
            if not spine_ok:
                raise MissionError("spine initialization did not prove the standard height")
            settle()

            phase = "OUTBOUND_BASE_RUNNING"
            save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
            result = execute("outbound", outbound_argv(config, run_id), config.outbound_timeout_s)
            outbound_report = extract_last_json_object(result.output)
            if not base_report_is_stable(outbound_report):
                raise MissionError("start-to-pickup route did not prove FINAL_STOP")
            if result.returncode != 0:
                emit("transport_exit_ignored_after_final_stop", returncode=result.returncode)
            settle()
        else:
            if resume_after_cup_held:
                phase = "CUP_HELD_RESUMED"
            elif resume_object is not None:
                phase = f"{resume_object.upper()}_AT_PICKUP_RESUMED"
            else:
                phase = "AT_PICKUP_RESUMED"
            save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)

        for index, task in enumerate(tasks):
            if index < resume_index:
                emit(
                    "object_skipped_already_delivered",
                    run_id=run_id,
                    object=task.name,
                    letter=task.letter,
                )
                continue
            stage_id = f"{run_id}_{task.name}"
            if index == 0 and resume_after_cup_held:
                pick_event = {
                    "event": "operator_confirmed_object_held_resume",
                    "object": task.name,
                }
                emit("cup_pick_skipped_object_already_held", run_id=run_id)
            else:
                phase = f"{task.name.upper()}_PICK_RUNNING"
                save_checkpoint(
                    args.checkpoint,
                    run_id,
                    phase,
                    tasks,
                    completed=completed,
                    object=task.name,
                    letter=task.letter,
                )
                result = execute(
                    f"{task.name}-pick",
                    object_pick_argv(config, task, arm_host),
                    config.pick_timeout_s,
                )
                stable, pick_event = pick_report_is_stable(result.output, result.returncode)
                if not stable:
                    raise MissionError(f"{task.name} pick did not reach stable held state")
                settle()

            phase = f"{task.name.upper()}_PREPARING_SEARCH"
            save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
            result = execute(
                f"{task.name}-prepare",
                prepare_search_argv(
                    config,
                    stage_id,
                    backward_m=0.25,
                ),
                args.base_phase_timeout_s,
            )
            prepare_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not preparation_report_ok(prepare_report):
                raise MissionError(f"{task.name} post-grasp route did not complete")

            phase = f"{task.name.upper()}_LETTER_SEARCH"
            save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
            result = execute(
                f"{task.name}-search",
                search_argv(
                    config,
                    task.plan,
                    stage_id,
                    plate_direct_place_on_d=(task.name == "plate" and task.letter == "D"),
                ),
                args.search_timeout_s,
            )
            search_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not search_report_ok(
                search_report,
                task.plan,
                plate_direct_place_on_d=(task.name == "plate" and task.letter == "D"),
            ):
                raise MissionError(f"{task.name} target {task.letter} was not stably centered")
            assert search_report is not None
            row = str(search_report["row"])
            right_m = max(0.0, float(search_report["actual_right_m"]))

            phase = f"{task.name.upper()}_PLACE_RUNNING"
            save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
            result = execute(
                f"{task.name}-place",
                letter_place_argv(config, task, row, stage_id, arm_host),
                config.place_timeout_s,
            )
            place_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not place_report_is_stable(place_report):
                raise MissionError(f"{task.name} release at {task.letter} was not verified")

            deliveries[task.name] = {
                "letter": task.letter,
                "row": row,
                "measured_right_m": right_m,
                "pick_event": pick_event,
                "place_report": place_report,
            }
            completed.append(task.name)
            phase = f"{task.name.upper()}_PLACED"
            save_checkpoint(
                args.checkpoint,
                run_id,
                phase,
                tasks,
                completed=completed,
                deliveries=deliveries,
            )
            settle()

            if index < len(tasks) - 1:
                phase = f"{task.name.upper()}_RETURN_RUNNING"
                save_checkpoint(args.checkpoint, run_id, phase, tasks, completed=completed)
                result = execute(
                    f"{task.name}-return",
                    return_argv(config, task.plan, right_m, stage_id),
                    args.return_timeout_s,
                )
                return_report = extract_return_report(result.output)
                if result.returncode != 0 or not return_report_ok(return_report):
                    raise MissionError(f"{task.name} return to pickup table was not verified")
                deliveries[task.name]["return_report"] = return_report
                settle()

        phase = "COMPLETE"
        save_checkpoint(
            args.checkpoint,
            run_id,
            phase,
            tasks,
            completed=completed,
            deliveries=deliveries,
            final_location=f"letter_{tasks[-1].letter}",
            returned_after_final=False,
        )
        emit(
            "complete",
            run_id=run_id,
            assignment=assignment(tasks),
            completed=completed,
            final_location=f"letter_{tasks[-1].letter}",
        )
        return 0
    except KeyboardInterrupt:
        save_checkpoint(
            args.checkpoint,
            run_id,
            "INTERRUPTED",
            tasks,
            completed=completed,
            interrupted_phase=phase,
        )
        return 130
    except Exception as exc:
        save_checkpoint(
            args.checkpoint,
            run_id,
            "FAILED",
            tasks,
            completed=completed,
            failed_phase=phase,
            error=f"{type(exc).__name__}: {exc}",
        )
        raise
    finally:
        if restore_teleop:
            try:
                result = execute(
                    "control-teleop", control_mode_argv(config, "teleop"), 20.0
                )
                report = extract_last_json_object(result.output)
                if result.returncode != 0 or not control_mode_report_ok(report, "teleop"):
                    emit("teleop_restore_failed", run_id=run_id)
                else:
                    emit("teleop_restored", run_id=run_id)
            except Exception as exc:
                emit(
                    "teleop_restore_failed",
                    run_id=run_id,
                    error=f"{type(exc).__name__}: {exc}",
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start-confirmed", action="store_true")
    parser.add_argument("--resume-at-pickup-confirmed", action="store_true")
    parser.add_argument("--resume-after-cup-held-confirmed", action="store_true")
    parser.add_argument(
        "--resume-object-at-pickup-confirmed",
        choices=tuple(spec[0] for spec in OBJECT_SPECS),
        help=(
            "operator-confirmed recovery: robot is at the pickup table and "
            "all preceding objects are already delivered"
        ),
    )
    parser.add_argument("--cup-letter", default="B")
    parser.add_argument("--bowl-letter", default="A")
    parser.add_argument("--plate-letter", default="D")
    parser.add_argument("--delivery-config", type=Path, default=DEFAULT_DELIVERY_CONFIG)
    parser.add_argument("--base-host", default="tmr-user@172.16.0.50")
    parser.add_argument("--base-root", default="/home/tmr-user/tmr_cycle")
    parser.add_argument("--arm-host", default=DEFAULT_ARM_HOST)
    parser.add_argument(
        "--arm-remote-root", default="/home/aup/tmr-mobile-manipulation"
    )
    parser.add_argument("--arm-env", default="/home/aup/tmr_env.sh")
    parser.add_argument("--init-timeout-s", type=float, default=180.0)
    parser.add_argument("--outbound-timeout-s", type=float, default=200.0)
    parser.add_argument("--pick-timeout-s", type=float, default=300.0)
    parser.add_argument("--base-phase-timeout-s", type=float, default=180.0)
    parser.add_argument(
        "--base-startup-timeout-s",
        type=float,
        default=150.0,
        help="bounded first-start allowance for base controller, LiDAR and SLAM",
    )
    parser.add_argument("--search-timeout-s", type=float, default=120.0)
    parser.add_argument("--place-timeout-s", type=float, default=220.0)
    parser.add_argument("--return-timeout-s", type=float, default=240.0)
    parser.add_argument("--transition-settle-s", type=float, default=0.35)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path.home() / ".tmr_three_object_delivery" / "state.json",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path.home() / ".tmr_three_object_delivery" / "logs",
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
