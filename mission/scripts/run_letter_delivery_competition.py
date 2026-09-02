#!/usr/bin/env python3
"""Complete competition mission with adaptive letter placement and return.

This local integration preserves the verified outbound/pick prefix, replaces
the fixed right-shift placement with letter-guided centering (max 2.4 m), then
returns by the measured lateral distance and the proven doorway sequence.
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
    FullConfig,
    arm_argv,
    initialization_argv,
    init_report_is_stable,
    pick_argv,
    pick_report_is_stable,
    place_report_is_stable,
    spine_initialization_argv,
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
    run_streamed_command,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DELIVERY_CONFIG = REPO_ROOT / "mission" / "config" / "letter_delivery.json"


class Phase(str, Enum):
    CREATED = "CREATED"
    INITIALIZING_DUAL_ARMS = "INITIALIZING_DUAL_ARMS"
    INITIALIZING_SPINE = "INITIALIZING_SPINE"
    OUTBOUND_BASE_RUNNING = "OUTBOUND_BASE_RUNNING"
    PICK_RUNNING = "PICK_RUNNING"
    OBJECT_HELD = "OBJECT_HELD"
    PREPARING_LABEL_SEARCH = "PREPARING_LABEL_SEARCH"
    LABEL_SEARCH_RUNNING = "LABEL_SEARCH_RUNNING"
    TARGET_CENTERED = "TARGET_CENTERED"
    LABEL_PLACE_RUNNING = "LABEL_PLACE_RUNNING"
    OBJECT_PLACED = "OBJECT_PLACED"
    RETURN_RUNNING = "RETURN_RUNNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    INTERRUPTED = "INTERRUPTED"


@dataclass(frozen=True)
class DeliveryPlan:
    target_letter: str
    requested_row: str
    alphabet: str
    maximum_right_m: float
    minimum_detection_right_m: float
    search_speed_mps: float
    refine_speed_mps: float
    center_tolerance_norm: float
    stable_frames: int
    minimum_confidence: float
    row_split_y_norm: float
    camera_topic: str
    near_forward_m: float
    near_down_m: float
    far_forward_m: float
    far_down_m: float
    return_turn_deg: float
    door_before_m: float
    door_forward_m: float


def emit(event: str, **values) -> None:
    print(
        "LETTER_MISSION="
        + json.dumps({"event": event, **values}, ensure_ascii=False, separators=(",", ":")),
        flush=True,
    )


def load_plan(path: Path, target_override: str | None, row_override: str | None) -> DeliveryPlan:
    value = json.loads(path.read_text(encoding="utf-8"))
    target = value["target"]
    search = value["search"]
    placement = value["placement"]
    returning = value["return"]
    letter = (target_override or target["letter"]).strip().upper()
    row = row_override or target["row"]
    alphabet = str(target["alphabet"]).upper()
    if len(letter) != 1 or letter not in alphabet:
        raise MissionError("target letter must be one character in configured alphabet")
    if row not in {"auto", "near", "far"}:
        raise MissionError("target row must be auto, near, or far")
    plan = DeliveryPlan(
        target_letter=letter,
        requested_row=row,
        alphabet=alphabet,
        maximum_right_m=float(search["maximum_right_m"]),
        minimum_detection_right_m=float(search.get("minimum_detection_right_m", 0.40)),
        search_speed_mps=float(search["search_speed_mps"]),
        refine_speed_mps=float(search["refine_speed_mps"]),
        center_tolerance_norm=float(search["center_tolerance_norm"]),
        stable_frames=int(search["stable_frames"]),
        minimum_confidence=float(search["minimum_confidence"]),
        row_split_y_norm=float(search["row_split_y_norm"]),
        camera_topic=str(search["camera_topic"]),
        near_forward_m=float(placement["near"]["forward_extension_m"]),
        near_down_m=float(placement["near"]["descent_m"]),
        far_forward_m=float(placement["far"]["forward_extension_m"]),
        far_down_m=float(placement["far"]["descent_m"]),
        return_turn_deg=float(returning["turn_ccw_deg"]),
        door_before_m=float(returning["door_before_m"]),
        door_forward_m=float(returning["door_forward_m"]),
    )
    if not 0.20 <= plan.maximum_right_m <= 2.40:
        raise MissionError("maximum right search must remain in [0.20, 2.40] m")
    if not 0.0 <= plan.minimum_detection_right_m < plan.maximum_right_m:
        raise MissionError("minimum detection distance must precede the right search limit")
    if not 0.095 <= plan.near_forward_m <= 0.105:
        raise MissionError("near row must include the requested 0.10 m forward placement offset")
    if not 0.255 <= plan.far_forward_m <= 0.265:
        raise MissionError("far row must include the requested 0.10 m forward placement offset")
    if abs(plan.door_before_m - 0.50) > 1e-9 or abs(plan.door_forward_m - 1.20) > 1e-9:
        raise MissionError("return must preserve door centre -> 0.50 m -> 1.20 m")
    return plan


def save(path: Path, run_id: str, phase: Phase, plan: DeliveryPlan, **values) -> None:
    payload = {
        "version": 1,
        "run_id": run_id,
        "phase": phase.value,
        "updated_unix_s": time.time(),
        "target_letter": plan.target_letter,
        "requested_row": plan.requested_row,
        **values,
    }
    atomic_write_json(path, payload)
    emit("phase", run_id=run_id, phase=phase.value)


def remote_argv(config: FullConfig, command: str) -> list[str]:
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
        command,
    ]


def prepare_search_argv(
    config: FullConfig, run_id: str, backward_m: float = 0.25
) -> list[str]:
    runner = shlex.quote(f"{config.base_root}/scripts/13_run_post_grasp_route.sh")
    state = shlex.quote(f"/tmp/tmr_letter_prepare_{run_id}.json")
    return remote_argv(
        config,
        f"{runner} --execute --fresh-start --stop-before-right "
        f"--backward-m {backward_m:.3f} --state-file {state}",
    )


def search_argv(
    config: FullConfig,
    plan: DeliveryPlan,
    run_id: str,
    plate_direct_place_on_d: bool = False,
) -> list[str]:
    runner = shlex.quote(f"{config.base_root}/scripts/14_run_letter_guided_search.sh")
    arguments = [
        "--execute",
        "--target-letter", plan.target_letter,
        "--row", plan.requested_row,
        "--alphabet", plan.alphabet,
        "--camera-topic", plan.camera_topic,
        "--max-right-m", f"{plan.maximum_right_m:.3f}",
        "--min-detection-right-m", f"{plan.minimum_detection_right_m:.3f}",
        "--search-speed-mps", f"{plan.search_speed_mps:.4f}",
        "--refine-speed-mps", f"{plan.refine_speed_mps:.4f}",
        "--center-tolerance-norm", f"{plan.center_tolerance_norm:.4f}",
        "--post-center-right-m", "0.080",
        "--stable-frames", str(plan.stable_frames),
        "--minimum-confidence", f"{plan.minimum_confidence:.4f}",
        "--row-split-y-norm", f"{plan.row_split_y_norm:.4f}",
        "--state-file", f"/tmp/tmr_letter_search_{run_id}.json",
        "--evidence-image", f"/tmp/tmr_letter_search_{run_id}_authorized.jpg",
    ]
    if plate_direct_place_on_d:
        arguments.append("--plate-direct-place-on-d")
    return remote_argv(config, runner + " " + " ".join(shlex.quote(item) for item in arguments))


def placement_values(plan: DeliveryPlan, row: str) -> tuple[float, float]:
    if row == "near":
        return plan.near_forward_m, plan.near_down_m
    if row == "far":
        return plan.far_forward_m, plan.far_down_m
    raise MissionError(f"vision returned invalid placement row: {row}")


def letter_place_argv(
    config: FullConfig, plan: DeliveryPlan, row: str, run_id: str
) -> list[str]:
    forward, down = placement_values(plan, row)
    return arm_argv(
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
    )


def return_argv(
    config: FullConfig,
    plan: DeliveryPlan,
    right_m: float,
    run_id: str,
) -> list[str]:
    runner = shlex.quote(f"{config.base_root}/scripts/15_run_return_from_letter.sh")
    arguments = [
        "--execute",
        "--fresh-start",
        "--left-m", f"{right_m:.4f}",
        "--turn-cw-deg", f"{plan.return_turn_deg:.3f}",
        "--disable-collision-guard",
        "--state-file", f"/tmp/tmr_letter_return_{run_id}.json",
    ]
    return remote_argv(config, runner + " " + " ".join(shlex.quote(item) for item in arguments))


def preparation_report_ok(report: dict | None) -> bool:
    return bool(
        isinstance(report, dict)
        and report.get("status") == "complete"
        and report.get("next_stage") == 3
        and report.get("zero_command_latched") is True
        and len(report.get("reports", [])) == 3
    )


def search_report_ok(
    report: dict | None,
    plan: DeliveryPlan,
    plate_direct_place_on_d: bool = False,
) -> bool:
    direct_plate_mode = bool(
        plate_direct_place_on_d
        and isinstance(report, dict)
        and report.get("target_letter") == "D"
        and report.get("post_center_right_requested_m") == 0.0
    )
    center_limit = 0.222 if direct_plate_mode else plan.center_tolerance_norm + 0.002
    return bool(
        isinstance(report, dict)
        and report.get("status") == "success"
        and report.get("target_centered") is True
        and report.get("target_letter") == plan.target_letter
        and report.get("row") in {"near", "far"}
        and plan.minimum_detection_right_m - 0.04
        <= float(report.get("actual_right_m", -1.0))
        <= plan.maximum_right_m + 0.04
        and abs(float(report.get("center_error_norm", 1.0))) <= center_limit
        and report.get("evidence_saved") is True
        and report.get("zero_command_latched") is True
    )


def return_report_ok(report: dict | None) -> bool:
    door = report.get("door_report") if isinstance(report, dict) else None
    return bool(
        isinstance(report, dict)
        and report.get("status") == "complete"
        and report.get("phase") == "COMPLETE"
        and report.get("zero_command_latched") is True
        and isinstance(door, dict)
        and door.get("status") == "success"
        and door.get("final_state") == "FINAL_STOP"
    )


def extract_return_report(text: str) -> dict | None:
    decoder = json.JSONDecoder()
    match = None
    for index, character in enumerate(text):
        if character != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and value.get("phase") == "COMPLETE" and "door_report" in value:
            match = value
    return match


def strategy(
    config: FullConfig,
    plan: DeliveryPlan,
    checkpoint: Path,
    *,
    stop_after_place: bool = False,
    resume_from_object_held: bool = False,
    resume_from_search_start: bool = False,
) -> dict:
    phases = []
    if resume_from_search_start:
        phases.append(
            "resume: post-grasp retreat/turn/backward already complete -> preserve pose and begin B search"
        )
    elif resume_from_object_held:
        phases.append(
            "resume: object already held at standard height -> preserve both arms and skip initialization/outbound/pick"
        )
    else:
        phases.extend(
            [
                "startup: reset both arm joints -> restore spine to 0.70 m -> keep right inward parking and left calibrated pick-top hold",
                "outbound -> door midpoint -> 0.50 m before -> forward 1.20 m",
                "visual cup pick -> stable object hold",
            ]
        )
    phases.extend(
        [
        "retreat 1.70 m -> CCW 180 deg -> backward 0.25 m",
        f"right letter search <= {plan.maximum_right_m:.2f} m -> temporal center lock",
        "near: forward 0.10 m -> vertical place; far: forward 0.26 m -> vertical place -> retract",
        ]
    )
    if not stop_after_place:
        phases.extend(
            [
                "left by measured right distance -> CW 180 deg",
                "verify both door edges/midpoint -> 0.50 m before -> forward 1.20 m -> final zero hold",
            ]
        )
    return {
        "status": "dry_run",
        "motion_enabled": False,
        "target_letter": plan.target_letter,
        "requested_row": plan.requested_row,
        "phases": phases,
        "stop_after_place": bool(stop_after_place),
        "checkpoint": str(checkpoint),
        "base_host": config.base_host,
    }


def checkpoint_allows_pickup_resume(previous: dict | None) -> bool:
    """Only resume when the recorded run had already entered the pick phase."""
    return bool(
        isinstance(previous, dict)
        and previous.get("phase") == Phase.FAILED.value
        and previous.get("failed_phase") == Phase.PICK_RUNNING.value
    )


def run(args: argparse.Namespace) -> int:
    plan = load_plan(args.delivery_config, args.target_letter, args.row)
    config = FullConfig(
        base_host=args.base_host,
        base_root=args.base_root,
        arm_root=str(args.arm_root.resolve()),
        arm_env=args.arm_env,
        init_timeout_s=args.init_timeout_s,
        outbound_timeout_s=args.outbound_timeout_s,
        pick_timeout_s=args.pick_timeout_s,
        return_timeout_s=args.base_phase_timeout_s,
        place_timeout_s=args.place_timeout_s,
        transition_settle_s=args.transition_settle_s,
    )
    if not args.execute:
        print(
            json.dumps(
                strategy(
                    config,
                    plan,
                    args.checkpoint,
                    stop_after_place=args.stop_after_place,
                    resume_from_object_held=args.resume_from_object_held_confirmed,
                    resume_from_search_start=args.resume_from_search_start_confirmed,
                ),
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0
    previous = load_checkpoint(args.checkpoint)
    resume_modes = (
        int(args.resume_from_pickup_confirmed)
        + int(args.resume_from_object_held_confirmed)
        + int(args.resume_from_search_start_confirmed)
    )
    if resume_modes > 1 or (resume_modes and args.fresh_start_confirmed):
        raise MissionError("choose exactly one marked-start or resume mode")
    if args.resume_from_search_start_confirmed:
        # The flag is the operator's explicit assertion that the three-stage
        # post-grasp route completed and the base is stopped at search start.
        pass
    elif args.resume_from_object_held_confirmed:
        # This flag is the operator's explicit assertion that an object is
        # already securely held at the standard transport height.  Do not
        # initialize either arm and do not replay the pick.
        pass
    elif args.resume_from_pickup_confirmed:
        if not checkpoint_allows_pickup_resume(previous):
            raise MissionError(
                "pickup resume requires a checkpoint failed specifically in PICK_RUNNING"
            )
    elif previous is not None and not args.fresh_start_confirmed:
        raise MissionError(
            f"checkpoint phase is {previous.get('phase')}; refuse automatic replay without "
            "--fresh-start-confirmed after returning to the marked start"
        )
    with MissionRunLock(args.checkpoint.with_suffix(args.checkpoint.suffix + ".lock")):
        return run_locked(
            args,
            config,
            plan,
            resume_from_pickup=args.resume_from_pickup_confirmed,
            resume_from_object_held=args.resume_from_object_held_confirmed,
            resume_from_search_start=args.resume_from_search_start_confirmed,
        )


def run_locked(
    args: argparse.Namespace,
    config: FullConfig,
    plan: DeliveryPlan,
    *,
    resume_from_pickup: bool = False,
    resume_from_object_held: bool = False,
    resume_from_search_start: bool = False,
) -> int:
    run_id = uuid.uuid4().hex[:12]
    logs = args.log_dir / run_id
    phase = Phase.CREATED
    save(args.checkpoint, run_id, phase, plan)
    try:
        if resume_from_search_start:
            phase = Phase.PREPARING_LABEL_SEARCH
            save(
                args.checkpoint,
                run_id,
                phase,
                plan,
                object_held_confirmed_by_operator=True,
                post_grasp_route_confirmed=True,
                skipped_initialization=True,
                skipped_outbound=True,
                skipped_pick=True,
                skipped_prepare_search=True,
            )
            emit(
                "resume",
                run_id=run_id,
                from_phase=Phase.LABEL_SEARCH_RUNNING.value,
                skipped_phases=[
                    Phase.INITIALIZING_DUAL_ARMS.value,
                    Phase.INITIALIZING_SPINE.value,
                    Phase.OUTBOUND_BASE_RUNNING.value,
                    Phase.PICK_RUNNING.value,
                    Phase.PREPARING_LABEL_SEARCH.value,
                ],
            )
        elif resume_from_object_held:
            phase = Phase.OBJECT_HELD
            save(
                args.checkpoint,
                run_id,
                phase,
                plan,
                object_held_confirmed_by_operator=True,
                skipped_initialization=True,
                skipped_outbound=True,
                skipped_pick=True,
            )
            emit(
                "resume",
                run_id=run_id,
                from_phase=Phase.OBJECT_HELD.value,
                skipped_phases=[
                    Phase.INITIALIZING_DUAL_ARMS.value,
                    Phase.INITIALIZING_SPINE.value,
                    Phase.OUTBOUND_BASE_RUNNING.value,
                    Phase.PICK_RUNNING.value,
                ],
            )
        else:
            phase = Phase.INITIALIZING_DUAL_ARMS
            save(args.checkpoint, run_id, phase, plan)
            result = run_streamed_command(
                "dual-init", initialization_argv(config), config.init_timeout_s, logs / "dual_init.log"
            )
            report = extract_last_json_object(result.output)
            if result.returncode != 0 or not init_report_is_stable(report):
                raise MissionError("dual-arm initialization did not prove two stable holds")

            phase = Phase.INITIALIZING_SPINE
            save(args.checkpoint, run_id, phase, plan)
            result = run_streamed_command(
                "spine-init",
                spine_initialization_argv(config),
                config.init_timeout_s,
                logs / "spine_init.log",
            )
            spine_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not spine_report_is_stable(spine_report):
                raise MissionError("spine initialization did not prove the 0.70 m height")

            if resume_from_pickup:
                emit(
                    "resume",
                    run_id=run_id,
                    from_phase=Phase.PICK_RUNNING.value,
                    skipped_phase=Phase.OUTBOUND_BASE_RUNNING.value,
                )
            else:
                phase = Phase.OUTBOUND_BASE_RUNNING
                save(args.checkpoint, run_id, phase, plan)
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
                    "outbound", build_base_argv(outbound_config, run_id),
                    config.outbound_timeout_s, logs / "outbound.log"
                )
                report = extract_last_json_object(result.output)
                if not base_report_is_stable(report):
                    raise MissionError("outbound route did not prove FINAL_STOP")

            phase = Phase.PICK_RUNNING
            save(args.checkpoint, run_id, phase, plan)
            result = run_streamed_command(
                "pick", pick_argv(config), config.pick_timeout_s, logs / "pick.log"
            )
            stable, pick_event = pick_report_is_stable(result.output, result.returncode)
            if not stable:
                raise MissionError("pick did not reach cycle_complete with stable hold")
            phase = Phase.OBJECT_HELD
            save(args.checkpoint, run_id, phase, plan, pick_event=pick_event)

        if not resume_from_search_start:
            phase = Phase.PREPARING_LABEL_SEARCH
            save(args.checkpoint, run_id, phase, plan)
            result = run_streamed_command(
                "prepare-search", prepare_search_argv(config, run_id),
                args.base_phase_timeout_s, logs / "prepare_search.log"
            )
            report = extract_last_json_object(result.output)
            if result.returncode != 0 or not preparation_report_ok(report):
                raise MissionError("retreat/turn/backward prefix did not complete exactly three stages")

        phase = Phase.LABEL_SEARCH_RUNNING
        save(args.checkpoint, run_id, phase, plan)
        result = run_streamed_command(
            "letter-search", search_argv(config, plan, run_id),
            args.search_timeout_s, logs / "letter_search.log"
        )
        search_report = extract_last_json_object(result.output)
        if result.returncode != 0 or not search_report_ok(search_report, plan):
            raise MissionError("target letter was not stably centered within the 2.4 m bound")
        assert search_report is not None
        row = str(search_report["row"])
        right_m = max(0.0, float(search_report["actual_right_m"]))
        phase = Phase.TARGET_CENTERED
        save(
            args.checkpoint, run_id, phase, plan,
            selected_row=row, measured_right_m=right_m, search_report=search_report,
        )

        phase = Phase.LABEL_PLACE_RUNNING
        save(args.checkpoint, run_id, phase, plan, selected_row=row, measured_right_m=right_m)
        result = run_streamed_command(
            "letter-place", letter_place_argv(config, plan, row, run_id),
            config.place_timeout_s, logs / "letter_place.log"
        )
        place_report = extract_last_json_object(result.output)
        if result.returncode != 0 or not place_report_is_stable(place_report):
            raise MissionError("row-specific place did not verify release, recovery, and stable hold")
        phase = Phase.OBJECT_PLACED
        save(
            args.checkpoint, run_id, phase, plan,
            selected_row=row, measured_right_m=right_m, place_report=place_report,
        )

        if args.stop_after_place:
            phase = Phase.COMPLETE
            save(
                args.checkpoint,
                run_id,
                phase,
                plan,
                selected_row=row,
                measured_right_m=right_m,
                place_report=place_report,
                returned_to_table=False,
                stopped_after_place=True,
            )
            emit(
                "complete",
                run_id=run_id,
                target_letter=plan.target_letter,
                selected_row=row,
                measured_right_m=right_m,
                returned_to_table=False,
                stopped_after_place=True,
            )
            return 0

        phase = Phase.RETURN_RUNNING
        save(args.checkpoint, run_id, phase, plan, measured_right_m=right_m)
        result = run_streamed_command(
            "return", return_argv(config, plan, right_m, run_id),
            args.return_timeout_s, logs / "return.log"
        )
        return_report = extract_return_report(result.output)
        if result.returncode != 0 or not return_report_ok(return_report):
            raise MissionError("measured-left/CW-180/door return did not prove final zero hold")
        phase = Phase.COMPLETE
        save(
            args.checkpoint, run_id, phase, plan,
            selected_row=row, measured_right_m=right_m, return_report=return_report,
        )
        emit(
            "complete", run_id=run_id, target_letter=plan.target_letter,
            selected_row=row, measured_right_m=right_m, returned_to_table=True,
        )
        return 0
    except KeyboardInterrupt:
        save(args.checkpoint, run_id, Phase.INTERRUPTED, plan, interrupted_phase=phase.value)
        return 130
    except Exception as exc:
        save(
            args.checkpoint, run_id, Phase.FAILED, plan,
            failed_phase=phase.value, error=f"{type(exc).__name__}: {exc}",
        )
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start-confirmed", action="store_true")
    parser.add_argument(
        "--resume-from-pickup-confirmed",
        action="store_true",
        help="after a PICK_RUNNING failure, rerun initialization and continue at pickup",
    )
    parser.add_argument(
        "--resume-from-object-held-confirmed",
        action="store_true",
        help="object is already held at transport height; skip init, outbound, and pick",
    )
    parser.add_argument(
        "--resume-from-search-start-confirmed",
        action="store_true",
        help="object is held and the post-grasp retreat/turn/backward route is already complete",
    )
    parser.add_argument(
        "--stop-after-place",
        action="store_true",
        help="stop with zero base velocity after release instead of running the return route",
    )
    parser.add_argument("--target-letter")
    parser.add_argument("--row", choices=("auto", "near", "far"))
    parser.add_argument("--delivery-config", type=Path, default=DEFAULT_DELIVERY_CONFIG)
    parser.add_argument("--base-host", default="tmr-user@172.16.0.50")
    parser.add_argument("--base-root", default="/home/tmr-user/tmr_cycle")
    parser.add_argument("--arm-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--arm-env", default="/home/aup/tmr_env.sh")
    parser.add_argument("--init-timeout-s", type=float, default=180.0)
    parser.add_argument("--outbound-timeout-s", type=float, default=180.0)
    parser.add_argument("--pick-timeout-s", type=float, default=240.0)
    parser.add_argument("--base-phase-timeout-s", type=float, default=160.0)
    parser.add_argument("--search-timeout-s", type=float, default=100.0)
    parser.add_argument("--place-timeout-s", type=float, default=200.0)
    parser.add_argument("--return-timeout-s", type=float, default=220.0)
    parser.add_argument("--transition-settle-s", type=float, default=0.5)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path.home() / ".tmr_letter_delivery" / "state.json",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path.home() / ".tmr_letter_delivery" / "logs",
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
