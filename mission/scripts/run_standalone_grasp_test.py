#!/usr/bin/env python3
"""Run one Shanghai-validated object grasp test without moving the base.

The test always initializes both arms and the lifting column before perception.
It can stop with the selected object held, or place it back at the same table
location using the proven down-open-up sequence.  Dry-run is the default.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
import uuid


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_full_competition_cycle import (  # noqa: E402
    init_report_is_stable,
    spine_report_is_stable,
)
from run_long_range_pick import (  # noqa: E402
    MissionError,
    MissionRunLock,
    atomic_write_json,
    extract_last_json_object,
    run_streamed_command,
)
from run_start_cup_bowl_cycle import (  # noqa: E402
    CycleConfig,
    arm_argv,
    initialization_argv,
    pick_argv,
    pick_report_is_stable,
    place_argv,
    place_report_is_stable,
    spine_initialization_argv,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
OBJECTS = {
    "cup": ("grasp/scripts/run_streamed_live_pick_cycle.py", 0.340),
    "bowl": ("grasp/scripts/run_streamed_food_bowl_pick_cycle.py", 0.360),
    "plate": ("grasp/scripts/run_streamed_plate_pick_cycle.py", 0.375),
}


def emit(event: str, **values) -> None:
    print(
        "STANDALONE_GRASP_TEST="
        + json.dumps({"event": event, **values}, ensure_ascii=False, separators=(",", ":")),
        flush=True,
    )


def build_config(args: argparse.Namespace) -> CycleConfig:
    return CycleConfig(
        base_host="unused",
        base_root="unused",
        arm_root=str(args.arm_root.resolve()),
        arm_env=args.arm_env,
        init_timeout_s=args.init_timeout_s,
        outbound_timeout_s=0.0,
        pick_timeout_s=args.pick_timeout_s,
        place_timeout_s=args.place_timeout_s,
        transition_settle_s=args.transition_settle_s,
    )


def object_pick_argv(config: CycleConfig, object_name: str) -> list[str]:
    script, descent_m = OBJECTS[object_name]
    if object_name in {"cup", "bowl"}:
        return pick_argv(config, object_name)
    return arm_argv(
        config,
        script,
        ["--force-restore-top", "--descent-m", f"{descent_m:.3f}"],
    )


def plan(args: argparse.Namespace) -> dict:
    script, descent_m = OBJECTS[args.object]
    phases = [
        "initialize both grippers and both arms",
        "verify right arm at Shanghai raised/inward/retracted parking posture",
        "restore lifting column to Shanghai 0.700 m home height",
        "use the initialized left-wrist view that covers the cup, bowl, and plate area",
        f"run {script} with fresh RGB alignment and {descent_m:.3f} m descent",
    ]
    if args.operation == "pick-place":
        phases.append(
            f"return object at the same XY: down {descent_m:.3f} m, open once, lift"
        )
    return {
        "motion_enabled": False,
        "object": args.object,
        "operation": args.operation,
        "phases": phases,
        "base_motion": False,
        "right_arm_used_for_grasp": False,
        "right_arm_parking_required": True,
        "initial_base_location": "stationary beside the pickup table",
        "table_height_profile": "Shanghai (assumed identical in Hamburg)",
        "calibration_origin": "Shanghai validated Stage 1",
    }


def run(args: argparse.Namespace) -> int:
    config = build_config(args)
    if not args.execute:
        print(json.dumps(plan(args), ensure_ascii=False, indent=2))
        return 0
    if not args.fresh_start_confirmed:
        raise MissionError(
            "--fresh-start-confirmed is required: place the selected object at the "
            "calibrated pickup table and leave both grippers empty"
        )

    required = [
        "grasp/scripts/initialize_dual_arm_pick_pose.py",
        "grasp/scripts/initialize_spine_height.py",
        OBJECTS[args.object][0],
    ]
    if args.operation == "pick-place":
        required.append("grasp/scripts/run_streamed_live_place_cycle.py")
    missing = [name for name in required if not Path(config.arm_root, name).is_file()]
    if missing:
        raise MissionError("missing arm-local test files: " + ", ".join(missing))

    run_id = uuid.uuid4().hex[:12]
    log_dir = args.log_dir / run_id
    state = {
        "version": 1,
        "run_id": run_id,
        "object": args.object,
        "operation": args.operation,
        "phase": "CREATED",
        "updated_unix_s": time.time(),
    }

    def set_phase(phase: str, **values) -> None:
        state.update(phase=phase, updated_unix_s=time.time(), **values)
        atomic_write_json(args.state_file, state)
        emit("phase", run_id=run_id, phase=phase)

    def execute(label: str, argv: list[str], timeout_s: float):
        return run_streamed_command(label, argv, timeout_s, log_dir / f"{label}.log")

    with MissionRunLock(args.lock_file):
        try:
            set_phase("INITIALIZING_DUAL_ARMS")
            result = execute("dual_init", initialization_argv(config), config.init_timeout_s)
            init_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not init_report_is_stable(init_report):
                raise MissionError(
                    "dual-arm initialization did not prove the Shanghai left pick-top "
                    "and right parking postures"
                )

            set_phase("INITIALIZING_SPINE", init_report=init_report)
            result = execute(
                "spine_init", spine_initialization_argv(config), config.init_timeout_s
            )
            spine_report = extract_last_json_object(result.output)
            if result.returncode != 0 or not spine_report_is_stable(spine_report):
                raise MissionError("lifting column did not prove the 0.700 m home target")

            set_phase("PICK_RUNNING", spine_report=spine_report)
            result = execute(
                f"{args.object}_pick",
                object_pick_argv(config, args.object),
                config.pick_timeout_s,
            )
            stable, pick_report = pick_report_is_stable(result.output, result.returncode)
            if not stable:
                raise MissionError(
                    f"{args.object} pick did not prove contact, lift, and stable hold"
                )
            set_phase("OBJECT_HELD", pick_report=pick_report)

            if args.operation == "pick-place":
                time.sleep(min(0.5, max(0.0, config.transition_settle_s)))
                descent_m = OBJECTS[args.object][1]
                result = execute(
                    f"{args.object}_same_place",
                    place_argv(config, args.object, run_id, descent_m),
                    config.place_timeout_s,
                )
                place_report = extract_last_json_object(result.output)
                if result.returncode != 0 or not place_report_is_stable(place_report):
                    raise MissionError(
                        f"{args.object} same-place release did not prove down-open-up"
                    )
                set_phase("COMPLETE_RELEASED", place_report=place_report)
                emit(
                    "complete",
                    run_id=run_id,
                    object=args.object,
                    object_held=False,
                    returned_to_same_xy=True,
                )
            else:
                set_phase("COMPLETE_HELD")
                emit(
                    "complete",
                    run_id=run_id,
                    object=args.object,
                    object_held=True,
                    returned_to_same_xy=False,
                )
            return 0
        except KeyboardInterrupt:
            set_phase("INTERRUPTED")
            emit("interrupted", run_id=run_id)
            return 130
        except Exception as exc:
            set_phase("FAILED", error=f"{type(exc).__name__}: {exc}")
            raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", choices=tuple(OBJECTS), required=True)
    parser.add_argument("--operation", choices=("pick", "pick-place"), default="pick")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start-confirmed", action="store_true")
    parser.add_argument("--arm-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--arm-env", default="/home/aup/tmr_env.sh")
    parser.add_argument("--init-timeout-s", type=float, default=180.0)
    parser.add_argument("--pick-timeout-s", type=float, default=300.0)
    parser.add_argument("--place-timeout-s", type=float, default=180.0)
    parser.add_argument("--transition-settle-s", type=float, default=0.5)
    parser.add_argument(
        "--state-file",
        type=Path,
        default=Path.home() / ".tmr_standalone_grasp_test" / "state.json",
    )
    parser.add_argument(
        "--lock-file",
        type=Path,
        default=Path("/tmp/tmr_task3_standalone_grasp.lock"),
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path.home() / ".tmr_standalone_grasp_test" / "logs",
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
