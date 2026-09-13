#!/usr/bin/env python3
"""Reset the robot beside the pickup table to the left-wrist observation pose.

The organizer places the mobile base beside the table before running this
entrypoint. It never creates a base command publisher or moves the base.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time
import traceback
from typing import Any

from hamburg_grasp_cycle import (
    DEFAULT_CYCLE_CONFIG,
    DEFAULT_VENUE_CONFIG,
    NativeArmGraspCycle,
    load_cycle_config,
    resolve_cycle_venue_overrides,
    write_report,
)
from hamburg_preflight import environment_report, load_config
from spine_control import SpineControl


def plan_report(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "plan",
        "operation": "Hamburg pickup-table observation reset",
        "manual_precondition": (
            "place the stopped mobile base beside the pickup table with clear arm workspace"
        ),
        "motion_commanded": False,
        "base_motion_commanded": False,
        "expected_left_wrist_frame": {
            "width": int(config["vision"]["width"]),
            "height": int(config["vision"]["height"]),
        },
        "success_check": (
            "review the saved post-reset frame and confirm the pickup table and "
            "approximately all three utensils are visible"
        ),
        "phases": [
            "validate native arm, gripper, spine, and 640x480 left-wrist interfaces",
            "move the spine to the configured pickup-view height",
            "park the right arm and move the left arm to the pickup-view posture",
            "open the left gripper",
            "capture a fresh left-wrist image for table and utensil placement review",
        ],
        "parameter_profile": config["profile_name"],
        "parameter_scope": config["parameter_scope"],
    }


def run_reset(
    runner: NativeArmGraspCycle,
    spine: SpineControl,
    config: dict[str, Any],
    output_dir: Path,
) -> dict[str, Any]:
    started = time.monotonic()
    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "running",
        "operation": "Hamburg pickup-table observation reset",
        "entrypoint": "pickup-reset",
        "object": "pickup-reset",
        "motion_commanded": False,
        "base_motion_commanded": False,
        "manual_base_placement_required": True,
        "parameter_profile": config["profile_name"],
        "applied_venue_overrides": config.get("applied_venue_overrides", {}),
        "phases_completed": [],
    }
    runner.last_report = report
    runner.set_phase(report, "wait_for_live_interfaces")
    runner.wait_live(10.0)
    report["controller_subscriber_counts"] = runner.assert_controller_graph()

    runner.set_phase(report, "spine_to_pickup_view_height")
    report["motion_commanded"] = True
    report["spine"] = spine.move_absolute(float(config["spine"]["initial_target_m"]))
    report["phases_completed"].append("spine_ready")

    runner.set_phase(report, "right_arm_to_parking")
    runner.move_to_joint_posture("right", config["posture"]["right_parking_rad"])
    report["phases_completed"].append("right_arm_parked")

    runner.set_phase(report, "left_arm_to_pickup_view")
    runner.move_to_joint_posture("left", config["posture"]["left_pick_top_rad"])
    report["phases_completed"].append("left_wrist_at_pickup_view")

    runner.set_phase(report, "open_left_gripper")
    report["left_gripper_open_feedback"] = runner.command_gripper(
        float(config["gripper"]["open"])
    )
    report["phases_completed"].append("left_gripper_open")

    runner.set_phase(report, "capture_fresh_left_wrist_view")
    frame = runner.fresh_wrist_bgr()
    report["left_wrist_frame"] = {
        "width": int(frame.shape[1]),
        "height": int(frame.shape[0]),
    }
    image_path = output_dir / "pickup-reset-left-wrist.png"
    try:
        import cv2

        image_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(image_path), frame):
            raise OSError(f"could not save left-wrist image to {image_path}")
        report["left_wrist_image"] = str(image_path)
        report["phases_completed"].append("fresh_left_wrist_image_saved")
    except Exception as image_exc:
        warning = f"left_wrist_image_write: {type(image_exc).__name__}: {image_exc}"
        report.setdefault("report_warnings", []).append(warning)
        print(warning, file=sys.stderr, flush=True)

    runner.spin_for(1.0)
    report["status"] = "passed"
    report["operator_next_step"] = (
        "review the saved post-reset frame; continue only if the pickup table and "
        "approximately all three utensils are visible"
    )
    report["duration_s"] = round(time.monotonic() - started, 3)
    runner.set_phase(report, "complete")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=Path(os.environ.get(
        "TMR_HAMBURG_GRASP_CYCLE_CONFIG", str(DEFAULT_CYCLE_CONFIG))))
    parser.add_argument("--venue-config", type=Path, default=DEFAULT_VENUE_CONFIG)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/tmr_hamburg_reset"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report_path = args.output or args.output_dir / "pickup-reset-report.json"
    try:
        config = load_cycle_config(args.config)
    except Exception as exc:
        write_report({
            "status": "invalid_configuration",
            "motion_commanded": False,
            "base_motion_commanded": False,
            "failed_phase": "load_and_validate_configuration",
            "errors": [f"{type(exc).__name__}: {exc}"],
            "traceback": traceback.format_exc(),
        }, report_path)
        return 2
    if not args.execute:
        write_report(plan_report(config), args.output)
        return 0

    try:
        venue = load_config(args.venue_config)
        config = resolve_cycle_venue_overrides(config, venue)
        environment, errors = environment_report(venue)
    except Exception as exc:
        write_report({
            "status": "invalid_environment",
            "motion_commanded": False,
            "base_motion_commanded": False,
            "failed_phase": "load_and_validate_venue",
            "errors": [f"{type(exc).__name__}: {exc}"],
            "traceback": traceback.format_exc(),
        }, report_path)
        return 2
    if errors:
        write_report({
            "status": "blocked",
            "motion_commanded": False,
            "base_motion_commanded": False,
            "failed_phase": "environment_preflight",
            "environment": environment,
            "errors": errors,
        }, report_path)
        return 2

    rclpy = None
    node = None
    rclpy_started = False
    runner: NativeArmGraspCycle | None = None
    spine: SpineControl | None = None
    report: dict[str, Any] = {
        "status": "starting", "motion_commanded": False, "base_motion_commanded": False
    }
    try:
        import rclpy as rclpy_module
        from rclpy.node import Node

        rclpy = rclpy_module
        rclpy.init(args=None)
        rclpy_started = True
        node = Node("tmr_task3_hamburg_pickup_reset")
        runner = NativeArmGraspCycle(node, venue, config, args.output_dir)
        spine = SpineControl(node, venue["interface_profile"]["spine"])
        report = run_reset(runner, spine, config, args.output_dir)
    except KeyboardInterrupt:
        report = dict(runner.last_report) if runner is not None else report
        report.setdefault("failed_phase", report.get("active_phase", "startup"))
        report.update({"status": "interrupted", "errors": ["operator interrupted pickup reset"]})
    except BaseException as exc:
        report = dict(runner.last_report) if runner is not None else report
        report.setdefault("failed_phase", report.get("active_phase", "startup"))
        report.update({
            "status": "failed",
            "errors": [f"{type(exc).__name__}: {exc}"],
            "traceback": traceback.format_exc(),
        })
    finally:
        cleanup_errors = []
        for label, snapshot in (
            ("arm_diagnostics", runner.diagnostic_snapshot if runner is not None else None),
            ("spine_diagnostics", spine.diagnostic_snapshot if spine is not None else None),
        ):
            if snapshot is None:
                continue
            try:
                report[label] = snapshot()
            except Exception as diagnostic_exc:
                report.setdefault("diagnostic_errors", []).append(
                    f"{label}: {type(diagnostic_exc).__name__}: {diagnostic_exc}"
                )
        for label, operation in (
            ("spine_close", spine.close if spine is not None else None),
            ("node_destroy", node.destroy_node if node is not None else None),
            ("rclpy_shutdown", rclpy.shutdown if rclpy is not None and rclpy_started else None),
        ):
            if operation is None:
                continue
            try:
                operation()
            except Exception as cleanup_exc:
                cleanup_errors.append(f"{label}: {type(cleanup_exc).__name__}: {cleanup_exc}")
        if cleanup_errors:
            report.setdefault("cleanup_errors", []).extend(cleanup_errors)
            if report.get("status") == "passed":
                report["status"] = "failed_cleanup"
    write_report(report, report_path)
    return 0 if report.get("status") == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
