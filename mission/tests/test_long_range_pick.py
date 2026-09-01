#!/usr/bin/env python3
"""Offline contracts for the integrated long-range mission coordinator."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "mission" / "scripts" / "run_long_range_pick.py"
SPEC = importlib.util.spec_from_file_location("long_range_pick", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
mission = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mission
SPEC.loader.exec_module(mission)


def config():
    return mission.StrategyConfig(
        base_host="tmr-user@172.16.0.50",
        base_root="/home/tmr-user/tmr_cycle",
        base_timeout_s=180.0,
        arm_root="/home/aup/tmr-mobile-manipulation",
        arm_env="/home/aup/tmr_env.sh",
        arm_timeout_s=180.0,
        transition_settle_s=0.8,
    )


class LongRangePickContracts(unittest.TestCase):
    def test_base_stage_uses_verified_single_process_route(self) -> None:
        shell = mission.build_remote_base_shell(config(), "abc123")
        self.assertIn("07_start_to_pickup.py", shell)
        self.assertIn("start_to_pickup.yaml --execute", shell)
        self.assertIn("--disable-collision-guard", shell)
        self.assertIn("ROS_DOMAIN_ID=97", shell)
        self.assertIn("trap 'stop_child' HUP INT TERM", shell)
        self.assertIn("flock -n 9", shell)
        self.assertIn("unset TMR_CYCLE_DISABLE_COLLISION_GUARD", shell)
        self.assertLess(shell.index("source /opt/ros/humble/setup.bash"), shell.index("set -u"))

    def test_arm_stage_is_robot_local_and_ordered_script(self) -> None:
        argv = mission.build_arm_argv(config())
        joined = " ".join(argv)
        self.assertNotIn("ssh", argv[0])
        self.assertIn("source /home/aup/tmr_env.sh", joined)
        self.assertIn("run_streamed_live_pick_cycle.py", joined)

    def test_nested_base_report_returns_outer_mission_object(self) -> None:
        text = 'log\n{"status":"success","final_state":"FINAL_STOP","nested":{"x":1}}\n'
        report = mission.extract_last_json_object(text)
        self.assertEqual(report["status"], "success")
        self.assertEqual(report["final_state"], "FINAL_STOP")

    def test_arm_requires_final_stable_cycle_complete_event(self) -> None:
        events = mission.parse_pick_events(
            'PICK={"event":"success"}\n'
            'PICK={"event":"controller","joint_impedance":"restored_to_hold"}\n'
            'PICK={"event":"cycle_complete","controller_hold":"stable"}\n'
        )
        self.assertEqual(events[-1]["event"], "cycle_complete")

    def test_resume_grasp_never_accepts_base_running_checkpoint(self) -> None:
        with self.assertRaisesRegex(mission.MissionError, "cannot resume"):
            mission.validate_start(
                {"run_id": "x", "phase": mission.Phase.BASE_RUNNING.value},
                resume_grasp=True,
                fresh_start=False,
            )

    def test_resume_grasp_requires_proven_safe_arm_failure(self) -> None:
        mission.validate_start(
            {"run_id": "x", "phase": mission.Phase.BASE_LOCKED_AT_PICKUP.value},
            resume_grasp=True,
            fresh_start=False,
        )
        mission.validate_start(
            {
                "run_id": "x",
                "phase": mission.Phase.ARM_FAILED.value,
                "resume_safe": True,
            },
            resume_grasp=True,
            fresh_start=False,
        )
        with self.assertRaisesRegex(mission.MissionError, "not proven safe"):
            mission.validate_start(
                {"run_id": "x", "phase": mission.Phase.ARM_FAILED.value},
                resume_grasp=True,
                fresh_start=False,
            )

    def test_base_handoff_requires_stationary_zero_lease_proof(self) -> None:
        report = {
            "status": "success",
            "final_state": "FINAL_STOP",
            "final_stationary": {"confirmed": True},
            "zero_command_latched": True,
            "control_lease_held": True,
        }
        self.assertTrue(mission.base_report_is_stable(report))
        report["zero_command_latched"] = False
        self.assertFalse(mission.base_report_is_stable(report))

    def test_resume_grasp_accepts_transport_failure_with_stable_base_proof(self) -> None:
        checkpoint = {
            "phase": mission.Phase.BASE_FAILED.value,
            "report": {
                "status": "success",
                "final_state": "FINAL_STOP",
                "final_stationary": {"confirmed": True},
                "zero_command_latched": True,
                "control_lease_held": True,
            },
        }
        mission.validate_start(checkpoint, resume_grasp=True, fresh_start=False)

    def test_arm_resume_never_reopens_after_close_or_operator_stop(self) -> None:
        closed = mission.arm_resume_assessment(
            [
                {"event": "failure", "phase": "GRASP_VERIFIED"},
                {"event": "internal_fault_recovery_complete", "failed_phase": "GRASP_VERIFIED"},
                {"event": "controller", "joint_impedance": "restored_to_hold"},
            ]
        )
        self.assertFalse(closed["safe"])
        stopped = mission.arm_resume_assessment(
            [{"event": "operator_stop", "stopped_at_phase": "AT_LOW"}]
        )
        self.assertFalse(stopped["safe"])

    def test_open_descent_failure_can_resume_only_after_top_recovery(self) -> None:
        events = [
            {"event": "failure", "phase": "AT_LOW"},
            {"event": "internal_fault_recovery_complete", "failed_phase": "AT_LOW"},
            {"event": "controller", "joint_impedance": "restored_to_hold"},
        ]
        self.assertTrue(mission.arm_resume_assessment(events)["safe"])
        self.assertFalse(mission.arm_resume_assessment(events[:1])["safe"])

    def test_only_one_coordinator_can_hold_the_run_lock(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            lock_path = Path(directory) / "mission.lock"
            with mission.MissionRunLock(lock_path):
                with self.assertRaisesRegex(mission.MissionError, "already running"):
                    with mission.MissionRunLock(lock_path):
                        pass

    def test_new_route_with_old_checkpoint_requires_physical_reset_flag(self) -> None:
        with self.assertRaisesRegex(mission.MissionError, "marked start"):
            mission.validate_start(
                {"run_id": "x", "phase": mission.Phase.COMPLETE.value},
                resume_grasp=False,
                fresh_start=False,
            )
        mission.validate_start(
            {"run_id": "x", "phase": mission.Phase.COMPLETE.value},
            resume_grasp=False,
            fresh_start=True,
        )

    def test_summary_preserves_confirmed_physical_order(self) -> None:
        phases = mission.strategy_summary(config(), Path("state.json"))["phases"]
        joined = " -> ".join(phases)
        self.assertLess(joined.index("forward 0.85"), joined.index("clockwise 90"))
        self.assertLess(joined.index("doorway midpoint"), joined.index("open -> restore pose"))


if __name__ == "__main__":
    unittest.main()
