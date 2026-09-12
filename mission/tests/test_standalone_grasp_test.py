from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "mission" / "scripts"
SCRIPT = SCRIPT_DIR / "run_standalone_grasp_test.py"


def load_module():
    import sys

    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location("standalone_grasp_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def args(object_name="plate", operation="pick"):
    return SimpleNamespace(
        object=object_name,
        operation=operation,
        execute=False,
        fresh_start_confirmed=False,
        arm_root=ROOT,
        arm_env="/home/aup/tmr_env.sh",
        init_timeout_s=180.0,
        pick_timeout_s=300.0,
        place_timeout_s=180.0,
        transition_settle_s=0.5,
        state_file=Path("/tmp/test-state.json"),
        lock_file=Path("/tmp/test-lock"),
        log_dir=Path("/tmp/test-logs"),
    )


def test_dry_run_is_motion_free_and_describes_right_parking():
    module = load_module()
    with mock.patch.object(module, "run_streamed_command") as execute, mock.patch(
        "sys.stdout", new_callable=io.StringIO
    ) as output:
        assert module.run(args()) == 0
    execute.assert_not_called()
    report = json.loads(output.getvalue())
    assert report["motion_enabled"] is False
    assert report["base_motion"] is False
    assert report["right_arm_parking_required"] is True
    assert report["initial_base_location"] == "stationary beside the pickup table"
    assert report["table_height_profile"] == "Shanghai (assumed identical in Hamburg)"


def test_each_object_reuses_the_validated_pick_component():
    module = load_module()
    config = module.build_config(args())
    expected = {
        "cup": "run_streamed_live_pick_cycle.py",
        "bowl": "run_streamed_food_bowl_pick_cycle.py",
        "plate": "run_streamed_plate_pick_cycle.py",
    }
    for object_name, script in expected.items():
        command = " ".join(module.object_pick_argv(config, object_name))
        assert script in command
        assert "--force-restore-top" in command
    plate = " ".join(module.object_pick_argv(config, "plate"))
    assert "--descent-m 0.375" in plate


def test_shanghai_initialization_guarantees_the_plate_clearance_posture():
    module = load_module()
    yaml = __import__("yaml")
    startup = yaml.safe_load(
        (ROOT / "grasp" / "config" / "grasp_initial_state.yaml").read_text(
            encoding="utf-8"
        )
    )
    full = __import__("run_full_competition_cycle")
    assert startup["initialization_order"] == ["right", "left"]
    assert startup["grippers"]["right"]["position"] == 0.8
    assert tuple(startup["right"]["positions"]) == full.RIGHT_PARKING_TARGET
    plan = module.plan(args("plate", "pick"))
    assert plan["right_arm_used_for_grasp"] is False
    assert plan["right_arm_parking_required"] is True


def test_validated_pick_component_has_complete_gripper_and_motion_order():
    source = (
        ROOT / "grasp" / "scripts" / "run_streamed_live_pick_cycle.py"
    ).read_text(encoding="utf-8")
    markers = [
        'phase = "OPENING"',
        'phase = "VISUAL_ALIGNING"',
        'phase = "DESCENDING"',
        'phase = "CLOSING"',
        'phase = "LIFTING"',
        'phase = "POST_LIFT_VERIFY"',
    ]
    positions = [source.index(marker, 25000) for marker in markers]
    assert positions == sorted(positions)
    assert 'command_gripper(gripper, 0.0, "open_before_motion")' in source
    assert 'command_gripper(node, 0.8, "close_after_fixed_descent")' in source
    assert source.index("move_vertical(arm, -DESCENT_M)", 25000) < source.index(
        "close_gripper_once(gripper)", 25000
    )


def stable_init_report(module):
    full = __import__("run_full_competition_cycle")
    reports = []
    for arm, target in (
        ("right", full.RIGHT_PARKING_TARGET),
        ("left", full.LEFT_PICK_TOP_TARGET),
    ):
        reports.append(
            {
                "arm": arm,
                "moved": True,
                "stable_hold": True,
                "target_joint_positions_rad": list(target),
                "measured_joint_positions_rad": list(target),
            }
        )
    return {
        "status": "success",
        "order": ["right", "left"],
        "reports": reports,
        "both_stable_hold": True,
        "gripper_order": ["right", "left"],
        "gripper_reports": [
            {
                "arm": "right",
                "target_position": 0.8,
                "measured_position": 0.8,
                "stable_reset": True,
            },
            {
                "arm": "left",
                "target_position": 0.0,
                "measured_position": 0.0,
                "stable_reset": True,
            },
        ],
        "both_grippers_reset": True,
        "gripper_commanded": True,
    }


def test_executable_coordinator_orders_init_spine_then_selected_pick():
    module = load_module()
    init_output = json.dumps(stable_init_report(module))
    spine_output = json.dumps(
        {
            "status": "success",
            "moved": True,
            "target_position_m": 0.7,
            "measured_position_m": 0.7,
        }
    )
    pick_output = "\n".join(
        [
            'PICK={"event":"success"}',
            'PICK={"event":"controller","joint_impedance":"restored_to_hold"}',
            'PICK={"event":"cycle_complete","final_state":"DONE","controller_hold":"stable"}',
        ]
    )

    result_type = __import__("run_long_range_pick").CommandResult
    for object_name in ("cup", "bowl", "plate"):
        labels = []

        def execute(label, _argv, _timeout, _log):
            labels.append(label)
            output = {
                "dual_init": init_output,
                "spine_init": spine_output,
                f"{object_name}_pick": pick_output,
            }[label]
            return result_type(0, output, 0.01)

        with tempfile.TemporaryDirectory() as directory:
            run_args = args(object_name, "pick")
            run_args.execute = True
            run_args.fresh_start_confirmed = True
            run_args.state_file = Path(directory) / "state.json"
            run_args.lock_file = Path(directory) / "motion.lock"
            run_args.log_dir = Path(directory) / "logs"
            with mock.patch.object(module, "run_streamed_command", side_effect=execute):
                assert module.run(run_args) == 0
        assert labels == ["dual_init", "spine_init", f"{object_name}_pick"]
