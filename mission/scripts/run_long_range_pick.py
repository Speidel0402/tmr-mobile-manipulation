#!/usr/bin/env python3
"""Run the verified base route and left-arm pick as one ordered mission.

The coordinator is intended to run on the robot computer (172.16.0.100).  It
starts the base-local route once over SSH, waits for its explicit success and
latched zero-speed stop, then starts the arm-local pick process.  The 20 Hz
base loop and all arm/camera loops therefore stay on their respective onboard
computers; SSH is only a phase boundary and log transport.

Without ``--execute`` this program only prints the strategy.  A failed arm
phase can be resumed with ``--resume-grasp`` without repeating the forward
drive or the clockwise turn, but only when the recorded arm events prove that
the arm recovered to a safe retry state.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from enum import Enum
import json
import os
from pathlib import Path
import queue
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from typing import Callable


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ARM_SCRIPT = REPO_ROOT / "grasp" / "scripts" / "run_streamed_live_pick_cycle.py"
ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


class Phase(str, Enum):
    CREATED = "CREATED"
    BASE_RUNNING = "BASE_RUNNING"
    BASE_LOCKED_AT_PICKUP = "BASE_LOCKED_AT_PICKUP"
    ARM_RUNNING = "ARM_RUNNING"
    COMPLETE = "COMPLETE"
    BASE_FAILED = "BASE_FAILED"
    ARM_FAILED = "ARM_FAILED"
    INTERRUPTED = "INTERRUPTED"


class MissionError(RuntimeError):
    pass


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    output: str
    elapsed_s: float


@dataclass(frozen=True)
class StrategyConfig:
    base_host: str
    base_root: str
    base_timeout_s: float
    arm_root: str
    arm_env: str
    arm_timeout_s: float
    transition_settle_s: float


class MissionRunLock:
    """Cross-process non-blocking lock held for the complete mission run."""

    def __init__(self, path: Path):
        self.path = path
        self.handle = None
        self._backend = ""

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("a+b")
        try:
            if os.name == "posix":
                import fcntl

                fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                self._backend = "fcntl"
            elif os.name == "nt":
                import msvcrt

                self.handle.seek(0, os.SEEK_END)
                if self.handle.tell() == 0:
                    self.handle.write(b"\0")
                    self.handle.flush()
                self.handle.seek(0)
                msvcrt.locking(self.handle.fileno(), msvcrt.LK_NBLCK, 1)
                self._backend = "msvcrt"
            else:
                raise OSError(f"unsupported lock platform: {os.name}")
        except OSError as exc:
            self.handle.close()
            self.handle = None
            raise MissionError(
                f"another long-range mission is already running (lock: {self.path})"
            ) from exc
        self.handle.seek(0)
        self.handle.truncate()
        self.handle.write(
            json.dumps({"pid": os.getpid(), "started_unix_s": time.time()}).encode("utf-8")
        )
        self.handle.flush()
        os.fsync(self.handle.fileno())
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        if self.handle is None:
            return
        try:
            if self._backend == "fcntl":
                import fcntl

                fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
            elif self._backend == "msvcrt":
                import msvcrt

                self.handle.seek(0)
                msvcrt.locking(self.handle.fileno(), msvcrt.LK_UNLCK, 1)
        finally:
            self.handle.close()
            self.handle = None


def emit(event: str, **values) -> None:
    print(
        "MISSION=" + json.dumps({"event": event, **values}, ensure_ascii=False, separators=(",", ":")),
        flush=True,
    )


def atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    os.replace(temporary, path)


def load_checkpoint(path: Path) -> dict | None:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict) or "phase" not in value:
        raise MissionError(f"invalid mission checkpoint: {path}")
    return value


def checkpoint(path: Path, run_id: str, phase: Phase, **values) -> dict:
    payload = {
        "version": 1,
        "run_id": run_id,
        "phase": phase.value,
        "updated_unix_s": time.time(),
        **values,
    }
    atomic_write_json(path, payload)
    emit("phase", run_id=run_id, phase=phase.value)
    return payload


def _base_environment() -> str:
    return "\n".join(
        [
            "source /opt/ros/humble/setup.bash",
            "source \"${HOME}/ros2_ws/install/setup.bash\"",
            "source \"${HOME}/tmr_navigation/install/setup.bash\"",
            "source \"${HOME}/tmr_navigation/install/tmr_local_navigation/share/tmr_local_navigation/local_setup.bash\"",
            # Keep every base-local mission phase in the same isolated Humble
            # graph as 03_start_navigation.sh.  The Jazzy arm host is
            # orchestrated over SSH, not by mixing both ROS graphs.
            "export ROS_DOMAIN_ID=${TMR_CYCLE_ROS_DOMAIN_ID:-97}",
            "export ROS_LOCALHOST_ONLY=${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}",
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
            "export PYTHONUNBUFFERED=1",
            # Debug/resume overrides were useful during manual experiments,
            # but must never leak into the integrated competition route.
            "unset TMR_CYCLE_DISABLE_COLLISION_GUARD TMR_CYCLE_SKIP_INITIAL_FORWARD TMR_CYCLE_SKIP_TURN TMR_CYCLE_RESUME_YAW_CORRECTION_DEG",
            "if [[ -f \"${HOME}/cyclonedds.xml\" ]]; then export CYCLONEDDS_URI=\"file://${HOME}/cyclonedds.xml\"; else unset CYCLONEDDS_URI || true; fi",
        ]
    )


def build_remote_base_shell(config: StrategyConfig, run_id: str) -> str:
    root = shlex.quote(config.base_root)
    pid_file = shlex.quote(f"/tmp/tmr_long_range_base_{run_id}.pid")
    mission = (
        f"python3 {root}/scripts/07_start_to_pickup.py "
        f"--config {root}/config/start_to_pickup.yaml --execute --disable-collision-guard"
    )
    return "\n".join(
        [
            # ROS setup files legitimately probe optional unset variables.
            # Enable nounset only after all overlays have been sourced.
            "set -eo pipefail",
            _base_environment(),
            "set -u",
            f"cd {root}",
            # flock is released by the kernel even after an SSH/process crash,
            # so a stale PID file can never permanently block the next run.
            "command -v flock >/dev/null 2>&1 || { echo 'base lock utility unavailable' >&2; exit 72; }",
            "exec 9>/tmp/tmr_long_range_base.lock",
            "flock -n 9 || { echo 'another base mission is already running' >&2; exit 73; }",
            "child=''",
            f"pid_file={pid_file}",
            "stop_child() {",
            "  if [[ -n \"${child}\" ]] && kill -0 \"${child}\" 2>/dev/null; then",
            "    kill -INT \"${child}\" 2>/dev/null || true",
            "    for _ in {1..30}; do kill -0 \"${child}\" 2>/dev/null || return 0; sleep 0.1; done",
            "    kill -TERM \"${child}\" 2>/dev/null || true",
            "  fi",
            "}",
            "trap 'stop_child' HUP INT TERM",
            f"{mission} &",
            "child=$!",
            "printf '%s\\n' \"${child}\" >\"${pid_file}\"",
            "set +e",
            "wait \"${child}\"",
            "rc=$?",
            "set -e",
            "rm -f \"${pid_file}\"",
            "trap - HUP INT TERM",
            "exit \"${rc}\"",
        ]
    )


def build_base_argv(config: StrategyConfig, run_id: str) -> list[str]:
    remote = build_remote_base_shell(config, run_id)
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
        "bash -lc " + shlex.quote(remote),
    ]


def build_arm_argv(config: StrategyConfig) -> list[str]:
    root = shlex.quote(config.arm_root)
    env_file = shlex.quote(config.arm_env)
    script = shlex.quote(str(Path(config.arm_root) / "grasp" / "scripts" / "run_streamed_live_pick_cycle.py"))
    command = f"export PYTHONUNBUFFERED=1 && source {env_file} && cd {root} && exec python3 {script}"
    return ["bash", "-lc", command]


def _terminate(process: subprocess.Popen, grace_s: float = 5.0) -> None:
    if process.poll() is not None:
        return
    if os.name == "posix":
        os.killpg(process.pid, signal.SIGINT)
    else:
        process.terminate()
    try:
        process.wait(timeout=grace_s)
        return
    except subprocess.TimeoutExpired:
        pass
    if os.name == "posix":
        os.killpg(process.pid, signal.SIGTERM)
    else:
        process.terminate()
    try:
        process.wait(timeout=3.0)
        return
    except subprocess.TimeoutExpired:
        pass
    if os.name == "posix":
        os.killpg(process.pid, signal.SIGKILL)
    else:
        process.kill()
    process.wait(timeout=2.0)


def run_streamed_command(
    label: str,
    argv: list[str],
    timeout_s: float,
    log_path: Path,
    on_line: Callable[[str], None] | None = None,
) -> CommandResult:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    process = subprocess.Popen(
        argv,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
        start_new_session=True,
    )
    lines: queue.Queue[str | None] = queue.Queue()

    def read_output() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=read_output, daemon=True).start()
    output: list[str] = []
    reader_done = False
    try:
        with log_path.open("w", encoding="utf-8") as log:
            while not reader_done or process.poll() is None:
                if time.monotonic() - started > float(timeout_s):
                    _terminate(process)
                    raise TimeoutError(f"{label} exceeded {timeout_s:.1f}s")
                try:
                    item = lines.get(timeout=0.10)
                except queue.Empty:
                    continue
                if item is None:
                    reader_done = True
                    continue
                output.append(item)
                log.write(item)
                log.flush()
                print(f"[{label}] {item}", end="", flush=True)
                if on_line is not None:
                    on_line(item.rstrip("\r\n"))
            returncode = process.wait(timeout=1.0)
    except KeyboardInterrupt:
        _terminate(process)
        raise
    return CommandResult(returncode, "".join(output), time.monotonic() - started)


def extract_last_json_object(text: str) -> dict | None:
    clean = ANSI_ESCAPE.sub("", text)
    decoder = json.JSONDecoder()
    best = None
    best_span = -1
    mission_report = None
    for index, char in enumerate(clean):
        if char != "{":
            continue
        try:
            value, end = decoder.raw_decode(clean[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            if end > best_span:
                best, best_span = value, end
            if "status" in value and "final_state" in value:
                mission_report = value
    return mission_report if mission_report is not None else best


def parse_pick_events(text: str) -> list[dict]:
    events = []
    for line in ANSI_ESCAPE.sub("", text).splitlines():
        marker = line.find("PICK=")
        if marker < 0:
            continue
        try:
            value = json.loads(line[marker + len("PICK=") :])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            events.append(value)
    return events


def base_report_is_stable(report: dict | None) -> bool:
    if not isinstance(report, dict):
        return False
    stationary = report.get("final_stationary")
    return bool(
        report.get("status") == "success"
        and report.get("final_state") == "FINAL_STOP"
        and isinstance(stationary, dict)
        and stationary.get("confirmed") is True
        and report.get("zero_command_latched") is True
        and report.get("control_lease_held") is True
    )


def arm_resume_assessment(events: list[dict]) -> dict:
    """Return a conservative proof that re-opening can only happen at a safe state."""
    if any(event.get("event") == "operator_stop" for event in events):
        return {"safe": False, "reason": "operator stop requires physical state review"}
    if any(event.get("event") == "controller_restore_failed" for event in events):
        return {"safe": False, "reason": "stable hold was not restored"}
    failures = [event for event in events if event.get("event") == "failure"]
    if not failures:
        return {"safe": False, "reason": "no structured arm failure phase was recorded"}
    phase = str(failures[-1].get("phase", ""))
    controller_was_stable = any(
        event.get("event") == "controller"
        and event.get("joint_impedance") == "restored_to_hold"
        for event in events
    )
    no_arm_motion_phases = {"INIT", "CAMERA_PREFLIGHT", "OPENING", "OPEN_VERIFIED"}
    top_motion_phases = {"RESTORING_REFERENCE_POSE", "VISUAL_ALIGNING", "ALIGNED"}
    if phase in no_arm_motion_phases:
        return {"safe": True, "reason": "failure occurred before arm motion", "phase": phase}
    if phase in top_motion_phases and controller_was_stable:
        return {"safe": True, "reason": "arm is held above the grasp plane", "phase": phase}
    if phase in {"DESCENDING", "AT_LOW"}:
        recovered = any(
            event.get("event") == "internal_fault_recovery_complete"
            and event.get("failed_phase") == phase
            for event in events
        )
        if recovered and controller_was_stable:
            return {"safe": True, "reason": "open gripper recovered to top hold", "phase": phase}
    return {
        "safe": False,
        "reason": "gripper/object or arm pose is not proven safe for automatic reopening",
        "phase": phase,
    }


def validate_start(checkpoint_value: dict | None, resume_grasp: bool, fresh_start: bool) -> None:
    if checkpoint_value is None:
        if resume_grasp:
            raise MissionError("--resume-grasp requires a BASE_LOCKED_AT_PICKUP/ARM_FAILED checkpoint")
        return
    phase = str(checkpoint_value.get("phase", ""))
    if resume_grasp:
        if phase == Phase.BASE_LOCKED_AT_PICKUP.value:
            return
        # A nested SSH teardown may return non-zero after the base has already
        # emitted a complete, independently verifiable stationary-stop proof.
        # Resume only the arm in that exact case; never repeat base motion.
        if phase == Phase.BASE_FAILED.value and base_report_is_stable(
            checkpoint_value.get("report")
        ):
            return
        if phase == Phase.ARM_FAILED.value and checkpoint_value.get("resume_safe") is True:
            return
        if phase == Phase.ARM_FAILED.value:
            raise MissionError(
                "arm failure is not proven safe for automatic reopening; restore/inspect the arm first"
            )
        if phase not in {Phase.BASE_LOCKED_AT_PICKUP.value, Phase.ARM_FAILED.value}:
            raise MissionError(f"cannot resume grasp from checkpoint phase {phase}")
    if not fresh_start:
        raise MissionError(
            f"checkpoint phase is {phase}; use --resume-grasp after an arm failure, or "
            "--fresh-start-confirmed only after physically returning to the marked start"
        )


def strategy_summary(config: StrategyConfig, checkpoint_path: Path) -> dict:
    return {
        "motion_enabled": False,
        "coordinator_host": "robot computer (arm-local)",
        "phases": [
            "base: forward 0.85 m",
            "base: clockwise 90 deg",
            "base: detect and align doorway midpoint",
            "base: stop 0.50 m before door, then advance 1.20 m",
            "base: latch zero-speed lease",
            "left arm: open -> restore pose -> visual align -> descend -> close -> lift",
            "left arm: stable impedance hold",
        ],
        "base_host": config.base_host,
        "base_control": "base-local persistent ROS process",
        "arm_control": "arm-local persistent ROS process",
        "checkpoint": str(checkpoint_path),
        "resume_rule": "--resume-grasp never repeats base motion",
    }


def run(args: argparse.Namespace) -> int:
    config = StrategyConfig(
        base_host=args.base_host,
        base_root=args.base_root,
        base_timeout_s=args.base_timeout_s,
        arm_root=str(args.arm_root.resolve()),
        arm_env=args.arm_env,
        arm_timeout_s=args.arm_timeout_s,
        transition_settle_s=args.transition_settle_s,
    )
    if not args.execute:
        print(json.dumps(strategy_summary(config, args.checkpoint), ensure_ascii=False, indent=2))
        return 0
    if not Path(config.arm_root, "grasp", "scripts", "run_streamed_live_pick_cycle.py").is_file():
        raise MissionError("arm pick script is missing from --arm-root")

    lock_path = args.checkpoint.with_name(args.checkpoint.name + ".lock")
    with MissionRunLock(lock_path):
        return run_locked(args, config)


def run_locked(args: argparse.Namespace, config: StrategyConfig) -> int:

    previous = load_checkpoint(args.checkpoint)
    validate_start(previous, args.resume_grasp, args.fresh_start_confirmed)
    if args.resume_grasp:
        run_id = str(previous["run_id"])
    else:
        run_id = uuid.uuid4().hex[:12]
        checkpoint(args.checkpoint, run_id, Phase.CREATED)
    log_dir = args.log_dir / run_id

    try:
        if not args.resume_grasp:
            checkpoint(args.checkpoint, run_id, Phase.BASE_RUNNING)
            try:
                base_result = run_streamed_command(
                    "base",
                    build_base_argv(config, run_id),
                    config.base_timeout_s,
                    log_dir / "base.log",
                )
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                checkpoint(
                    args.checkpoint,
                    run_id,
                    Phase.BASE_FAILED,
                    error=f"{type(exc).__name__}: {exc}",
                )
                raise
            base_report = extract_last_json_object(base_result.output)
            if not base_report_is_stable(base_report):
                checkpoint(
                    args.checkpoint,
                    run_id,
                    Phase.BASE_FAILED,
                    returncode=base_result.returncode,
                    report=base_report,
                )
                raise MissionError("base route did not reach a confirmed FINAL_STOP")
            if base_result.returncode != 0:
                emit(
                    "base_transport_status_ignored",
                    run_id=run_id,
                    returncode=base_result.returncode,
                    reason="structured FINAL_STOP proof is complete",
                )
            checkpoint(
                args.checkpoint,
                run_id,
                Phase.BASE_LOCKED_AT_PICKUP,
                base_elapsed_s=base_result.elapsed_s,
                base_report=base_report,
            )
            # The base report already contains a bounded stationary proof.
            # This small capped delay only lets the SSH child close cleanly.
            time.sleep(min(1.0, max(0.0, config.transition_settle_s)))

        checkpoint(args.checkpoint, run_id, Phase.ARM_RUNNING)
        try:
            arm_result = run_streamed_command(
                "arm",
                build_arm_argv(config),
                config.arm_timeout_s,
                log_dir / "arm.log",
            )
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            checkpoint(
                args.checkpoint,
                run_id,
                Phase.ARM_FAILED,
                error=f"{type(exc).__name__}: {exc}",
                resume_safe=False,
                resume_reason="arm process outcome/phase is unknown",
            )
            raise
        pick_events = parse_pick_events(arm_result.output)
        complete = [event for event in pick_events if event.get("event") == "cycle_complete"]
        failures = [
            event
            for event in pick_events
            if event.get("event") in {"failure", "controller_restore_failed"}
        ]
        if arm_result.returncode != 0 or not complete or failures:
            resume = arm_resume_assessment(pick_events)
            checkpoint(
                args.checkpoint,
                run_id,
                Phase.ARM_FAILED,
                returncode=arm_result.returncode,
                last_pick_event=pick_events[-1] if pick_events else None,
                arm_events_tail=pick_events[-16:],
                resume_safe=bool(resume["safe"]),
                resume_reason=resume["reason"],
                failed_phase=resume.get("phase"),
            )
            raise MissionError("arm pick did not reach cycle_complete with stable hold")
        checkpoint(
            args.checkpoint,
            run_id,
            Phase.COMPLETE,
            arm_elapsed_s=arm_result.elapsed_s,
            final_pick_event=complete[-1],
        )
        emit("complete", run_id=run_id, base_lease="latched_zero", grasp="confirmed")
        return 0
    except KeyboardInterrupt:
        checkpoint(args.checkpoint, run_id, Phase.INTERRUPTED)
        emit("interrupted", run_id=run_id, action="children_interrupted_and_base_zero_retained")
        return 130


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="enable base and arm execution")
    parser.add_argument("--resume-grasp", action="store_true", help="skip all base motion using a valid checkpoint")
    parser.add_argument(
        "--fresh-start-confirmed",
        action="store_true",
        help="start a new route despite an old checkpoint; robot must be back at the marked start",
    )
    parser.add_argument("--base-host", default="tmr-user@172.16.0.50")
    parser.add_argument("--base-root", default="/home/tmr-user/tmr_cycle")
    parser.add_argument("--base-timeout-s", type=float, default=180.0)
    parser.add_argument("--arm-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--arm-env", default="/home/aup/tmr_env.sh")
    parser.add_argument("--arm-timeout-s", type=float, default=180.0)
    parser.add_argument("--transition-settle-s", type=float, default=0.8)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path.home() / ".tmr_long_range_pick" / "state.json",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path.home() / ".tmr_long_range_pick" / "logs",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        return run(args)
    except Exception as exc:
        emit("aborted", error=f"{type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
