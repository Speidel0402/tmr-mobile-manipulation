# EBiM Task 3 Phase 2 — Stage 1

[![Offline tests](https://github.com/Speidel0402/tmr-mobile-manipulation/actions/workflows/offline-tests.yml/badge.svg)](https://github.com/Speidel0402/tmr-mobile-manipulation/actions/workflows/offline-tests.yml)

Submission package for the completed Stage 1 portion of EBiM Task 3 Phase 2, implemented on a TMR mobile manipulation platform with dual Franka FR3 arms, Robotiq grippers, a lifting column, dual LiDAR, wrist-mounted RealSense D405 cameras, and a head-mounted ZED camera.

The repository provides task orchestration, robot integration, perception components, and operator tools. This README covers setup, execution, verification, and recovery.

## Stage 1 task definition

Letter targets are presented as white rectangular cards containing one uppercase letter with clear local intensity contrast. The letter does not have to be black: grey or coloured lettering is supported when it remains visibly darker than the card surface. The head-mounted ZED RGB pipeline detects the card, rectifies its perspective, normalizes the glyph, matches it within the configured alphabet, and requires consistent observations across fresh frames. It does not require a depth image or an online recognition service.

The destination mapping is configured through `--cup-letter`, `--bowl-letter`, and `--plate-letter` in `mission/scripts/run_three_object_delivery.py`; the complete-start wrapper is `mission/scripts/run_complete_from_start.sh`. Shared letter-search defaults are stored in `mission/config/letter_delivery.json`. Each mapping value must be a distinct single uppercase letter. The submitted label assignment is `A` for the food bowl, `B` for the cup, and `D` for the plate; the venue recognition set includes `A` through `E` so non-target cards are not forced into one of the selected classes.

The execution order is fixed: cup, food bowl, then plate. After delivering the cup and food bowl, the robot returns to the pickup table for the next object. Plate delivery completes the Stage 1 run, after which the base is stopped and operator control is restored.

### Complete test video

[Watch the complete Stage 1 test recorded with event officials present](https://github.com/Speidel0402/tmr-mobile-manipulation/releases/download/stage1-pre-submission/ebim-task3-phase2-stage1-official-test.mp4).

## Hamburg venue package

The evaluator-facing Hamburg deployment package is isolated under
[`deploy/hamburg/`](deploy/hamburg/README.md).  It targets the documented
arm64 Ubuntu 22.04 / ROS 2 Humble companion environment and preserves the
validated Stage 1 policy.  Gripper, lifting-column, DDS, and camera differences
are handled through a venue profile and read-only preflight rather than by
changing the task strategy.

## System requirements

| Component | Environment | Role |
| --- | --- | --- |
| Base computer | ROS 2 Humble | Mobile base, LiDAR, and head-camera services |
| Arm computer | ROS 2 Jazzy and the configured robot environment | Arms, grippers, lifting column, wrist cameras, and task coordinator |
| Operator computer | Windows PowerShell and OpenSSH | Service startup, task launch, and observation |

The robot must already have its vendor drivers, SDKs, ROS workspaces, calibration, and device configuration installed. Installing the offline Python dependencies alone does not provision a robot.

The demonstrated deployment uses the following endpoints. They are defaults,
not files that must already exist inside the evaluator container:

- Arm computer: `aup@172.16.0.100`.
- Base computer: `tmr-user@172.16.0.50`.
- Evaluator-provided key-based SSH access to both computers.

Keep the configured control and camera ROS domains separate. Run real-time control on the corresponding robot computer, not through an operator-side command loop. Credentials and machine-specific overrides are not included in the repository.

## Running the submission

### Docker build and launch

This portability update was limited to issues identified from earlier-task
deployment failures and the known differences between the Shanghai and German
testbeds. Only the necessary packaging, environment discovery, SSH, and
diagnostic behavior was changed. The validated task strategy, calibration,
motion sequence, perception decisions, and object-to-letter mapping were not
changed.

The submitted container is the mission coordinator. It does not install or source ROS itself. At launch it stages the exact policy files embedded in the image into an isolated temporary directory on both robot computers. Base commands then run only in the base computer's ROS 2 Humble environment, while arm, gripper, lifting-column, and wrist-camera commands run only in the arm computer's ROS 2 Jazzy environment. The two ROS installations are never overlaid or sourced in the same process.

The testbed's vendor drivers, ROS services, calibrated configuration, and passwordless SSH connectivity must already be available as described above. The container requires access to the testbed LAN at run time; it does not require Internet access or a GPU. Staging does not modify the vendor workspaces or their installed ROS packages. The image stages its own policy checkout and a non-secret ROS environment loader, so `/home/aup/tmr-mobile-manipulation` and `/home/aup/tmr_env.sh` are not required by the container entrypoint.

From a clean checkout, build the pinned submission as follows:

```bash
docker build -t edl-team-task3-phase2 .
```

Validate the image without contacting either robot computer:

```bash
docker run --rm edl-team-task3-phase2 preflight
```

With the robot at the designated start and all required services healthy, launch the submitted policy with the operator's read-only SSH configuration mounted into the container:

```bash
docker run --rm --network host \
  --mount type=bind,src="$HOME/.ssh",dst=/root/.ssh,readonly \
  --tmpfs /root/.tmr_three_object_delivery \
  edl-team-task3-phase2 check
```

`check` stages the submitted files and verifies SSH, the native ROS
environments, required services/actions, fresh wrist-camera data, base topics,
and fresh odometry. It does not intentionally command physical motion. Only
after it succeeds, run the policy:

```bash
docker run --rm --network host \
  --mount type=bind,src="$HOME/.ssh",dst=/root/.ssh,readonly \
  --tmpfs /root/.tmr_three_object_delivery \
  edl-team-task3-phase2 execute
```

SSH credentials are evaluator-provided deployment credentials and are never
stored in this repository or image. The container accepts the first host keys
inside its disposable runtime and does not require a pre-populated
`known_hosts`. Override `EBIM_ARM_HOST`, `EBIM_BASE_HOST`, or `EBIM_ARM_ENV`
when the evaluation deployment differs from the demonstrated hosts. Setting
`EBIM_ARM_ENV` remains supported, but its default is now the staged portable
loader, which imports only the ROS runtime variables from an already-running
robot process and never copies or prints the complete process environment.

The image entrypoint is `docker/run_task3.sh`, which stages the pinned policy and then invokes `mission/scripts/run_complete_from_start.sh`. Optional letter overrides may be appended to the `docker run` command. Do not launch a second coordinator concurrently.

Detailed evaluator diagnostics and recovery steps are in
[`docs/EVALUATOR_TROUBLESHOOTING.md`](docs/EVALUATOR_TROUBLESHOOTING.md).

### A. Robot services are already running

Use this entry when the robot is at the designated start, the grippers are empty, and the FR3, Robotiq, lifting-column, D405, and ZED services are healthy. The calibrated scene and device configuration must be in place, and no other autonomous task should be running.

On the arm computer:

```bash
cd /home/aup/tmr-mobile-manipulation
bash mission/scripts/run_complete_from_start.sh \
  --cup-letter B --bowl-letter A --plate-letter D
```

The destination letters are command-line parameters. The entry explicitly starts a new complete task and rejects intermediate-resume options. It manages task control ownership, checks the base runtime, and attempts to restore gamepad control on completion, failure, or interruption.

This is the task launcher, **not** a hardware power-on or full driver installer. Do not launch a second copy or rerun it from an arbitrary intermediate position.

### B. Cold start or services have been stopped

First power on the platform, complete the required vendor/FCI activation, and resolve any physical emergency-stop condition. From the repository root on the Windows operator computer, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\grasp\scripts\start_tmr_system.ps1
```

This helper starts the configured services and performs initialization, which can move the arms, grippers, and lifting column. Wait for service readiness and initialization to finish, then use the task command in section A on the arm computer.

Do not add `-EnableTeleop` for autonomous task startup. Do not repeatedly run the startup helper over an active task. It does not install vendor software, power on hardware, or clear a physical emergency stop.

### Stopping and restarting

- Use `Ctrl+C` in the task terminal for a controlled software interruption. Keep the hardware emergency stop accessible.
- Wait for the task to exit before restarting services or handing control to another program.
- Use the complete-start entry again only after returning the robot to the designated start with empty grippers.
- For an intermediate restart, confirm the physical state and use an explicit recovery entry. Never infer the current state solely from an old checkpoint or camera image.

## Intermediate recovery

Run these commands on the arm computer from the repository root, only after confirming that the robot is at the configured pickup location with no object held:

```bash
# Restart object handling from the pickup location.
python3 mission/scripts/run_three_object_delivery.py --execute \
  --resume-at-pickup-confirmed \
  --cup-letter B --bowl-letter A --plate-letter D

# Resume with a specific remaining object: bowl or plate.
python3 mission/scripts/run_three_object_delivery.py --execute \
  --resume-object-at-pickup-confirmed bowl \
  --cup-letter B --bowl-letter A --plate-letter D
```

Use `--resume-after-cup-held-confirmed` only when the cup is physically secured and the arm is in the configured raised holding pose. If the robot position, held object, or completed stage is uncertain, reset to the designated start instead of guessing a resume mode.

## Troubleshooting

Stop the active task before applying service-recovery commands. Do not bypass failures by widening motion limits or perception thresholds.

| Symptom | Recommended action |
| --- | --- |
| Arm activation fails or joint state stops updating | Resolve the FCI/hardware fault, restart the arm controller process if needed, and confirm both arm controllers are active. |
| Base reports `No configuration`, `no odometry progress`, or does not respond | Restore the managed base runtime and verify fresh odometry. Avoid duplicate drivers or velocity publishers. |
| Camera page is visible but the image is stale | Check the capture service's frame counter and health status. Restore the affected camera/snapshot service before resuming. |
| Object or label detection is inconsistent | Verify the camera identity, fresh RGB frames, lighting, and calibrated setup. Keep the validated dependency versions; do not reuse an old detection. |
| FK/IK or Cartesian services are missing | Restore the geometry service through the startup helper, using the arm computer's configured ROS environment. |
| A motion times out or initialization is visibly incorrect | Inspect the latest task log and controller state. Do not repeatedly send isolated motion or gripper commands. |
| Gamepad control is unavailable after exit | Confirm the autonomous task has exited, then restore teleoperation on the base computer. |

### Arm controller recovery

After resolving the underlying FCI fault and stopping the task, restart the managed arm process on the arm computer if it has failed:

```bash
screen -S tmr_fr3_arms -X quit
screen -L -Logfile /tmp/tmr_fr3_arms.log -dmS tmr_fr3_arms bash -lc \
  'source /home/aup/tmr_env.sh; exec ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=tmr_duo_config.yaml'
```

Check both controllers before resuming:

```bash
source /home/aup/tmr_env.sh
ros2 control list_controllers -c /left/controller_manager
ros2 control list_controllers -c /right/controller_manager
```

Both `joint_impedance_controller` instances should be `active`. A successful process launch alone is not proof of hardware readiness.

### Base runtime recovery

From the arm computer:

```bash
ssh tmr-user@172.16.0.50 '~/tmr_cycle/scripts/19_ensure_navigation_stack.sh'
```

### Camera health

On the arm computer:

```bash
curl -fsS http://127.0.0.1:18080/healthz
```

Check that the reported frame sequence advances across requests. A browser image remaining on screen is not evidence of a live stream. Keep healthy camera services running throughout a task; do not switch camera sessions during active manipulation.

### Restore gamepad control

On the base computer, after the task has exited:

```bash
~/tmr_cycle/scripts/17_control_mode.sh teleop
```

## Offline verification

Offline tests do not connect to or move the robot. Use an isolated Python environment rather than installing test packages into the vendor ROS environment:

```bash
python -m venv .venv
```

Activate it with `source .venv/bin/activate` on Linux, or `.\.venv\Scripts\Activate.ps1` in Windows PowerShell, then run:

```bash
python -m pip install -r requirements-offline.txt
python -m pip install --no-deps -e grasp
python -m pip check
python -m pytest base/tests grasp/tests grasp/scripts/test_pick_cycle_policy.py mission/tests
python -m compileall -q base/scripts grasp/scripts mission/scripts tools
```

The pinned test profiles prevent unreviewed numerical-library upgrades from changing perception results. The primary profile uses OpenCV 4.12; `requirements-offline-base.txt` tests the base-compatible OpenCV 4.10/NumPy 1.x combination in a separate environment. OpenCV 5 is not part of the validated profiles. GitHub Actions runs the full test selection, including `mission/tests`, for both profiles.

The integrated task has been demonstrated on the configured robot. Offline checks validate software behavior, not hardware readiness or suitability for a different venue. Use the workflow badge above to check the result for the latest submitted commit.

## Repository layout

| Directory | Contents |
| --- | --- |
| `base/` | Mobile-base integration and supporting tools |
| `grasp/` | Manipulation, perception, and hardware integration |
| `mission/` | Task entry points and recovery coordination |
| `tools/` | Camera viewers and operator utilities |
| `docs/` | Engineering and operational reference material |

Task logs and checkpoints are stored under `~/.tmr_three_object_delivery/` on the arm computer. Keep the relevant run identifier and logs when reporting a problem. Do not commit passwords, SSH keys, raw sensor recordings, or temporary field-debug images.
