# TMR Mobile Manipulation

[![Offline tests](https://github.com/Speidel0402/tmr-mobile-manipulation/actions/workflows/offline-tests.yml/badge.svg)](https://github.com/Speidel0402/tmr-mobile-manipulation/actions/workflows/offline-tests.yml)

Competition submission for a TMR mobile manipulation platform with dual Franka FR3 arms, Robotiq grippers, a lifting column, dual LiDAR, wrist-mounted RealSense D405 cameras, and a head-mounted ZED camera.

The repository provides task orchestration, robot integration, perception components, and operator tools. This README covers setup, execution, verification, and recovery.

## System requirements

| Component | Environment | Role |
| --- | --- | --- |
| Base computer | ROS 2 Humble | Mobile base, LiDAR, and head-camera services |
| Arm computer | ROS 2 Jazzy and the configured robot environment | Arms, grippers, lifting column, wrist cameras, and task coordinator |
| Operator computer | Windows PowerShell and OpenSSH | Service startup, task launch, and observation |

The robot must already have its vendor drivers, SDKs, ROS workspaces, calibration, and device configuration installed. Installing the offline Python dependencies alone does not provision a robot.

The deployed setup uses:

- Arm computer: `aup@172.16.0.100`, repository at `/home/aup/tmr-mobile-manipulation`.
- Base computer: `tmr-user@172.16.0.50`, base package at `~/tmr_cycle`.
- Arm environment: `~/tmr_env.sh`.
- Key-based SSH from the arm computer to the base computer.

Keep the configured control and camera ROS domains separate. Run real-time control on the corresponding robot computer, not through an operator-side command loop. Credentials and machine-specific overrides are not included in the repository.

## Running the submission

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
