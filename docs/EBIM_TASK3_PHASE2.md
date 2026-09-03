# EBiM Task 3 — Phase II Submission Guide

| Field | Value |
| --- | --- |
| Competition | EBiM Competition 2026 |
| Task | Task 3 — Assisted Living & Feeding |
| Phase | Phase II — Real-Robot Validation |
| Repository | <https://github.com/Speidel0402/tmr-mobile-manipulation> |
| Container | `tmr-mobile-manipulation:ebim-task3-phase2` |
| Task entry | `mission/scripts/run_complete_from_start.sh` |

This guide packages the existing three-object delivery task. The original policy, parameters, calibration files, and operating instructions are unchanged. The [root README](../README.md) remains the operating reference for service startup, initial conditions, interruption, recovery, and offline verification.

The submission fields below follow the [official Phase II Policy Submission form](https://github.com/EBiM-Benchmark/submissions/blob/82338edc557f82728c97612d800dfdfa1b3a54d5/.github/ISSUE_TEMPLATE/phase2-submission.yml).

## Compatibility and validation status

The original `base/`, `grasp/`, and `mission/` code and configuration are unchanged from the [pre-Docker revision](https://github.com/Speidel0402/tmr-mobile-manipulation/tree/2d626067904a3b847a66301c87d11833431b929a). Building this image installs its dependencies inside the image; it does not replace the robot hosts' Python, ROS, CUDA, drivers, or calibrated environment. The original bare-metal entry remains available.

The container build, motion-disabled plan, launcher help, and existing offline tests have passed. A real-robot container run, timing measurements, and remote interruption/disconnection behavior have not been validated by these checks.

- **Execution path:** bare-metal execution at the standard arm-host path runs arm phases locally. The container coordinator runs those same phases through the existing SSH branch. Both robot hosts must be reachable and authenticated, including arm-host self-login when Docker runs there.
- **Timing:** task order, completion gates, motion parameters, and stage timeouts are unchanged. Control loops remain on the robot hosts, but SSH connection and output-transport delays contribute to the existing stage timeouts. Equal end-to-end latency is not guaranteed.
- **Interruption:** stopping the container requests interruption of the coordinator and its local SSH processes. There is no remote-arm termination acknowledgement in this path; container exit alone does not prove that remote motion processes stopped.
- **Exclusive operation:** container and bare-metal checkpoints use different paths and therefore different file locks. Never run both entries concurrently. The container name and existing checkpoint locks do not provide a shared lock across the two modes.
- **Host deployment:** the archive commands below overwrite same-name host files. Save the existing host directories and site-specific modifications before deployment; an unchanged Git policy does not guarantee that a field machine has no uncommitted differences.

## Build and run commands

Run the following on a Linux computer with Docker Engine, Git, an SSH agent, and access to the configured robot network. The operator's SSH agent must already authenticate as both `aup@172.16.0.100` and `tmr-user@172.16.0.50`; both verified host keys must be in `$HOME/.ssh/known_hosts`. No SSH private key is built into the image.

### 1. Check out the pinned submission

Replace `<PINNED_COMMIT_SHA>` with the full 40-character SHA entered in the submission form. Use an empty working directory for this checkout.

```bash
git clone https://github.com/Speidel0402/tmr-mobile-manipulation.git
cd tmr-mobile-manipulation
git checkout --detach <PINNED_COMMIT_SHA>
git rev-parse HEAD
```

### 2. Build and inspect the motion-disabled plan

```bash
docker build \
  --build-arg VCS_REF="$(git rev-parse HEAD)" \
  -t tmr-mobile-manipulation:ebim-task3-phase2 .

docker run --rm --network none \
  tmr-mobile-manipulation:ebim-task3-phase2
```

The default command prints the existing plan with `"status": "dry_run"` and `"motion_enabled": false`. It does not connect to the robot. The image's `org.opencontainers.image.revision` label records the supplied checkout SHA.

### 3. Deploy the same pinned files to the robot hosts

Do this before starting a task, with active tasks and affected services stopped. The robot hosts must already have the vendor environments and device configuration described in [System requirements](../README.md#system-requirements). These commands install the submitted repository files at the paths expected by the existing policy; they do not install vendor software. Retain any machine-specific configuration outside the submitted files.

Before an initial deployment, save the existing directories, including local calibration and configuration edits. Keep the resulting archives outside the deployment directories. On the arm host (`aup@172.16.0.100`):

```bash
tar -czf "$HOME/tmr-mobile-manipulation-before-phase2-$(date -u +%Y%m%dT%H%M%SZ).tar.gz" \
  -C "$HOME" tmr-mobile-manipulation
```

On the base host (`tmr-user@172.16.0.50`):

```bash
tar -czf "$HOME/tmr-cycle-before-phase2-$(date -u +%Y%m%dT%H%M%SZ).tar.gz" \
  -C "$HOME" tmr_cycle
```

Complete and retain both backups before returning to the clean pinned checkout and deploying:

```bash
git archive HEAD | ssh aup@172.16.0.100 \
  'mkdir -p /home/aup/tmr-mobile-manipulation && tar -x -C /home/aup/tmr-mobile-manipulation'

git archive HEAD:base | ssh tmr-user@172.16.0.50 \
  'mkdir -p /home/tmr-user/tmr_cycle && tar -x -C /home/tmr-user/tmr_cycle'
```

Deploy the full repository on the arm host and the **contents of `base/`** on the base host. `git archive` preserves the submitted executable permissions. Both hosts must use these files for the evaluated run; the container does not automatically synchronize host files.

If the base host's letter-vision environment (`~/tmr_cycle/.venv-letter` or `~/tmr_cycle/.letter-deps`) has not already been prepared, run the existing one-time setup before starting the task:

```bash
ssh tmr-user@172.16.0.50 '~/tmr_cycle/scripts/14_prepare_letter_vision.sh'
```

This setup requires pre-provisioned wheels or package-download access; the task itself does not install packages. Start or restore robot services using the unchanged [cold-start procedure](../README.md#b-cold-start-or-services-have-been-stopped).

### 4. Check the container's SSH access

```bash
test -S "$SSH_AUTH_SOCK"
test -f "$HOME/.ssh/known_hosts"

docker run --rm \
  --mount type=bind,src="$SSH_AUTH_SOCK",dst=/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --mount type=bind,src="$HOME/.ssh/known_hosts",dst=/root/.ssh/known_hosts,readonly \
  tmr-mobile-manipulation:ebim-task3-phase2 \
  bash -lc 'ssh -o BatchMode=yes -o ConnectTimeout=5 aup@172.16.0.100 true && ssh -o BatchMode=yes -o ConnectTimeout=5 tmr-user@172.16.0.50 true'
```

This access check does not launch a robot task. The container uses SSH for both hosts even when Docker runs on the arm computer itself, so self-login to `172.16.0.100` must also work in that case.

### 5. Run the complete task

Use the original [ready-to-run conditions](../README.md#a-robot-services-are-already-running): robot at the designated start, empty grippers, calibrated scene, healthy required services, and no other active autonomous task. The example retains the existing cup `B`, bowl `A`, plate `D` assignment; destination letters remain command-line parameters.

```bash
mkdir -p "$PWD/outputs/phase2-state"

docker run --rm --init -it \
  --name tmr-ebim-task3-phase2 \
  --mount type=bind,src="$SSH_AUTH_SOCK",dst=/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --mount type=bind,src="$HOME/.ssh/known_hosts",dst=/root/.ssh/known_hosts,readonly \
  --mount type=bind,src="$PWD/outputs/phase2-state",dst=/state \
  tmr-mobile-manipulation:ebim-task3-phase2 \
  bash mission/scripts/run_complete_from_start.sh \
    --checkpoint /state/state.json \
    --log-dir /state/logs \
    --cup-letter B --bowl-letter A --plate-letter D
```

The container runs the existing coordinator from `/opt/tmr-mobile-manipulation`. Arm commands execute on `aup@172.16.0.100` under `/home/aup/tmr-mobile-manipulation`, using `/home/aup/tmr_env.sh`; base commands execute on `tmr-user@172.16.0.50` under `/home/tmr-user/tmr_cycle`. Real-time control stays on its corresponding robot host. Container checkpoints and logs persist in `outputs/phase2-state/` on the Docker host.

Use `Ctrl+C` in the attached terminal as described in the original operating instructions. From another terminal, the container's `SIGINT` stop signal can request the same software-interruption path:

```bash
docker stop --time 45 tmr-ebim-task3-phase2
```

Docker sends the configured stop signal to the container and may forcibly terminate it after the timeout ([Docker stop documentation](https://docs.docker.com/reference/cli/docker/container/stop/)). This is a software-interruption request, not proof of remote robot stoppage. Confirm that the corresponding remote task/motion processes have exited and that both arms and the base are stationary before restarting or switching to fallback. The original [intermediate recovery instructions](../README.md#intermediate-recovery) still apply.

## Fallback to the original bare-metal entry

If container startup, SSH access, or execution is unsuccessful, the original operating method remains available. No Git rollback or Dockerfile removal is required: the original task code is identical in this packaging revision.

1. On the Docker host, request shutdown and check that the named container is no longer running:

   ```bash
   docker stop --time 45 tmr-ebim-task3-phase2
   docker ps --filter 'name=^/tmr-ebim-task3-phase2$'
   ```

   If startup failed or the `--rm` container already exited, Docker may report that it does not exist. Still perform the remote-process and physical-state checks below.

2. Confirm on both robot hosts that this run's task/motion processes have exited and confirm the physical robot is stationary. If they have not, follow the existing host recovery procedure before proceeding; do not rely on container exit alone.
3. If deployment overwrote field-specific changes, restore the saved host files while tasks and affected services are stopped. A Git checkout cannot recover uncommitted calibration or configuration that was overwritten without a backup.
4. With the robot at the designated start, empty grippers, and healthy services, run the original command **on the arm computer at its standard path**:

   ```bash
   cd /home/aup/tmr-mobile-manipulation
   bash mission/scripts/run_complete_from_start.sh \
     --cup-letter B --bowl-letter A --plate-letter D
   ```

This restores the original local arm execution path. If services have stopped, use the original Windows cold-start helper first. If the robot is at an intermediate state, use the explicit recovery entry corresponding to a confirmed physical state instead of the complete-start command.

Container state is stored in the Docker host's `outputs/phase2-state/`; the bare-metal entry uses `~/.tmr_three_object_delivery/` on the arm host. They do not share checkpoints or locks, and fallback does not automatically resume the container checkpoint. Keep the failed run's logs for diagnosis.

## Startup and initialization time

No measured full startup duration is recorded in the repository, so a fixed number of minutes cannot be promised.

| Startup path | What the code establishes |
| --- | --- |
| Default Windows cold-start helper | The configured waits total **56 seconds** (`8 + 8 + 5 + 7 + 12 + 5 + 8 + 3`). SSH/ROS readiness checks, arm movement, gripper initialization, and spine movement take additional time. The helper has no overall elapsed-time deadline. |
| Complete-task entry with services already running | It still obtains base control and initializes the grippers, arms, and spine. This is the original behavior, including after the cold-start helper has completed. |
| Task preparation timeout budgets | Base readiness/control: **150 seconds**; dual-arm/gripper initialization: **180 seconds per attempt**; spine initialization: **180 seconds per attempt**. These are failure guards, not measured normal durations or a total startup estimate. |
| Docker coordinator startup | No extra deliberate startup sleep or runtime package installation is added. Image download/build time and SSH connection time are separate from hardware initialization. |

See the original [startup delay configuration](../grasp/config/system_startup.psd1), [cold-start helper](../grasp/scripts/start_tmr_system.ps1), and [task initialization](../mission/scripts/run_three_object_delivery.py). Some failed initialization reports can trigger a retry; an elapsed-time timeout aborts the stage. Do not interpret the 180-second guard as a promise that initialization normally takes three minutes.

For a measured cold-start duration on the configured platform, time the existing helper after manual power-on and FCI activation are complete. Run this from the Windows repository root only when the robot is ready for initialization:

```powershell
$tmrStartupWatch = [System.Diagnostics.Stopwatch]::StartNew()
powershell -ExecutionPolicy Bypass -File .\grasp\scripts\start_tmr_system.ps1
$tmrStartupExitCode = $LASTEXITCODE
$tmrStartupWatch.Stop()
Write-Host ('Startup helper elapsed: {0:N1} minutes; exit code: {1}' -f $tmrStartupWatch.Elapsed.TotalMinutes, $tmrStartupExitCode)
```

The timer measures the helper invocation, not the complete competition task. Record success/failure and the starting hardware state together with the elapsed time before publishing a normal startup-time estimate.

## Environment and dependencies

| Component | Submitted environment |
| --- | --- |
| Container base | `python:3.11-slim-bookworm` (Debian Bookworm, Python 3.11) |
| Container packages | `bash`, `ca-certificates`, `openssh-client`; coordinator imports use the Python standard library |
| Container ROS / CUDA / GPU | None required by the coordinator image |
| Docker host | Linux with Docker Engine, SSH agent, verified `known_hosts`, persistent state directory, and robot-network access |
| Arm host | The existing ROS 2 Jazzy and vendor environment sourced from `/home/aup/tmr_env.sh` |
| Base host | The existing ROS 2 Humble environment and configured base, LiDAR, and head-camera services |
| Runtime network | Required: SSH access from the container to both robot hosts, plus the existing host-to-host and camera/service connectivity |
| Public internet at run time | Not required by the container's task coordinator; host vendor software and any separately provisioned resources must already be available |

The container does not need privileged mode, GPU passthrough, host networking, or direct device mounts. Robot drivers, SDKs, ROS workspaces, calibration, camera services, and vendor-specific dependencies remain prerequisites on the robot hosts. The [original system requirements](../README.md#system-requirements) and [offline verification](../README.md#offline-verification) sections are preserved in full.

## Hardware assumptions

The existing platform is the TMR mobile base with dual Franka FR3 arms, Robotiq grippers, a lifting column, dual LiDAR, wrist-mounted RealSense D405 cameras, and a head-mounted ZED camera. The policy assumes the configured scene, camera placement, calibration, designated start, and empty-gripper initial condition described in the original README.

The [architecture reference](ARCHITECTURE.md) specifies a 20 Hz base control loop. The container itself performs orchestration and requires no GPU inference. GPU model/VRAM requirements for the installed host services, their exact driver versions, other control/inference rates, and quantified lighting/scene tolerances are not specified by the original submission content; the team must supply these deployment details in the form rather than assume values.

Reset to the designated start with empty grippers before a complete rerun. Use the original explicit recovery entries only after confirming the stated physical conditions.

## Team fields and declarations

Use the title format `[Phase II] <team name> — Task 3 — Assisted Living & Feeding` and select `Task 3 — Assisted Living & Feeding` in the official form.

| Official field | How to complete it |
| --- | --- |
| Team name / Point of Contact email | Use the team's current registration details. |
| Assigned Phase II pathway | Use the assignment in the team's advancement email. |
| Public GitHub repository URL | Use the repository URL at the top of this guide. |
| Pinned commit SHA | Use `git rev-parse HEAD` for the final public commit that contains these packaging files. Keep it reachable from a branch or tag. |
| Build and run commands | Use the commands above with the pinned SHA and the actual destination-letter assignment. |
| Environment and dependencies | Use the table above and supply the missing host-specific version details. |
| Hardware assumptions | Use the preserved operating conditions and complete the host/scene details above. |
| Externally provided object poses | The current code combines onboard RGB object/letter detection with calibrated or taught arm poses, fixed reference heights, and object-specific descent/placement offsets. Review the form's `Partly — see Notes` option against the actual deployment; do not describe the policy as having no pose priors. |
| Organizer-released trajectory dataset | Supply the team's actual training-data declaration; repository contents do not establish training provenance. |
| Changes since Phase I / supplementary links / notes | Optional; retain the team's actual information. |

The packaging workflow builds the image and checks only the motion-disabled plan and launcher help, with networking disabled. It does not establish that a real-robot run from the pinned clean checkout succeeded. Complete the official clean-checkout/run confirmation only after the team has performed that run. Team acknowledgements must be completed by the submitting team.

Use the deadline in the team's Phase II advancement email, as required by the current official form.
