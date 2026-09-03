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

Wait for exit and verify the physical state before restarting. The original [intermediate recovery instructions](../README.md#intermediate-recovery) still apply.

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
