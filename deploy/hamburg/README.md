# EBiM Task 3 — Hamburg deployment area

This directory is an isolated Hamburg compatibility package for the submitted
EBiM Task 3 Phase 2 Stage 1 policy.  It does not modify the Shanghai runtime or
the validated task strategy.

## What is preserved

- Object order: cup, bowl, plate.
- Destination mapping: cup to B, bowl to A, plate to D.
- Shanghai table-height profile as the Hamburg default.
- Existing route, perception, grasp, placement, and return strategy files.

The submitted Hamburg profile assumes the Hamburg and Shanghai pickup tables
have the same height.  No Hamburg-specific vertical offset is added.

## Why this is separate

Hamburg runs team code on the arm64 `companion` computer with Ubuntu 22.04,
Python 3.10, and ROS 2 Humble.  The organizer starts the robot drivers and
exports a Fast DDS UDP-only environment.  Team code must not overwrite that
environment, restart robot services, or create new DDS participants during an
active mission.

The submitted Shanghai launcher uses SSH across two ROS computers, includes
Jazzy support, and starts a new Python/ROS process for each phase.  Reusing it
would violate the Hamburg runtime contract.  See `COMPATIBILITY.md` for the
file-by-file audit.

## Organizer-side check

Run this after the organizer has started all robot services and exported the
DDS environment, but before enabling physical motion:

```bash
cd /path/to/tmr-mobile-manipulation
./deploy/hamburg/run_hamburg.sh check \
  --output /tmp/tmr_task3_hamburg_preflight.json
```

The command creates one read-only ROS node, receives fresh samples from the
documented robot state, camera, LiDAR, and TF streams, and verifies that every
documented command topic has a controller subscriber.  It does not invoke the
ROS CLI or publish a motion command.

A successful result has `"status": "ready"` and
`"motion_commanded": false`.  Send the complete JSON file back with any venue
interface changes; it contains the discovered topic types needed for the final
single-node manipulation port.

The exact questions and optional relay contract for the organizer are in
`ORGANIZER_ACTIONS.md`.  The preferred path is to send the generated JSON, not
to modify a working controller stack.

## Shanghai command defaults and fast overrides

Hamburg initially uses the values already validated in Shanghai.  The isolated
profile is `config/interfaces-shanghai.json`:

- both gripper targets use `std_msgs/msg/Float32`, field `data`, normalized
  opening width; `0.8` is open and `0.0` is closed;
- the startup targets are left open and right closed;
- the spine target uses `std_msgs/msg/Float32`, field `data`, an absolute height
  in metres, with the Shanghai home target of `0.7` m.

The gripper topic and the older action interface use opposite numeric
directions.  The compatibility conversion is
`width_topic_value = 0.8 - legacy_gripper_action_position`; it preserves the
validated open/close sequence without changing the policy.

If Hamburg reports a different type, field, topic, or scalar convention, set
only the corresponding environment value before `check` (and before the final
single-node runner):

```bash
export TMR_HAMBURG_GRIPPER_MESSAGE_TYPE=std_msgs/msg/Float32
export TMR_HAMBURG_GRIPPER_FIELD=data
export TMR_HAMBURG_GRIPPER_OPEN=0.8
export TMR_HAMBURG_GRIPPER_CLOSED=0.0
export TMR_HAMBURG_SPINE_MESSAGE_TYPE=std_msgs/msg/Float32
export TMR_HAMBURG_SPINE_FIELD=data
export TMR_HAMBURG_SPINE_HOME_M=0.7
```

Topic overrides are also available as
`TMR_HAMBURG_LEFT_GRIPPER_TOPIC`, `TMR_HAMBURG_RIGHT_GRIPPER_TOPIC`, and
`TMR_HAMBURG_SPINE_TOPIC`.  To inspect the resolved profile without ROS or
motion:

```bash
./deploy/hamburg/run_hamburg.sh check --print-interface-only
```

For a persistent venue profile, copy `interfaces-shanghai.json`, change only
the confirmed fields, and pass `--interface-config /path/to/profile.json`.

The separately runnable Shanghai object tests and the six-view grasp/release
montage are documented in `../../docs/STANDALONE_GRASP_TESTS.md`.  They reuse
the validated task components and always initialize the right arm into its
raised, inward, retracted parking posture before a plate test.  They are not a
substitute for Hamburg interface confirmation: run them in Hamburg only after
the preflight proves the venue command adapter and joint targets.

To reproduce the source audit against any later checkout:

```bash
python3 deploy/hamburg/audit_submission.py \
  --output /tmp/tmr_task3_hamburg_source_audit.json
```

## Container build and check

Build on an arm64 builder (or use a multi-platform builder):

```bash
docker buildx build --platform linux/arm64 \
  -f deploy/hamburg/Dockerfile \
  -t tmr-task3:hamburg-check --load .
```

Run with host networking and the organizer-provided DDS values.  The Fast DDS
profile path must be mounted at the same path inside the container:

```bash
docker run --rm --network host \
  -e ROS_DOMAIN_ID \
  -e ROS_LOCALHOST_ONLY \
  -e RMW_IMPLEMENTATION \
  -e FASTRTPS_DEFAULT_PROFILES_FILE \
  -v "${FASTRTPS_DEFAULT_PROFILES_FILE}:${FASTRTPS_DEFAULT_PROFILES_FILE}:ro" \
  -v /tmp:/runtime \
  tmr-task3:hamburg-check check \
  --output /runtime/tmr_task3_hamburg_preflight.json
```

Do not use Docker's default bridge network.  Do not source the repository's
Shanghai environment loaders in Hamburg.

## Tests

The package-level tests require only Python 3:

```bash
python3 -m unittest discover -s deploy/hamburg/tests -v
```

They verify the Hamburg host/DDS/camera contract, the preserved Stage 1
mapping, the arm64 Humble image, and the absence of SSH, ROS CLI calls, and DDS
environment overrides in the Hamburg executables.

Every push also runs `.github/workflows/hamburg-package.yml`.  It repeats the
checks with Python 3.10 on Ubuntu 22.04 and cross-builds the Hamburg image for
`linux/arm64`; this catches unavailable ROS packages or an invalid Jetson-host
image before venue delivery.

## Current execution status

`check` is reproducible and ready to use.  Physical `mission` execution remains
locked until the live report confirms the Shanghai-derived command profile and
the single-process manipulation state machine is ported.  This is intentional:
the Hamburg document does not guarantee the MoveIt/PTP, Robotiq action, spine
service, or mission command-adapter interfaces used by the Shanghai scripts.
