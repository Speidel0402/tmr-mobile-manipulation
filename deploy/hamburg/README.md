# EBiM Task 3 — Hamburg deployment

This directory contains the Hamburg interface check and the starting point for
porting Stage 1 to the organizer's single `companion` computer (aarch64,
Ubuntu 22.04, ROS 2 Humble). The preserved object
order is cup, bowl, plate, with destinations B, A, D. The Shanghai table-height
assumption is unchanged and still needs venue acceptance.

## Current status

`check` is read-only. It has been updated to inspect Hamburg's native interfaces
without organizer relay nodes. `mission` remains locked: the submitted Shanghai
pickup, transport, and placement programs create ROS participants during motion
and require MoveIt/PTP and camera services not established by the Hamburg
preflight. A passing interface check does **not** authorize Stage 1 motion.

## Native interface profile

`config/interfaces-shanghai.json` now defaults to the organizer-confirmed
`franka_spine_msgs/action/MoveAbsolute` action type at
`/franka_spine_node/move_absolute` and the
`franka_spine_msgs/srv/GetPosition` service at
`/franka_spine_node/get_position`. The organizer confirmed the service name,
but the action name and service type come from the Shanghai source and still
need a native Hamburg check. The service is queried without sending a goal.
The `franka_spine_msgs` Python package must be built for and sourced into the
same Humble environment as the team process; the generic Docker image does not
contain that vendor interface package.

`spine_control.py` contains the native motion adapter for a caller-owned ROS
node. It creates its action and service clients at construction, checks the
target range and action result, and verifies the measured final height. The
mission does not yet call this adapter.

Base state comes directly from `/swerve_drive_controller/odom` as
`nav_msgs/msg/Odometry`. No `/mobile_base/pose`, `/mobile_base/twist`, or
`/spine/joint_states` relay is required. Arm, gripper, LiDAR, camera, TF, and
base-command topics remain as documented in `config/venue.json`.

The gripper profile retains `std_msgs/msg/Float32` with `0.8 = open` and
`0.0 = closed`. The spine home target remains 0.7 m. Those values come from
the Shanghai submission and organizer's interface report; the check itself
sends no command and cannot validate grasp mechanics or calibration.

The head-camera contract requires 640×360 `bgr8` at
`/head_camera/zed_node/rgb/color/rect/image`. The organizer obtained that size
from its HD720 ZED using 2x downscaling. In the current Stereolabs ROS 2
wrapper, set `general.pub_resolution: CUSTOM` together with
`general.pub_downscale_factor: 2.0`; the factor alone is ignored under the
default `NATIVE` publishing mode. Confirm the actual image and `camera_info`
dimensions before motion. The expected dimensions
can be changed using `TMR_HAMBURG_HEAD_CAMERA_WIDTH` and
`TMR_HAMBURG_HEAD_CAMERA_HEIGHT`; this changes validation only, not the
Shanghai camera calibration or mission processing.

The spine command interface is selectable with
`TMR_HAMBURG_SPINE_INTERFACE=action|topic`. `action` is the Hamburg default.
The topic option is retained for a different venue that actually subscribes to
`/spine/target_height`; it must not be used to claim native Hamburg readiness.
The action and position service names can be overridden with
`TMR_HAMBURG_SPINE_ACTION_NAME` and
`TMR_HAMBURG_SPINE_POSITION_SERVICE`. The gripper and scalar settings remain
listed in the profile's `environment_overrides` map.

## Read-only venue check

After the organizer starts the robot stack and exports the venue's DDS values:

```bash
./deploy/hamburg/run_hamburg.sh check \
  --output /tmp/tmr_task3_hamburg_preflight.json
```

The program uses one ROS node and does not start services or publish motion. It
requires fresh arm, gripper, odometry, LiDAR, camera, and TF streams; live command
subscribers; a ready native spine action; and a successful position service
response. A native result is useful only when `status` is `ready`, `errors` is
empty, `motion_commanded` is `false`,
`ros_graph.temporary_relay_topics_present` is empty, and
`ros_graph.native_spine` confirms both endpoints. The check blocks if the four
known organizer bridge topics remain visible. A prior report produced with
organizer relays does not establish native readiness.

Run `check --print-interface-only` to inspect the resolved profile without ROS.
The generic image in `Dockerfile` verifies Humble/arm64 packaging, but a live
native check inside it requires the organizer's `franka_spine_msgs` overlay in
that container. Running directly in the sourced companion environment is the
simplest path.

## Offline verification and physical tests

```bash
python3 -m unittest discover -s deploy/hamburg/tests -v
python3 -m compileall -q deploy/hamburg
python3 deploy/hamburg/hamburg_preflight.py --print-interface-only
./deploy/hamburg/run_hamburg.sh grasp-check --object cup --plan
```

`grasp-check --object cup|bowl|plate` without `--plan` is a read-only live
observation on the Humble companion. It uses one node, checks the native
spine action/service, both arm and gripper state streams, command subscribers,
the Shanghai pickup/parking joint targets, the 0.7 m spine height, and five
fresh left-wrist frames with stable object-specific rim detections. For example:

```bash
./deploy/hamburg/run_hamburg.sh grasp-check --object cup \
  --output /tmp/hamburg_cup_observation.json
```

The result explicitly records `grasp_executed: false` and
`motion_commanded: false`; `observation_ready` means only that this static view
was detectable. The selected object, table height, joint targets, and visual
calibration still require physical acceptance. `grasp-test` refuses motion.
The separately runnable Shanghai grasp tests in
`../../docs/STANDALONE_GRASP_TESTS.md` still depend on Shanghai
MoveIt/PTP/gripper-action/camera interfaces and must not run on Hamburg.
A physical Hamburg grasp test and full Stage 1 trial require a single-node motion
port, calibration review, and an organizer-supervised run. See
`COMPATIBILITY.md` and `ORGANIZER_ACTIONS.md` for the remaining work.
The evidence and cross-module/domain audit are in `INTEGRATION_AUDIT.md`.
