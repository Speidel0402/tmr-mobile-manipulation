# Hamburg compatibility audit

Audited source revision: `53e4909879716868f57fa3e315ef8ded75124a90`.

## Result

The submitted Stage 1 strategy is internally consistent, but its Shanghai
runtime cannot be started unchanged in Hamburg.  The incompatibilities are in
deployment and control interfaces, not in the object order or destination
mapping.

| Area | Submitted runtime | Hamburg contract | Resolution in this directory |
| --- | --- | --- | --- |
| Host layout | Windows coordinator plus separate base and arm computers over SSH | One `companion` computer | No SSH or remote staging |
| ROS | Humble base plus Jazzy arm | Humble only | Humble/arm64 image and strict preflight |
| DDS | Domain 97/CycloneDDS in several launchers | Domain 0/Fast DDS/UDP-only | Validate organizer environment; never overwrite it |
| Drivers | Helper scripts can start navigation, ZED, arm, and controller processes | Organizer starts the complete robot stack | Hamburg launcher starts no drivers |
| Node lifecycle | Phase scripts repeatedly create and destroy ROS nodes | Create all nodes during startup; no new participants after arms are active | Read-only preflight is one participant; legacy mission is locked |
| Head camera | Old compressed `/head_camera/zed/...` path | Raw best-effort `/head_camera/zed_node/.../image` | Correct topic, best-effort subscription, frame shape/encoding checks |
| Wrist cameras | Old path without `/camera/` | Paths include `/camera/` | Corrected in venue contract |
| Arm motion | Custom MoveIt FK/cartesian services and PTP actions | Only Gello command topics are guaranteed | Legacy actions/services are not assumed |
| Grippers | Robotiq action server | Width-percent command topics | Legacy action is not assumed |
| Spine | Custom services/action | `/spine/target_height` topic | Legacy services/action are not assumed |
| Base command | Mission command adapter plus remote mode switching | Direct swerve command topic | Adapter and teleop stack restarts are not assumed |
| Calibration | Shanghai calibrated task geometry | Hamburg table height stated to match Shanghai | Preserve Shanghai profile, then perform one venue acceptance check |

## Code-path findings

The following submitted helpers must not be called by a Hamburg entrypoint:

- `docker/run_task3.sh` and the two remote environment loaders: remote staging
  and mixed ROS assumptions.
- `base/scripts/03_start_navigation.sh`, `18_start_zed_stream.sh`, and
  `19_ensure_navigation_stack.sh`: duplicate driver/stack startup risk.
- `grasp/scripts/start_tmr_system.ps1`: Windows and remote service startup.
- `mission/scripts/run_three_object_delivery.py`: sequential subprocess/SSH
  phases create new DDS participants while the robot is active.

The calibrated policy remains cup, bowl, plate with destinations B, A, D.  No
route distance, grasp depth, placement offset, classifier threshold, or object
ordering was changed by the Hamburg compatibility work.

## Remaining motion blocker

The venue document guarantees topic command interfaces but does not specify the
message types/units for the gripper and spine commands, and it does not
guarantee the submitted MoveIt/PTP/Robotiq/spine action interfaces.  A faithful
Hamburg mission therefore requires the organizer's live `check` report before
the manipulation state machine can be ported to one long-lived Humble node.

`hamburg_preflight.py` obtains that report without ROS CLI subprocesses and
without commanding motion.  Unlocking the legacy Shanghai mission would hide
the incompatibility and can reproduce the controller/DDS failures described in
the venue document, so `run_hamburg.sh mission` deliberately refuses to do so.
