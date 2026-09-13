# Hamburg compatibility audit

Source policy revision: `53e4909879716868f57fa3e315ef8ded75124a90`.
The original strategy keeps cup → B, bowl → A, and plate → D. The September
2026 organizer report establishes the native Hamburg graph, but the attached
`ready` preflight was obtained with three temporary relay paths and ZED
2× downscaling. It did not execute motion.

| Area | Submitted Shanghai runtime | Hamburg adaptation / status |
| --- | --- | --- |
| Host and ROS | Separate Humble base and Jazzy arm computers, SSH | One Humble companion; Hamburg launcher uses neither SSH nor Shanghai environment loaders |
| DDS | Several launchers set domain 97/CycloneDDS | Validate organizer's domain 0/Fast DDS UDP-only environment without overriding it |
| Base state | `/mobile_base/pose` and `/mobile_base/twist` relay | Direct `nav_msgs/msg/Odometry` from `/swerve_drive_controller/odom` in preflight |
| Spine | Float32 target and joint-state relay assumed by old Hamburg check | Native `MoveAbsolute` action plus `GetPosition` service in preflight; a caller-owned-node action adapter exists but is not wired into a mission |
| Head camera | Shanghai compressed path | Hamburg raw ZED topic; require 640×360 `bgr8` with ZED `general.pub_resolution: CUSTOM` and `general.pub_downscale_factor: 2.0` |
| Arms and grippers | MoveIt/PTP, Robotiq actions in mission scripts | Hamburg Gello `JointState` and Float32 width topics are identified; motion policy not yet ported |
| Standalone grasps | Shanghai script initializes/moves each arm and uses action feedback to prove contact | Hamburg `grasp-check` verifies a static object-specific view from the 640×480 left wrist camera without motion or head-ZED dependency; physical `grasp-test` remains locked |
| Node lifecycle | Phase scripts repeatedly create ROS nodes | Read-only check is one node; mission remains locked until a one-node port exists |
| Calibration and room geometry | Shanghai table height, pickup pose, doorway and route distances | Treat as references only; measure Hamburg table/room/door and approve new paths before motion |

The existing `mission/scripts/run_three_object_delivery.py`, object pick
scripts, and base scripts cannot be launched from Hamburg unchanged. They
spawn new processes and ROS participants during active motion, use interfaces
not established on the testbed, and expect Shanghai camera and host services.
Changing only the launcher or passing the new preflight would conceal those
incompatibilities. The physical `mission` mode therefore remains fail-closed.

The organizer's temporary bridges appear additive in the supplied graph: the
native odometry, arm, gripper, LiDAR and camera streams are still present. That
report cannot prove they remain healthy under concurrent physical motion.
The ZED downscale does change the actual head-image dimensions, so consumers
using absolute pixel coordinates, camera intrinsics, or a 1280x720 assumption
must be recalibrated or run against a separate compatible stream. The old
Shanghai wrist snapshot rejects Hamburg's `/camera/` topic path, and the old
gripper contact classifier requires Robotiq action fields absent from the
Float32 command topic. Running both native and relay spine command paths at
once also risks competing goals; the Hamburg motion port must own one path.

Shanghai's route values such as `initial_forward_m: 0.85`,
`before_door_m: 0.50`, `forward_from_before_door_m: 1.20`, and start-relative
table/letter positions are not Hamburg room dimensions. They cannot be scaled
from a single room-length ratio; Hamburg needs a measured map, door clearance,
pickup pose, and fresh route validation. The 0.7 m spine home and object
descents are not portable table-height corrections.

The next implementation milestone is a long-lived Humble controller that
creates all subscriptions, publishers, service clients, and action clients
before motion; directly consumes odometry and ROS images; uses only confirmed
native arm, gripper, spine, and base commands; checks freshness, bounds,
feedback, and stop conditions on every phase; and runs cup/bowl/plate acceptance
before a full Stage 1 trial. Real robot testing is required to validate timing
and manipulation success. The current offline checks validate contracts only.
