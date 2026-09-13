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
| Head camera | Shanghai compressed path | Hamburg raw ZED topic; require 640×360 `bgr8` via ZED `pub_downscale_factor:=2.0` |
| Arms and grippers | MoveIt/PTP, Robotiq actions in mission scripts | Hamburg Gello `JointState` and Float32 width topics are identified; motion policy not yet ported |
| Node lifecycle | Phase scripts repeatedly create ROS nodes | Read-only check is one node; mission remains locked until a one-node port exists |
| Calibration | Shanghai table and grasp geometry | Preserved as an assumption; requires Hamburg physical acceptance |

The existing `mission/scripts/run_three_object_delivery.py`, object pick
scripts, and base scripts cannot be launched from Hamburg unchanged. They
spawn new processes and ROS participants during active motion, use interfaces
not established on the testbed, and expect Shanghai camera and host services.
Changing only the launcher or passing the new preflight would conceal those
incompatibilities. The physical `mission` mode therefore remains fail-closed.

The next implementation milestone is a long-lived Humble controller that
creates all subscriptions, publishers, service clients, and action clients
before motion; directly consumes odometry and ROS images; uses only confirmed
native arm, gripper, spine, and base commands; checks freshness, bounds,
feedback, and stop conditions on every phase; and runs cup/bowl/plate acceptance
before a full Stage 1 trial. Real robot testing is required to validate timing
and manipulation success. The current offline checks validate contracts only.
