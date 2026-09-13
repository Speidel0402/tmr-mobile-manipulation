# Hamburg integration audit

This audit interprets the two organizer messages and the supplied
`EDL_task3_hamburg_preflight_ready.json` as observations, not instructions to
discard the fail-closed mission gate. The JSON describes source policy commit
`53e4909879716868f57fa3e315ef8ded75124a90`, before the native adaptation.

## What the organizer actually established

The first message says the previous check returned `ready`, no errors, and
`motion_commanded: false`, then explicitly says Stage 1 did not run. The second
message explains why: the organizer temporarily supplied odometry-to-pose/twist,
position-service-to-joint-state, and Float32-to-`MoveAbsolute` relays, and set
ZED `pub_downscale_factor:=2.0`. `ready` therefore means that a *bridged,
read-only interface check* passed. It does not prove that an unbridged Hamburg
process, any grasp, or the complete mission works.

The attached JSON confirms live arm/gripper, odometry, LiDAR, camera and TF
streams and controller subscribers in that bridged graph. It has no action
endpoint entry, goal schema, actual `GetPosition` service type, arm joint names,
base command frame/watchdog, gripper contact feedback, CPU or latency series.
Its `service_count: 428` is a count, not proof that the required service is
callable. The new check probes the action and service directly but still cannot
prove motion semantics.

## Executable/module consistency

| Module | Hamburg status | Remaining dependency |
| --- | --- | --- |
| `run_hamburg.sh check` / `hamburg_preflight.py` | Native odometry plus spine action/service, one read-only node, domain inherited; blocks if the four known relay topics remain in the graph | Must be run on the actual Humble companion |
| `spine_control.py` | Action client uses a caller-owned node; checks bounds, result and measured height | Not wired into a runnable mission; action name and `GetPosition` type need live confirmation |
| `run_hamburg.sh grasp-check` / `hamburg_grasp_check.py` | One read-only node, 640x480 raw left-wrist RGB and existing cup/bowl/plate detectors, posture and spine checks; no head ZED requirement | Detectability is not grasp success; wrist-camera pixel calibration and posture need venue acceptance |
| `run_hamburg.sh grasp-test` | Explicitly locked | No validated Gello motion planner or Float32 gripper contact proof |
| `run_hamburg.sh mission` | Explicitly locked | No one-node port of pickup, travel, search and placement |
| `mission/scripts/run_standalone_grasp_test.py` | Shanghai executor rejects Humble execution | It spawns phase processes and uses MoveIt/PTP, Robotiq actions and old camera snapshot path |
| `mission/scripts/run_three_object_delivery.py` | Shanghai-only | SSH, multiple processes/nodes and a base command adapter unavailable in Hamburg |
| `grasp/scripts/run_streamed_*_pick_cycle.py` | Detector functions reused read-only; motion path not reused | PTP/FK/Cartesian services, action result fields, HTTP snapshot and legacy camera topics |
| `base/scripts/*` | Not invoked by Hamburg launcher | Several scripts export domain 97/CycloneDDS and start drivers/teleop; Hamburg uses organizer stack |

## Variables, ROS graph and calibration

| Item | Hamburg value | Consistency / qualification |
| --- | --- | --- |
| Host / ROS | one aarch64 Ubuntu 22.04 companion; Humble/Python 3.10 | Hamburg launcher does not source the Shanghai Jazzy arm environment or SSH to a second host |
| DDS | domain 0, `rmw_fastrtps_cpp`, organizer UDP-only profile | Hamburg code checks and inherits these values; legacy base scripts use domain 97/CycloneDDS and are not on the Hamburg path |
| Base state / command | `/swerve_drive_controller/odom` `Odometry`; `/swerve_drive_controller/cmd_vel` `TwistStamped` | Names/types confirmed in bridged report; command frame, timeout and ownership not confirmed |
| Spine | `MoveAbsolute` + `/franka_spine_node/get_position`; 0.7 m home | Action *type* and service *name* stated by organizer; `/franka_spine_node/move_absolute`, `GetPosition` type, goal/result fields and accepted range are inferred from Shanghai source pending native check |
| Grippers | Float32 width topics, 0.8 open / 0.0 closed | Organizer confirms endpoints/convention; old action uses the opposite numeric direction and its stall/result fields are absent from a Float32 publication |
| Wrist RGB | `/wrist_camera_left/camera/color/image_raw`, 640x480 `rgb8` | Dimensions match Shanghai detector gates, but old HTTP snapshot validator expects `/wrist_camera_left/color/image_raw`; mount, intrinsics and visual targets remain unverified |
| Head RGB | `/head_camera/zed_node/rgb/color/rect/image`, 640x360 `bgr8` after 2x downscale | Required by the full preflight/mission path, not standalone `grasp-check`. Current ZED wrapper needs `general.pub_resolution: CUSTOM` for `general.pub_downscale_factor: 2.0` to apply; image and camera_info must be verified. Old letter-search uses a different compressed topic and pixel assumptions may change |
| Pickup posture | left pick-top and right parking targets copied from `grasp_initial_state.yaml` | Reference only; the venue has not accepted their clearance or joint-order semantics against a different table |
| Object descent | cup 0.340 m, bowl 0.360 m, plate 0.375 m | Shanghai reference values only. No Hamburg motion uses them until table height and arm path are measured and approved |
| Room/route geometry | Shanghai starts with 0.85 m initial forward, 0.50 m before the door and 1.20 m through it; `route.yaml` also has start-relative table and inspection positions | Hamburg room size, door opening and table location are unknown; these distances cannot be transferred or uniformly scaled |

## Effect of the organizer's temporary glue

The supplied graph still shows the original odometry, arm, gripper, LiDAR and
camera topics receiving data, so there is no evidence that the relays disabled
those endpoints during the read-only check. The odometry and position relays
add publications/polling; they may add delay and DDS load, but the report has
no rates or CPU/latency measurements. The Float32 relay can issue native spine
action goals. If a direct action controller runs at the same time, the two
command paths can compete; only one should own spine motion. The ZED setting
does alter the original head-image size for all consumers of that stream and
can invalidate pixel-based assumptions. Whether camera info is scaled with it
must be checked on the venue. None of these risks is a measured failure in
the supplied report.

The native check rejects the four named temporary relay topics if they remain
visible, so a new `ready` report cannot silently rely on those bridges. This
is a graph-level check, not proof that an unseen helper process is absent.
The [current Stereolabs wrapper configuration](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/config/common_stereo.yaml)
documents `CUSTOM` as the publishing mode that applies the downscale factor;
the organizer's installed wrapper version and live output remain authoritative.

## Head-ZED coordinates under 2x downscaling

Shanghai starts its letter-search ZED with `base/config/zed_letters_override.yaml`:
`pub_resolution: NATIVE`, `pub_downscale_factor: 1.0`, 10 published RGB frames/s,
and no depth. The launcher uses a separate vision DDS domain, exports a
compressed JPEG frame atomically, and the control process detects letters from
that file. Hamburg has so far confirmed a different *raw* `sensor_msgs/Image`
topic. A new single-node mission must subscribe to that native topic and decode
it accordingly; changing the expected width alone does not fix the transport.

The active Shanghai letter recognizer scales candidate card area and dimensions
with image width, normalizes each glyph to a 96x96 canvas, records card centres
as `x/width` and `y/height`, and makes row/centering decisions in normalized
coordinates. Its image-motion gain is learned in normalized-width per metre.
`annotate()` draws the detected quadrilateral in the *current frame's* pixel
coordinates. Thus a pure 1280x720 to 640x360 resize does not require manually
halving stored letter-card centre points or annotation positions. An offline
regression using the repository's synthetic A/B/D scene retains all three
detections, their normalized centres, and near/far rows at both sizes.

That regression does not establish venue accuracy: half-resolution loses fine
glyph detail and changes edge/threshold noise, while Shanghai's actual camera
mount, capture resolution and JPEG transport are not proved identical to
Hamburg. Compare paired venue frames, confidence, false positives, and latency
before accepting the new output. The unused `head_rgb_descent.py` path has a
normalized gripper column, but also absolute ROI and pixel-per-metre bounds;
its `descend_with_head_rgb()` function is not called by the current pick main.
It would need separate scaling/calibration if enabled later.

## Acceptance still required

1. Run the updated native `check` without organizer relays and record the
   action/service schema, fresh streams, image metadata and errors.
2. Measure Hamburg room, door and table geometry. Run read-only `grasp-check`
   separately for cup, bowl and plate at an organizer-approved stationary
   observation pose, without moving to Shanghai's pose solely for this check.
   Its result is deliberately labeled
   `grasp_executed: false`.
3. Obtain Gello joint command order, limits, rate/hold/watchdog and fault
   behavior; gripper joint-state mapping and contact evidence; base command
   frame/watchdog; head/wrist calibration and TF; and the actual spine action
   name/type/range.
4. Port the physical grasp and complete mission to one long-lived Humble node,
   then perform supervised object trials and an end-to-end timing test. Offline
   unit tests and the old bridged JSON cannot replace these trials.
