# Hamburg integration audit

The organizer's ready preflight proves the bridged read-only graph, not a grasp
or Stage 1. The minimum Hamburg adaptation now provides:

- pickup-reset: manual table-side base placement followed by native spine/arm
  reset and a fresh 640×480 left-wrist review frame, with no base command path;
- grasp-test: wrist observation, visual XY correction, object-specific
  descent, close, lift, gripper contact/retention check, release and retract;
- mission: the same grasp loop plus native odometry base motion, raw head-ZED
  letter centering, placement and return, all in one node;
- site-config: explicit measured overrides without editing the launcher.

Cup/bowl/plate descents and near/far placement paths pass offline IK checks
against the official FR3v2 chain and joint limits. The 50 Hz smooth command
stream accounts for its peak interpolation velocity. Physical reports remain
the evidence for controller timing, calibration and success.

Reference sources: Franka's
[follower controller](https://github.com/frankarobotics/franka_follower_controllers)
explicitly accepts low-frequency absolute joint targets from teleoperation or
AI publishers, and the local solver uses the official FR3v2
[kinematics](https://github.com/frankarobotics/franka_description/blob/main/robots/fr3v2/kinematics.yaml)
and [joint limits](https://github.com/frankarobotics/franka_description/blob/main/robots/fr3v2/joint_limits.yaml).

Head downsampling needs no separate letter-point remap because card centres are
normalized and annotations use the current frame. It can reduce OCR confidence,
so the mission saves venue frames. It does not alter the wrist camera resolution
or wrist grasp mapping.

The default result must be described as a **Shanghai-reference trial on Hamburg
native interfaces**, not venue generalization. Room length/width, door clear
width/location, tabletop height/size, pickup standoff, letter spacing and each
route segment are independent Hamburg measurements.
