Hi @Speidel0402 — thank you for the Hamburg preflight and interface details.

We have added native Hamburg execution paths for the standalone utensil tests and Stage 1. The runner uses one ROS 2 Humble node throughout active motion, consumes /swerve_drive_controller/odom directly, drives the spine through franka_spine_msgs/action/MoveAbsolute and verifies it with GetPosition, and consumes the raw wrist/head images. It does not use SSH, MoveIt/PTP, Robotiq actions, or the temporary odometry/spine relay topics.

For the arms, it sends autonomous seven-joint targets to the organizer-confirmed /{left,right}/gello/joint_states controller inputs. “gello” is the deployed topic name; no GELLO leader or manual teleoperation is used. Please stop other publishers to the arm, gripper and base command topics during the autonomous test.

Please first run the three standalone observation-to-grasp loops, with only the named utensil at the pickup position:

    ./deploy/hamburg/run_hamburg.sh check --output /tmp/hamburg-check.json

    ./deploy/hamburg/run_hamburg.sh grasp-test --object cup --execute \
      --output /tmp/cup-grasp.json --output-dir /tmp/cup-evidence
    ./deploy/hamburg/run_hamburg.sh grasp-test --object bowl --execute \
      --output /tmp/bowl-grasp.json --output-dir /tmp/bowl-evidence
    ./deploy/hamburg/run_hamburg.sh grasp-test --object plate --execute \
      --output /tmp/plate-grasp.json --output-dir /tmp/plate-evidence

Each test performs a fresh 640×480 wrist observation, iterative visual correction, object-specific descent, close, lift, and contact/retention check against an empty-close baseline. It then releases the utensil and retracts. Please send the three JSON reports and aligned wrist images.

If all three pass, please inspect and run the complete cup → B, bowl → A, plate → D loop:

    ./deploy/hamburg/run_hamburg.sh mission
    ./deploy/hamburg/run_hamburg.sh mission --execute \
      --output /tmp/hamburg-stage1.json --output-dir /tmp/hamburg-stage1-evidence

The initial profile deliberately reuses the Shanghai pickup posture, wrist calibration, 0.7 m spine target, 0.340/0.360/0.375 m object descents, and individually named route values. This should be treated as a Shanghai-reference trial on Hamburg native interfaces, not as a claim that the two venues generalize. Hamburg room length/width, clear door width and location, tabletop height/size, pickup standoff, letter spacing and each route segment may differ independently; they should not be changed with one global scale factor. The repository includes a site-config command and a phase-specific troubleshooting table so we can update only the value implicated by each report.

The wrist stream remains 640×480. The full mission expects the organizer-confirmed 640×360 raw head ZED image. Letter centres are normalized and annotations use the current frame, so a pure 2× head resize needs no manual point remap, although the saved Hamburg frames are still needed to assess confidence loss. Please include the live head Image and camera_info dimensions and, if available, the arm target rate/hold and base command watchdog behavior with the results.
