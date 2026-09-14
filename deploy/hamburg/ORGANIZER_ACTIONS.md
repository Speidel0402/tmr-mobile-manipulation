# Hamburg organizer run sequence

In every new terminal, source both ROS Humble and the Hamburg testbed workspace
that provides the native spine action/service Python types, in that order:

    source /opt/ros/humble/setup.bash
    source <hamburg-testbed-workspace>/install/setup.bash

Use the testbed workspace path that launches `/franka_spine_node`. If `check`
reports `ModuleNotFoundError: No module named 'franka_spine_msgs'`, that terminal
has not sourced this overlay (or the overlay was built without the package); it
does not mean the running robot lacks the native spine interface.

Use the pinned commit on the single Humble companion with the normal domain 0 /
Fast DDS UDP-only environment. Temporary odometry/spine relays are not needed.
Run the native check without the odometry bridge or Float32-to-action relay;
the mission calls MoveAbsolute directly.
Stop manual publishers to the arm, gripper and base command topics during the
autonomous run.

Keep the left and right joint-impedance controllers inactive when starting a
physical entrypoint, then let the normal Hamburg activation procedure activate
them while this runner is publishing. The runner first sends each arm's current
measured joints as a neutral GELLO sample, then continuously maintains the
stream. It converts every absolute robot target with Hamburg's confirmed
direction map `[-1,-1,1,1,1,1,-1]`. Do not pre-activate the controller against
an older GELLO sample and do not publish robot-space targets directly to the
GELLO topic.

The earlier constant following error came from treating this relative interface
as absolute. The following-error guard, catch-up behavior, feedback-age check
and final target tolerance remain active after the encoding correction. Arm
publishing runs independently from input callback servicing, and the measured
feedback-age limit is now a bounded venue setting. No speed or trajectory-timing
override is required for the first retry.

Place the cup, bowl and plate in the normal pickup arrangement, then manually
place the stopped robot beside the pickup table with clear arm workspace. Run
the pickup reset first. It moves only the spine, arms and left gripper, saves a
fresh 640×480 left-wrist frame, and does not create a base command publisher:

    ./deploy/hamburg/run_hamburg.sh check --output /tmp/hamburg-check.json
    ./deploy/hamburg/run_hamburg.sh pickup-reset
    ./deploy/hamburg/run_hamburg.sh pickup-reset --execute \
      --output /tmp/pickup-reset.json --output-dir /tmp/pickup-reset-evidence

Confirm that `/tmp/pickup-reset-evidence/pickup-reset-left-wrist.png` shows the
table and approximately all three utensils. Then run cup, bowl and plate
separately, resetting the arrangement between runs. The grasp tests reset the
arms to the same Shanghai-reference pickup configuration and do not move the
base:

    ./deploy/hamburg/run_hamburg.sh grasp-test --object cup --execute \
      --output /tmp/cup-grasp.json --output-dir /tmp/cup-evidence
    ./deploy/hamburg/run_hamburg.sh grasp-test --object bowl --execute \
      --output /tmp/bowl-grasp.json --output-dir /tmp/bowl-evidence
    ./deploy/hamburg/run_hamburg.sh grasp-test --object plate --execute \
      --output /tmp/plate-grasp.json --output-dir /tmp/plate-evidence

If all three pass, return the robot to the official Hamburg start, then inspect
and run the full loop including base navigation, grasp, letter observation,
placement and return:

    ./deploy/hamburg/run_hamburg.sh mission
    ./deploy/hamburg/run_hamburg.sh mission --execute \
      --output /tmp/hamburg-stage1.json --output-dir /tmp/hamburg-stage1-evidence

Defaults are named Shanghai references. Please send the JSON and images with
Hamburg room/door/table measurements. Use site-config or edit a copied JSON to
change only the failed phase's value; do not scale the whole route.

If a posture move still times out, first retry that same command with a slower
venue override and keep the resulting JSON:

    export TMR_HAMBURG_ARM_POSTURE_VELOCITY_RAD_S=0.05
    ./deploy/hamburg/run_hamburg.sh pickup-reset --execute \
      --output /tmp/pickup-reset.json --output-dir /tmp/pickup-reset-evidence

Do not change the wrist calibration, object descent or route for this symptom.
The report records the resolved speed, following limits, hold/recovery events,
measured joints, encoded GELLO inputs, activation references, publish count and
feedback ages.

If live source topics remain healthy but the JSON shows a process-local joint
feedback age above the default `1.0 s`, preserve that JSON and retry only the
same entrypoint with a measured venue override (maximum accepted value: `5 s`):

    export TMR_HAMBURG_ARM_JOINT_FEEDBACK_STALE_TIMEOUT_S=1.5

Do not use this override for a topic that has actually stopped publishing.

Phase transitions are printed to the terminal and written atomically to the
output directory. Please preserve the terminal output as well as the final JSON;
if recovery or report writing also fails, both the original and secondary errors
are retained.
