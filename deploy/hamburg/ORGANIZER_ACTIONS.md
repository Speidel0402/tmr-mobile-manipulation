# Hamburg organizer run sequence

Use the pinned commit on the single Humble companion with the normal domain 0 /
Fast DDS UDP-only environment. Temporary odometry/spine relays are not needed.
Run the native check without the odometry bridge or Float32-to-action relay;
the mission calls MoveAbsolute directly.
Stop manual publishers to the arm, gripper and base command topics during the
autonomous run.

Place the cup, bowl and plate in the normal pickup arrangement. Each grasp test
resets the arms to the Shanghai-reference pickup configuration; the left wrist
camera should then have an approximate view of all three objects. Run cup, bowl
and plate separately, resetting the arrangement between runs. These tests do
not move the base:

    ./deploy/hamburg/run_hamburg.sh check --output /tmp/hamburg-check.json

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

Phase transitions are printed to the terminal and written atomically to the
output directory. Please preserve the terminal output as well as the final JSON;
if recovery or report writing also fails, both the original and secondary errors
are retained.
