# Hamburg organizer run sequence

Use the pinned commit on the single Humble companion with the normal domain 0 /
Fast DDS UDP-only environment. Temporary odometry/spine relays are not needed.
Run the native check without the odometry bridge or Float32-to-action relay;
the mission calls MoveAbsolute directly.
Stop manual publishers to the arm, gripper and base command topics during the
autonomous run.

Run cup, bowl and plate separately so the requested utensil can be placed alone:

    ./deploy/hamburg/run_hamburg.sh check --output /tmp/hamburg-check.json

    ./deploy/hamburg/run_hamburg.sh grasp-test --object cup --execute \
      --output /tmp/cup-grasp.json --output-dir /tmp/cup-evidence
    ./deploy/hamburg/run_hamburg.sh grasp-test --object bowl --execute \
      --output /tmp/bowl-grasp.json --output-dir /tmp/bowl-evidence
    ./deploy/hamburg/run_hamburg.sh grasp-test --object plate --execute \
      --output /tmp/plate-grasp.json --output-dir /tmp/plate-evidence

If all three pass, inspect and run the full loop:

    ./deploy/hamburg/run_hamburg.sh mission
    ./deploy/hamburg/run_hamburg.sh mission --execute \
      --output /tmp/hamburg-stage1.json --output-dir /tmp/hamburg-stage1-evidence

Defaults are named Shanghai references. Please send the JSON and images with
Hamburg room/door/table measurements. Use site-config or edit a copied JSON to
change only the failed phase's value; do not scale the whole route.
