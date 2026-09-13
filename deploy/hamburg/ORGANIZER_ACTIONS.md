# Hamburg organizer follow-up

Please rerun `./deploy/hamburg/run_hamburg.sh check --output /tmp/tmr_task3_hamburg_preflight.json`
on the single Humble companion **without** the temporary odometry, spine-state,
or Float32-to-action relay nodes. Source the Humble workspace containing
`franka_spine_msgs` before starting the team process. Keep the established
ROS domain 0, Fast DDS UDP-only profile, and existing robot drivers.

The new report should show direct `/swerve_drive_controller/odom` samples,
`/franka_spine_node/get_position` readiness and a successful position query,
and `/franka_spine_node/move_absolute` action readiness. Please confirm the
native action name and the `MoveAbsolute` goal/result fields if they differ
from the Shanghai package (`position`, `velocity`, `acceleration`,
`deceleration`; result `success`, `error`, `stop_by`). The check sends no goal.

Please also run `grasp-check --object cup`, then bowl and plate, with each
object at the normal pickup area and both arms already in the
configured Shanghai-reference pickup/parking posture **only if the different
Hamburg table and room provide safe clearance**. For a common stationary view,
`grasp-check-all /tmp/hamburg_grasps` saves three JSON reports and three
annotated wrist-frame PNGs in that order when frames are available. This
check is read-only. It reports
joint-position error, spine height, RGB frame identity, and stability of the
existing object-specific detector; it does not close a gripper or certify a
grasp. It uses only the left wrist RGB camera, so the head ZED setting does not
gate this standalone observation. Please send all three JSON reports and PNGs.

Hamburg room length/width, door clear width, tabletop size/height and the
base-to-table pickup distance may differ from Shanghai. Please fill a copy of
`config/geometry-observations.json` and run `geometry-review --input <file>`;
the script reports missing measurements and never commands motion. We propose
using the Shanghai 0.7 m spine and object-specific descent values as first
**review references**, then changing the implicated target only after a
measured, supervised object trial. No uniform scaling of the Shanghai room
route is valid.

For the head camera, please publish 640×360 `bgr8` and report both image and
`camera_info` dimensions. In the current Stereolabs ROS 2 wrapper, 2x
downscaling requires `general.pub_resolution: CUSTOM` as well as
`general.pub_downscale_factor: 2.0`; setting the factor alone under `NATIVE`
does not change the published resolution. The camera configuration must remain
stable during the trial.

Before any physical cup, bowl, plate, or full Stage 1 test, we also need the
Gello joint-target contract (joint order, update frequency, hold behavior,
limits and fault response), the base command frame/watchdog behavior, and the
venue's accepted calibration/start pose. A read-only topic subscriber count
cannot establish motion semantics. Please do not run the Shanghai standalone
grasp scripts on Hamburg: they still use unavailable motion interfaces and
create additional ROS participants.
The physical Hamburg grasp entrypoint is still locked; please treat the three
`grasp-check` reports as the first diagnostic step and send their failures,
geometry measurements, Gello command contract and gripper feedback so we can
prepare a separate physical cup/bowl/plate runner before the full mission.
