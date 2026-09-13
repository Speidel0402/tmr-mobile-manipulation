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

For the head camera, please set the ZED's `pub_downscale_factor:=2.0` before
running the check, or report the camera configuration that will remain stable
during the trial. The submitted visual policy expects 640×360 `bgr8`.

Before any physical cup, bowl, plate, or full Stage 1 test, we also need the
Gello joint-target contract (joint order, update frequency, hold behavior,
limits and fault response), the base command frame/watchdog behavior, and the
venue's accepted calibration/start pose. A read-only topic subscriber count
cannot establish motion semantics. Please do not run the Shanghai standalone
grasp scripts on Hamburg: they still use unavailable motion interfaces and
create additional ROS participants.
