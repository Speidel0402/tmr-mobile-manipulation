# Hamburg grasp-first trial and fast adjustment guide

Run the three object checks before any navigation or full Stage 1 attempt. The
current Hamburg `grasp-check` is **read-only**: it proves live native interfaces,
the measured stationary pose, and object visibility, but does not close or move
the gripper. `grasp-test` and `mission` remain locked because the Shanghai
motion scripts require interfaces not established on this Humble testbed.

## First pass: Shanghai reference, one object at a time

1. Measure and record the Hamburg room, clear door opening, table top above
   floor, tabletop dimensions, and base-to-table-edge distance. Fill a copy of
   `config/geometry-observations.json`. The Shanghai route and 0.7 m spine/
   0.340/0.360/0.375 m object descent values are references, not an automatic
   height correction. Run:

   ```bash
   ./deploy/hamburg/run_hamburg.sh geometry-review \
     --input /tmp/hamburg_geometry.json --output /tmp/hamburg_geometry_review.json
   ```

2. The organizer confirms that the robot can be held stationary in the
   Shanghai reference pickup/parking pose without touching the Hamburg table,
   objects, walls or door. Keep the standard robot drivers and venue DDS setup;
   stop temporary relay nodes. Run native `check` and keep its JSON:

   ```bash
   ./deploy/hamburg/run_hamburg.sh check --output /tmp/hamburg_native_check.json
   ```

   A head-ZED resolution failure in this full-interface check does not prevent
   the wrist-only grasp observation below. It does block a claim that the full
   venue profile is ready.

3. With cup, bowl and plate visible from the approved stationary left-wrist
   view, run all three checks in Shanghai object order, saving separate reports:

   ```bash
   ./deploy/hamburg/run_hamburg.sh grasp-check-all /tmp/hamburg_grasps
   ```

   The script runs cup, bowl and plate sequentially even if one is blocked.
   Each object gets a JSON report and an annotated real wrist-camera PNG when
   a decodable frame was received. Inspect the marked rim point and visible
   table clearance, especially before accepting a changed camera pose.
   For a new pose or separate object placement, use `grasp-check --object NAME`
   individually. Do not interpret `observation_ready` as a completed grasp.

4. Once native interfaces, room/table clearance and three observation reports
   are reviewed, port and supervise a **physical** Hamburg cup trial, then bowl,
   then plate. Require actual command/feedback evidence for each object. Only
   after all three physical grasps succeed should the team trial the full
   route, letter search and placement sequence. The repository does not yet
   contain that physical Humble runner.

## Quick adjustments after a failed report

| Symptom | Check and minimal adjustment |
| --- | --- |
| `ROS_DOMAIN_ID`, RMW or UDP profile mismatch | Use the organizer's domain 0/Fast DDS setup; do not source Shanghai domain 97 or Jazzy scripts. |
| `temporary relay topic still present` | Stop the organizer's odom/spine relays before judging native readiness. |
| Spine action/service unavailable | Confirm the actual action name and `GetPosition` service type; override the confirmed action name with `TMR_HAMBURG_SPINE_ACTION_NAME` and service name with `TMR_HAMBURG_SPINE_POSITION_SERVICE`. This changes lookup, not motion. |
| Spine differs from Shanghai 0.7 m | Record the actual position and measured table height first. `TMR_HAMBURG_SPINE_HOME_M` can change the **comparison target** for a new check; it does not move the spine. Accept a new target only after clearance review. |
| Joint posture mismatch | Confirm joint names/order and table clearance. To compare with a newly approved stationary observation pose, copy `config/grasp-observation.json`, edit the seven measured target joints, then pass `--posture-config /tmp/approved_pose.json` or set `TMR_HAMBURG_GRASP_POSTURE_CONFIG`. This does not move either arm. |
| Wrist frame missing, wrong size, or unstable detection | Confirm `/wrist_camera_left/camera/color/image_raw` is fresh 640×480 RGB, object placement, lighting and camera frame; send the object-specific report and frame. The head ZED downscale is not a grasp-check requirement. |
| Full `check` fails only on head image | Confirm the raw Hamburg ZED topic and 640×360 `bgr8`; current wrapper needs `general.pub_resolution: CUSTOM` with `general.pub_downscale_factor: 2.0`. Check `camera_info` and actual image. |
| Room/door/table does not match Shanghai | Measure local map, door clear width, table top and pickup pose. Review `base/config/start_to_pickup.yaml`, `base/config/route.yaml`, and mission placement offsets. Do not rescale all distances by one room-size ratio or reuse Shanghai descents as a table-height correction. |

The geometry script identifies missing measurements and reports any supplied
Shanghai/Hamburg table-top difference. It intentionally generates no motion
target. If a physical trial fails, send its exact phase, controller error,
measured joints/spine position, fresh images and room/table measurements; then
change only the implicated interface or parameter and repeat that object before
advancing to the next one.
