# Hamburg trial and troubleshooting

Keep the JSON report and evidence image from every run. The report names the
last completed phase, selected configuration, measured feedback, and exact
failure. Change the parameter related to that phase and repeat the same utensil
before continuing.

The physical entrypoints print every phase to standard error and atomically
replace a progress JSON at each phase transition. A final report is attempted on
success, failure, or operator interruption. Evidence-image write failures are
reported as warnings and do not interrupt otherwise valid motion. Safety faults
still stop the base and prevent the next phase.

| Failure or symptom | Check | Fast, local change |
| --- | --- | --- |
| Environment/preflight failure | Domain 0, `rmw_fastrtps_cpp`, UDP-only profile, native topic types and spine action/service | Correct the sourced Hamburg environment. Do not source Shanghai domain 97/Jazzy launchers. |
| Arm target has `gello` in its name | This is the organizer-confirmed direct JointState input to the Franka impedance controller; no leader device is used | Do not rename the topic unless the organizer reports a different controller endpoint. Override the topic only in a copied grasp JSON. |
| Arm following error or endpoint timeout | Reported arm, maximum error and measured joints; controller update/hold requirements | Lower `motion.maximum_joint_velocity_rad_s` in the copied grasp JSON, or correct joint-name order/topic from organizer feedback. Do not alter IK or descent first. |
| Object not detected or rim point unstable | Saved 640×480 wrist frame, lighting, object placement, occlusion and frame identity | Fix the physical view first. If the approved camera pose changes, recalibrate `target_right_rim_px` and `shanghai_base_xy_to_image_uv`; do not reuse the old pixel mapping by assertion. |
| Visual corrections move away from the target | Per-iteration pixel error and commanded base-XY delta in the report | Re-estimate the two-column wrist Jacobian from small measured X/Y probes. Changing room dimensions cannot fix its sign or scale. |
| Descent reaches limit or misses utensil | Exact object, top pose, tabletop/object height, last joints and clearance | Change only that object's `descent_m` in a copied grasp JSON, initially in small measured increments. Do not apply one table-height delta to all utensils automatically. |
| Close occurs but contact check fails | Empty-close, object-close and after-lift gripper joint vectors | Check utensil position and finger feedback. Adjust `minimum_contact_delta_from_empty_closed` only from repeated empty/object measurements, not to force a pass. |
| Spine action fails | Action name/type, goal rejection/result and measured `GetPosition` value | Correct confirmed action/service names in the interface profile. For a new physical height, set `--spine-m` with `site-config`. |
| Spine Python interface is unavailable | `franka_spine_msgs` import/type-support error before any motion | Source the organizer's Humble workspace that provides `franka_spine_msgs` before running. A container used for physical execution must include or mount the same compatible interface package. |
| Full check fails only on head image | Raw Hamburg topic is fresh 640×360 `bgr8`; `camera_info` matches | Use ZED `pub_resolution: CUSTOM` with downscale factor `2.0`, or change both validation and mission camera configuration to the measured output. Wrist grasp settings stay 640×480. |
| Letter confidence falls after head downsampling | Saved 640×360 evidence image and false proposals | Improve view/lighting first. Then adjust the normalized detector threshold in the mission JSON from venue frames; card centres and drawings already use the resized frame coordinates. |
| Base route fails at a named segment | Segment report, requested/actual odometry, Hamburg door/table measurement and LiDAR range | Modify the matching named segment in the site mission JSON. Room size, door clearance, table standoff and letter spacing are independent; never apply a global scale. |
| Pickup table not found | `approach_pickup_table` travel and front LiDAR clearance | Set the measured maximum approach and front clearance. Check that the front scan's zero-angle cone faces forward. |
| LiDAR hard stop | Reported sensor and range; physical carried envelope and scan mounting | Remove the obstacle or correct the sensor frame/threshold from a measured clear scan. Do not disable feedback freshness. |
| Full mission fails after a successful grasp | The object remains held unless the manipulation recovery completed; base zero is latched | Use the exact failed phase and evidence to update its one parameter. Repeat the relevant standalone grasp if manipulation changed, then restart from the marked physical start. |

The defaults intentionally run the Shanghai settings first. The result should be
described as “Shanghai-reference parameters executed on Hamburg native
interfaces.” It should not be described as venue generalization. A successful
grasp shows that the local pickup view and manipulation happened to remain
compatible; it does not validate Hamburg room or route dimensions. A successful
full trial is the evidence for the selected Hamburg route file.

Create measured overrides without changing the launcher:

```bash
./deploy/hamburg/run_hamburg.sh site-config \
  --mission-output /tmp/hamburg-mission.json \
  --grasp-output /tmp/hamburg-grasp.json \
  --room-length-m MEASURED --room-width-m MEASURED \
  --door-clear-width-m MEASURED --tabletop-height-m MEASURED \
  --initial-forward-m MEASURED --before-door-m MEASURED \
  --through-door-m MEASURED --pickup-front-clearance-m MEASURED
```

Then pass both files to `grasp-test`/`mission` as shown in the README. The
generated mission file lists any Hamburg measurement fields that are still
missing, while keeping each unchanged Shanghai value visible.
