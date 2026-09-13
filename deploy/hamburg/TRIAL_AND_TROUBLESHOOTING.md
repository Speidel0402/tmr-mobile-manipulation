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
| Pickup-reset image does not show the table or all three utensils | Manual base position and heading, table edge, table height, occlusion and clear arm workspace | Reposition the stopped base beside the table and rerun `pickup-reset`. If measured table height still leaves the vertical view wrong, create a copied grasp config with `site-config --spine-m` and pass it to `pickup-reset --config`; keep the wrist mapping and grasp descent unchanged until the view is confirmed. |
| Arm target has `gello` in its name | This is the organizer-confirmed relative, activation-referenced and direction-mapped JointState input to the Franka impedance controller; no leader device is used | Do not rename the topic unless the organizer reports a different controller endpoint. Override the topic only in a copied grasp JSON. |
| Arm holds or moves in the wrong direction immediately after activation | Check `arm_command_sync`, robot/input activation references, direction vector, and whether the controller was already active before neutral samples arrived | Start the entrypoint with the controller inactive so it captures the continuously published measured-joint neutral input. Hamburg uses relative GELLO semantics with directions `[-1,-1,1,1,1,1,-1]`; do not publish absolute robot targets directly to `/gello/joint_states`. |
| Arm controller stops during spine, image processing, IK, evidence writing, gripper or base work | Check `arm_publish_count`, `last_published_arm_input`, `last_arm_publish_age_s`, `arm_keepalive_thread_alive`, `arm_keepalive_error`, and spine `arm_keepalive_count` | The normal control loop remains the primary publisher; a dedicated publisher fills computation gaps without doubling the configured trajectory rate. Send the JSON if the controller still reports stale input. |
| Arm lag hold, following error or endpoint timeout | Reported lag/recovery events, arm, maximum error, measured/commanded joints and feedback age | The runner now freezes the current target at transient lag and resumes only after both arms catch up. For repeated posture lag, lower `motion.posture_joint_velocity_rad_s` or set `TMR_HAMBURG_ARM_POSTURE_VELOCITY_RAD_S`; change the hard limit only from measured venue evidence. Persistent lag, stale feedback and hard-limit errors still abort. Do not alter IK, wrist calibration or descent for this symptom. |
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

Arm motion values are also independent venue overrides. The current defaults
retain the original `0.06 rad/s` posture ramp, hold target advancement only at
`0.20 rad` lag, resume below `0.14 rad`, abort above `0.24 rad`, and allow `5 s`
for recovery.
They specifically address the fresh-feedback Hamburg trial in which the right
controller reached `0.1603 rad` transient lag. They do not change the Cartesian
waypoints, wrist-camera resolution, Shanghai pixel mapping, or object descents.

Then pass both files to `grasp-test`/`mission` as shown in the README. The
generated mission file lists any Hamburg measurement fields that are still
missing, while keeping each unchanged Shanghai value visible.
