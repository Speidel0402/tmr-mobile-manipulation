# EBiM Task 3 — Hamburg deployment

This directory provides native Hamburg entrypoints for the organizer's single
aarch64 Ubuntu 22.04 / ROS 2 Humble companion. It does not start robot drivers,
change the DDS domain, use SSH, or require the organizer's temporary bridges.

The trial order is deliberately staged:

1. native interface check;
2. manual base placement beside the pickup table, followed by the arm/spine
   pickup-view reset and a fresh left-wrist review image;
3. stationary wrist observation for cup, bowl, then plate;
4. autonomous observation → visual correction → grasp → lift → contact check →
   release for each utensil;
5. the one-node cup → B, bowl → A, plate → D mission.

`pickup-reset`, `grasp-test` and `mission` are runnable. Without `--execute`
they print the resolved plan; with `--execute` they use the physical interfaces.

The Hamburg arm controller may track a long parking move more slowly than a
short Cartesian grasp move. Posture speed therefore has its own venue override,
while the default remains the original `0.06 rad/s` ramp. The hard following
guard now has margin above the measured Hamburg transient. Only if lag reaches
a higher fallback threshold does the node freeze the current target until both
arms catch up. Feedback that exceeds the hard limit, becomes stale, or does not
recover still aborts the phase. The same behavior is used by `pickup-reset`,
every standalone grasp, and the full mission.

## Native control path

| Component | Hamburg path used by these entrypoints |
| --- | --- |
| Arm state | `/{left,right}/franka_robot_state_broadcaster/measured_joint_states` |
| Arm command | `/{left,right}/gello/joint_states`, relative activation-referenced `sensor_msgs/JointState`, direction `[-1,-1,1,1,1,1,-1]` |
| Gripper | native Float32 width target, `0.8=open`, `0.0=closed`, plus gripper joint feedback |
| Spine | `/franka_spine_node/move_absolute` and `/franka_spine_node/get_position` |
| Base | direct `/swerve_drive_controller/odom` and `/swerve_drive_controller/cmd_vel` |
| Wrist RGB | raw 640×480 left camera; grasp observation and alignment |
| Head RGB | raw 640×360 ZED; letter recognition and centering in the full mission |
| Range | front and rear native LaserScan streams |

The arm command topics contain `gello` because Hamburg's deployed
joint-impedance controller implements the GELLO teleoperation convention. The
input is not an absolute robot target. At controller activation it captures a
robot reference and an input reference, then applies directions
`[-1,-1,1,1,1,1,-1]`. The Hamburg runner therefore keeps every trajectory,
joint-limit check and following-error check in absolute robot coordinates, and
encodes only the outgoing message as
`input_zero + direction * (robot_target - robot_zero)`.

No physical GELLO leader or manual teleoperation is used. Before any requested
arm motion, the runner continuously publishes the current measured joints as a
neutral input. Start the physical entrypoint with the arm controllers inactive
or ready to activate so they capture that neutral sample. The same input is
kept alive while the spine action, cameras, gripper and base loops run; Hamburg
reports that a stale GELLO stream zeroes torque and stops the controller. If an
arm moves during neutral synchronization, the run stops before the parking
trajectory and reports that the controller must be reactivated with the neutral
stream present.

The official FR3v2 kinematic chain and joint limits are applied locally, so the
Hamburg path does not depend on MoveIt, PTP, IK/FK services, or Robotiq actions.

Each physical entrypoint creates one ROS node and
constructs all subscriptions, publishers, action clients, and service clients
before active motion. They keep publishing arm hold targets while the base,
vision, and gripper loops run. No phase starts a child process or creates a new
DDS participant.

## Camera resolution and coordinates

The wrist camera remains 640×480. Head-ZED downsampling does not change the
wrist detector target, wrist visual Jacobian, or grasp alignment coordinates.

Hamburg head RGB is expected at 640×360 `bgr8` on
`/head_camera/zed_node/rgb/color/rect/image`. Configure the ZED wrapper with
`general.pub_resolution: CUSTOM` and `general.pub_downscale_factor: 2.0`; verify
the live image and `camera_info` dimensions. The letter detector uses normalized
centres and rescales its card-size thresholds from the current image width, and
annotations are drawn in the current frame. A pure 1280×720 → 640×360 resize
therefore needs no manual halving of stored letter centres. Lower resolution can
still reduce glyph confidence, so keep the saved evidence frames from the test.

## Shanghai references and Hamburg dimensions

`config/grasp-cycle-shanghai-reference.json` intentionally reuses the Shanghai
pickup/parking posture, wrist target and visual Jacobian, spine `0.7 m`, and
object descents (`cup=0.340`, `bowl=0.360`, `plate=0.375 m`) for the first trial.

`config/mission-shanghai-reference.json` contains individually named Shanghai
route values. These are starting values, not a generalized venue model. Hamburg
room length/width, door position and clear width, tabletop height/size,
base-to-table standoff, letter positions, and every route segment are independent
physical quantities. Do not multiply all Shanghai distances by a room-size
ratio. A table-height difference also does not by itself determine spine height
or grasp descent: camera view, tool clearance, object height, and arm reach must
be considered separately.

The mission report always includes the selected parameter profile, the unchanged
Shanghai values, and the Hamburg measurement fields. Use `site-config` to make
an explicit local file without editing entrypoint code:

```bash
./deploy/hamburg/run_hamburg.sh site-config \
  --mission-output /tmp/hamburg-mission.json \
  --grasp-output /tmp/hamburg-grasp.json
```

Add the measured fields when they are available. The command accepts
`--initial-forward-m`, `--before-door-m`, `--through-door-m`,
`--pickup-approach-maximum-m`, `--pickup-front-clearance-m`, `--spine-m`, and
the three object-specific `--*-descent-m` options. Fields not supplied remain
clearly identified Shanghai references.

The site-config command also accepts `--posture-velocity-rad-s`,
`--maximum-following-error-rad`, `--following-error-pause-rad`,
`--following-error-resume-rad`, and
`--following-error-recovery-timeout-s`. For a quick venue-only trial, the same
values can be supplied without editing a file:

```bash
export TMR_HAMBURG_ARM_POSTURE_VELOCITY_RAD_S=0.05
```

The relaxed default guard should be tried first, with the original motion
timing. If the controller remains slower, lower the posture velocity before
increasing the hard limit again. Every
applied environment override and every lag/recovery event is included in the
JSON report.

## Run order

Use the organizer's already sourced domain 0 / Fast DDS UDP-only environment.
That sourced Humble workspace must expose the organizer's
`franka_spine_msgs/action/MoveAbsolute` and `franka_spine_msgs/srv/GetPosition`
types; the native preflight checks this before any motion.
First run the native check without the temporary odometry or spine relays:

```bash
./deploy/hamburg/run_hamburg.sh check \
  --output /tmp/tmr_task3_hamburg_preflight.json
```

Next, manually place the stopped mobile base beside the pickup table with clear
arm workspace and arrange all three utensils as in the pickup trial. Inspect the
plan, then execute the reset:

```bash
./deploy/hamburg/run_hamburg.sh pickup-reset
./deploy/hamburg/run_hamburg.sh pickup-reset --execute \
  --output /tmp/pickup-reset.json \
  --output-dir /tmp/pickup-reset-evidence
```

This entrypoint does not create a base command publisher and never moves the
base. It sets the spine and both arms to the Shanghai-reference pickup view,
opens the left gripper, then saves a newly received 640×480 frame as
`/tmp/pickup-reset-evidence/pickup-reset-left-wrist.png`. Continue when that
frame shows the pickup table and approximately all three utensils. If it does
not, correct the manual base placement and view before changing grasp values.

Confirm each selected utensil in the stationary wrist view:

```bash
./deploy/hamburg/run_hamburg.sh grasp-check --object cup \
  --output /tmp/cup-observation.json --image-output /tmp/cup-observation.png
./deploy/hamburg/run_hamburg.sh grasp-check --object bowl \
  --output /tmp/bowl-observation.json --image-output /tmp/bowl-observation.png
./deploy/hamburg/run_hamburg.sh grasp-check --object plate \
  --output /tmp/plate-observation.json --image-output /tmp/plate-observation.png
```

Then run one physical closed-loop trial at a time. Each successful trial returns
the utensil to its pickup position, opens the gripper, and finishes above it:

```bash
./deploy/hamburg/run_hamburg.sh grasp-test --object cup --execute \
  --output /tmp/cup-grasp.json --output-dir /tmp/cup-grasp-evidence
./deploy/hamburg/run_hamburg.sh grasp-test --object bowl --execute \
  --output /tmp/bowl-grasp.json --output-dir /tmp/bowl-grasp-evidence
./deploy/hamburg/run_hamburg.sh grasp-test --object plate --execute \
  --output /tmp/plate-grasp.json --output-dir /tmp/plate-grasp-evidence
```

The pass condition requires a stable rim detection, visual alignment, completed
descent/lift, and gripper feedback that differs from the empty-close baseline
both before and after lift. A close command alone is not reported as a grasp.
Each phase is also printed to the terminal and saved in a progress JSON under
`--output-dir`; the final report includes primary, recovery, cleanup, and live
state diagnostics when available.

Inspect the full plan, then run it with the chosen files:

```bash
./deploy/hamburg/run_hamburg.sh mission \
  --config /tmp/hamburg-mission.json --grasp-config /tmp/hamburg-grasp.json

./deploy/hamburg/run_hamburg.sh mission --execute \
  --config /tmp/hamburg-mission.json --grasp-config /tmp/hamburg-grasp.json \
  --output /tmp/hamburg-stage1.json --output-dir /tmp/hamburg-stage1-evidence
```

The full mission reads native odometry, checks fresh dual LiDAR and head frames,
stops on stale feedback or the configured LiDAR hard range, visually centres
each assigned letter, uses the measured lateral search distance for the return,
and latches a zero base command on completion or failure.

See `TRIAL_AND_TROUBLESHOOTING.md` for phase-specific changes and
`INTEGRATION_AUDIT.md` for the interface and organizer-glue audit.

## Offline verification

```bash
python3 -m unittest discover -s deploy/hamburg/tests -v
python3 -m compileall -q deploy/hamburg
./deploy/hamburg/run_hamburg.sh pickup-reset
./deploy/hamburg/run_hamburg.sh grasp-test --object cup
./deploy/hamburg/run_hamburg.sh mission
```
