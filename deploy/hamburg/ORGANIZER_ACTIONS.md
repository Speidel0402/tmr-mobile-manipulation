# Information requested from the Hamburg organizer

The Task 3 venue document gives the gripper and spine topic names but does not
give their ROS message types, fields, units, or value conventions.  We now use
the Shanghai-validated scalar contract as the Hamburg default.  Please do not
change the robot controllers merely for our policy; if the live contract is
different, use the overrides below.

## Preferred procedure

Before either FR3 arm is activated, run:

```bash
cd /path/to/tmr-mobile-manipulation
./deploy/hamburg/run_hamburg.sh check \
  --output /tmp/tmr_task3_hamburg_preflight.json
```

Please send us the complete JSON file.  For each command topic it records:

- the discovered ROS message type;
- all top-level message fields;
- the number of controller subscribers;
- the remaining unit/range questions.

The check is read-only, creates one ROS participant, sends no command, and does
not start or restart any robot service.

## Defaults to confirm

For both gripper topics:

```text
/left/gripper/gripper_client/target_gripper_width_percent
/right/gripper/gripper_client/target_gripper_width_percent
```

the submitted Hamburg profile assumes:

1. `std_msgs/msg/Float32`, field `data`.
2. Absolute normalized opening width.
3. `0.0 = closed`, `0.8 = open` (Shanghai convention).

For the spine topic:

```text
/spine/target_height
```

the submitted Hamburg profile assumes:

1. `std_msgs/msg/Float32`, field `data`.
2. Absolute target in metres.
3. Shanghai home target `0.7` m.

The gripper type and value direction are supported by the Shanghai launcher;
the spine scalar message type is a Hamburg default pending the live read-only
check.  The preflight rejects a mismatched live type or missing field before
motion.

## Fast venue correction (no policy change)

Set only the values that differ, then rerun the read-only check:

```bash
export TMR_HAMBURG_GRIPPER_MESSAGE_TYPE=std_msgs/msg/Float32
export TMR_HAMBURG_GRIPPER_FIELD=data
export TMR_HAMBURG_GRIPPER_OPEN=0.8
export TMR_HAMBURG_GRIPPER_CLOSED=0.0
export TMR_HAMBURG_SPINE_MESSAGE_TYPE=std_msgs/msg/Float32
export TMR_HAMBURG_SPINE_FIELD=data
export TMR_HAMBURG_SPINE_HOME_M=0.7
./deploy/hamburg/run_hamburg.sh check \
  --output /tmp/tmr_task3_hamburg_preflight.json
```

Topic names can be changed with `TMR_HAMBURG_LEFT_GRIPPER_TOPIC`,
`TMR_HAMBURG_RIGHT_GRIPPER_TOPIC`, and `TMR_HAMBURG_SPINE_TOPIC`.  All override
names and their JSON paths are listed in
`deploy/hamburg/config/interfaces-shanghai.json`.  A persistent replacement
profile can be supplied with `--interface-config`; this changes only the venue
adapter, not the route, perception, grasp, placement, or return strategy.

If ROS CLI is used instead of the supplied check, it must be run before arm
activation because each CLI command creates a DDS participant.  The equivalent
commands are:

```bash
ros2 topic type /left/gripper/gripper_client/target_gripper_width_percent
ros2 topic type /right/gripper/gripper_client/target_gripper_width_percent
ros2 topic type /spine/target_height
ros2 topic info -v /left/gripper/gripper_client/target_gripper_width_percent
ros2 topic info -v /right/gripper/gripper_client/target_gripper_width_percent
ros2 topic info -v /spine/target_height
```

Then run `ros2 interface show <reported-type>` once for every distinct type.

## If the controller uses custom message packages

Please make the exact Humble/arm64 interface package available in the
organizer-provided environment and source it before starting the team process.
Do not provide a Jazzy-generated package or an x86_64-only binary package.

Only if the native types cannot be made available to the container, the
organizer may expose three stable relay topics using `std_msgs/msg/Float64`:

```text
/tmr/team/left_gripper/target_width_fraction
/tmr/team/right_gripper/target_width_fraction
/tmr/team/spine/target_height_m
```

For this optional relay contract, use `0.0 = closed` and `1.0 = fully open` for
the grippers, and an absolute height in metres for the spine.  Relay nodes must
be started with the robot stack before arm activation and must remain alive for
the complete mission.  The team code must not start or restart them.

## DDS and camera conditions

Please retain the venue-provided Humble/Fast DDS UDP-only environment.  The
team package validates but does not overwrite `ROS_DOMAIN_ID`,
`RMW_IMPLEMENTATION`, or `FASTRTPS_DEFAULT_PROFILES_FILE`.  The head camera is
subscribed at `/head_camera/zed_node/rgb/color/rect/image` with best-effort QoS;
no HTTP camera service is required.
