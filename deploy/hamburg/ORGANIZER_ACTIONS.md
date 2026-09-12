# Information requested from the Hamburg organizer

The Task 3 venue document gives the gripper and spine topic names but does not
give their ROS message types, fields, units, or value conventions.  Please do
not change the robot controllers merely for our policy.  We first need the live
interface contract below.

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

## Values that need confirmation

For both gripper topics:

```text
/left/gripper/gripper_client/target_gripper_width_percent
/right/gripper/gripper_client/target_gripper_width_percent
```

please confirm:

1. Exact ROS message type and field name.
2. Whether the range is `0..1` or `0..100`.
3. Whether zero means fully closed and the maximum means fully open.
4. Whether the command is an absolute opening-width target.

For the spine topic:

```text
/spine/target_height
```

please confirm:

1. Exact ROS message type and field name.
2. Whether the command is an absolute target.
3. Whether the unit is metres.
4. Minimum/maximum accepted height and whether it continuously holds the last
   target.

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

As an alternative, the organizer may expose three stable relay topics using
`std_msgs/msg/Float64`:

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
