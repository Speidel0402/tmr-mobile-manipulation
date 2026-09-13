# Hamburg compatibility

The existing Shanghai implementation is preserved. Hamburg uses native
entrypoints under this directory because the venue has one ROS 2 Humble
companion and no documented MoveIt/PTP path.

| Component | Hamburg path |
| --- | --- |
| Arm | absolute robot targets encoded for the organizer-confirmed relative, direction-mapped JointState inputs; no GELLO leader |
| Gripper | Float32 width command plus joint-state contact/retention evidence |
| Spine | native MoveAbsolute action plus GetPosition verification |
| Base | direct swerve Odometry and TwistStamped |
| Cameras | 640×480 raw wrist for grasp; 640×360 raw head ZED for letters |
| Runtime | one node; no SSH, child phase processes, MoveIt/PTP, or relay topics |

The deployed arm topics retain “gello” in their names, but the Hamburg scripts
do not use a leader device. They publish a measured-joint neutral stream before
controller activation, then encode autonomous targets against the captured
activation references. A physical GELLO publisher must be stopped before
autonomous execution so there is one command owner.

The organizer glue is unnecessary for the new path. Odometry/state relays are
additive but can add traffic. The Float32-to-action spine relay creates a second
command path and should be stopped. Head-ZED downsampling changes that stream
for all consumers; the new letter path uses normalized centres, while the
separate 640×480 wrist calibration is unchanged.

Shanghai posture, wrist calibration, spine height, object descents and route
segments are retained only as first-trial references. Hamburg room, door,
table and letter geometry must be changed independently, never by one venue
scale factor.
