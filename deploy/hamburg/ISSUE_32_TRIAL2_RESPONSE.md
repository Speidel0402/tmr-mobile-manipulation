Dear EBiM Task 3 organizers,

Thank you for the precise controller formula and the activation-reference
values. They show that the constant following error was caused by our publisher
treating the Hamburg GELLO input as an absolute robot-joint target. The
following guard was reporting the resulting mismatch correctly.

We have corrected the common Hamburg arm-command layer. As soon as both
measured arm states arrive, before waiting for the wrist camera or gripper, it
publishes each arm's current measured joints as the neutral GELLO input. This
allows the inactive controller to capture matching robot/input references
during activation. Every later absolute robot target is encoded as
`input_at_activation + direction * (target - robot_at_activation)` with the
confirmed direction `[-1,-1,1,1,1,1,-1]`. Absolute robot coordinates are still
used internally for FR3v2 limits, IK, trajectories and following checks.

The encoded hold stream is now maintained continuously by a gap-filling
publisher, including during image processing, IK, evidence writing, the native
spine action/service, and gripper and base phases, without changing the normal
trajectory rate. Reports include both activation references, the last encoded
input, publish age/count, keepalive health and any
synchronization drift. If a controller was already activated against an
incompatible old input and the arm moves during neutral synchronization, the
run stops before the parking trajectory with an explicit
reactivation message.

The same implementation is used by `pickup-reset`, all cup/bowl/plate
`grasp-test` runs and the complete `mission`. The physical run commands and
staged order remain unchanged. Please start each physical entrypoint with the
arm controllers inactive so the normal Hamburg activation occurs while the
neutral stream is present, then retry `pickup-reset --execute`. No speed or
timing override should be needed for the first retry.

If reset passes, please inspect the fresh left-wrist image and continue with the
cup, bowl and plate stationary trials before the full mission. Please send the
JSON and terminal output if activation synchronization or any later phase still
fails.

Thank you again for the detailed diagnosis and your patience with the staged
Hamburg tests.
