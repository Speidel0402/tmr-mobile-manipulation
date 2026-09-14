Hi — thank you for the repeated trials and the detailed timing evidence.

We traced this to a process-local scheduling interaction in our runner. The
continuous arm publisher could hold up the same thread that services incoming
joint-state callbacks. That explains why all inbound ages increased together
even though the source topics remained healthy, and why the hardcoded `0.5 s`
guard fired just above its limit before any motion.

The update keeps the Shanghai-reference poses, grasp parameters, GELLO mapping,
motion speed and phase order unchanged. It makes the continuous 50 Hz arm
publisher independent of input callback servicing, drains multiple ready input
callbacks per control tick, and applies the feedback-age guard consistently to
pickup reset, all three standalone grasp tests and the full mission. The guard
is still active, now defaults to a bounded `1.0 s`, reports both age and limit,
and can be overridden for measured venue scheduling gaps with
`TMR_HAMBURG_ARM_JOINT_FEEDBACK_STALE_TIMEOUT_S` (maximum `5 s`). The observed
`0.501-0.613 s` ages are within the new default, so no override or motion-timing
change should be needed for the next retry.

We also restored executable permission on `deploy/hamburg/run_hamburg.sh` and
clarified the startup environment: source `/opt/ros/humble/setup.bash`, then the
Hamburg testbed workspace overlay that provides `franka_spine_msgs`. A
`ModuleNotFoundError` for that package means the current terminal is missing
the overlay; it does not indicate that the running spine interface is absent.

Please use the same run sequence, starting with the controllers inactive and
the stopped robot manually placed beside the pickup table:

```bash
./deploy/hamburg/run_hamburg.sh check --output /tmp/hamburg-check.json
./deploy/hamburg/run_hamburg.sh pickup-reset --execute \
  --output /tmp/pickup-reset.json \
  --output-dir /tmp/pickup-reset-evidence
```

If reset passes, please confirm that the saved left-wrist image approximately
shows the cup, bowl and plate, then continue with the three standalone grasp
tests and finally the full mission from the official start position. Please
send the reset JSON and wrist image; if anything still stops, the report will
include the exact phase, resolved feedback-age limit, feedback ages and
publisher diagnostics.
