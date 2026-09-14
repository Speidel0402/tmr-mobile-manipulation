Dear EBiM Task 3 organizers,

Thank you for your patience, the seven repeated trials, and the detailed timing
comparison. We reviewed the latest failure together with the earlier following
error and relative GELLO feedback, and updated `codex/hamburg-native-preflight`.

Your evidence points to delayed callback servicing inside our process. We found
a concrete startup path where the first wrist-image decode could spend time
loading OpenCV after neutral streaming had begun, followed immediately by a
freshness check without processing queued feedback. OpenCV is now loaded before
neutral streaming, and feedback is refreshed after decoding. This is consistent
with the reported symptom, but we cannot confirm it as the sole cause without
another Hamburg run.

The shared reset/grasp/mission controller also now:

- services incoming callbacks in bounded batches while maintaining the arm
  stream; the feedback-age limit is configurable and defaults to `1.0 s`, so the
  reported `0.501–0.613 s` ages alone no longer trip a hardcoded `0.5 s` guard;
- defaults this process to asynchronous Fast DDS publication before ROS
  initialization, while preserving explicit venue settings and reporting when
  XML QoS takes precedence;
- waits for each ramp target to be submitted to the local publisher before
  advancing if publication is delayed, and monitors publication continuity
  during the later phases as well;
- processes the newest camera frame instead of a backlog, checks arm feedback
  during base motion, and cancels an accepted spine action when its wait fails
  or is interrupted;
- records per-arm publication timing, input-service gaps, startup decode timing,
  the failed phase, and original plus recovery/cleanup errors.

The original Shanghai poses, wrist calibration, object parameters, relative
GELLO direction map and normal motion speeds remain unchanged. Neutral streaming
still precedes controller activation. Old copied configuration files remain
supported, and plans now resolve the same explicit overrides as execution.

We also restored executable permission on `run_hamburg.sh`. In each new terminal,
please source `/opt/ros/humble/setup.bash`, then the Hamburg testbed workspace
overlay providing `franka_spine_msgs`. A missing Python package in that shell
does not mean the running native spine interface is absent.

Please update the branch and keep the same run order. With the arm controllers
initially inactive, manually place the stopped robot beside the pickup table:

```bash
./deploy/hamburg/run_hamburg.sh check --output /tmp/hamburg-check.json
./deploy/hamburg/run_hamburg.sh pickup-reset --execute \
  --output /tmp/pickup-reset.json --output-dir /tmp/pickup-reset-evidence
```

Reset does not move the base. Please confirm the saved left-wrist image shows
the table and approximately all three utensils, then run `grasp-test --object cup --execute`
for the cup, `grasp-test --object bowl --execute` for the bowl, and
`grasp-test --object plate --execute` for the plate, resetting the arrangement
between runs. These are stationary observation–grasp–release tests. After all
three pass, start `mission --execute` from the official Hamburg start; it
includes base motion and delivery **cup → B, bowl → A, plate → D**. Full commands
and troubleshooting are in
[ORGANIZER_ACTIONS.md](https://github.com/Speidel0402/tmr-mobile-manipulation/blob/codex/hamburg-native-preflight/deploy/hamburg/ORGANIZER_ACTIONS.md).

The full route still starts from Shanghai reference values. Hamburg room
length/width, doorway dimensions, table height and position, and route distances
must be checked independently; these are substantial physical differences and
cannot be handled by assuming generalization or applying one global scale.
The [Shanghai test video](https://github.com/Speidel0402/tmr-mobile-manipulation/releases/download/stage1-pre-submission/ebim-task3-phase2-stage1-official-test.mp4)
shows the original placement arrangement.

Local validation passed 67 Hamburg regression tests and 229 existing base,
grasp and mission tests, plus Python compilation and lint checks. These cover
delayed decoding/publication, activation sync, configuration compatibility and
exception handling. Hamburg hardware execution
still needs your retry. Please send the reset JSON and wrist image first; if it
stops, the additional timing diagnostics should help isolate the remaining cause.
