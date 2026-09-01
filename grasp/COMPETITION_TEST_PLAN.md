# Competition pick-cycle release plan

## Deployment decision

Run the frozen competition workflow on the robot's onboard computer, beside the
camera, MoveIt, arm controller, and gripper controller.  Keep it in a separate
user-space release directory/workspace; do not edit the vendor robot packages,
firmware, or controller source.  SSH/Codex is only the operator console for
start, STOP, telemetry, and log download.  Do not use streamed stdin, `/tmp`
scripts, or a laptop/network link in the real-time execution path during the
competition.

## Required state sequence

`OPEN_VERIFIED -> ALIGNED -> AT_LOW -> CLOSE_ACCEPTED -> GRASP_VERIFIED -> LIFTING -> DONE`

- A close result with `stalled=true, position=0.0` is `NO_MOTION`, never success.
- `ABORTED + stalled` is contact evidence only after measurable closing travel.
- A fully closed gripper is not by itself proof that an object was grasped.
- One bounded open-reset-close retry is allowed at the low pose.
- Internal faults use an explicit logged recovery lift.
- Operator STOP cancels the active action and holds the current pose; it does
  not enter the internal-fault recovery path.

## Test order

1. Offline regression: saved RGB frames, recorded success/failure action results,
   syntax checks, and policy unit tests.
2. Stationary robot: camera freshness, cup/right-rim detection, class diagnostics,
   and live status display only; no movement commands.
3. Motion-disabled integration: mock arm/gripper actions and inject rejected,
   timeout, stale-camera, no-motion-stall, and STOP events at every phase.
4. Hardware staged test: open only, horizontal alignment only, vertical movement
   only, gripper only, then the complete cycle at reduced speed.
5. Repeatability: at least 30 complete cup relocations; record alignment error,
   descent residual, close travel, result class, return-height residual, and
   total cycle time.
6. Freeze: versioned release folder, configuration snapshot, dependency list,
   SHA-256 manifest, one launch command, one STOP command, and rollback release.

## Release acceptance

- Zero false-success results for `NO_MOTION` and fully-closed-empty cases.
- Every action timeout is cancelled before another stage can start.
- Cup detection uses three fresh frames and remains independent of bowl/plate
  confidence when the cup itself is unique and high-confidence.
- Small controller-handoff rebound is automatically restored/rebased; only a
  gross pose mismatch or real controller/planning fault blocks execution.
- STOP is fault-injected successfully at every state without an unexpected lift.
