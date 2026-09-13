Dear EBiM Task 3 organizers,

Thank you for the trial report. It confirms that the native Humble interfaces,
controller activation and spine action are working, and that the blocker is a
transient tracking lag during the right-arm parking move.

We have updated `codex/hamburg-native-preflight` for this controller behavior:

- the normal parking/pickup ramp and phase timing are retained, while posture
  speed is available as a separate venue override if later evidence needs it;
- the hard following limit now has margin above the measured transient;
- only at higher fresh-feedback lag does target advancement pause and hold the
  current target until both arms catch up, while stale feedback,
  excessive error, recovery timeout and final-position timeout still stop the
  run;
- posture speed, pause/resume thresholds, recovery timeout and hard limit are
  exposed as Hamburg environment and site-config overrides;
- each hold, recovery or timeout is printed and included with measured and
  commanded joints in the JSON diagnostics.

This common arm primitive is used by `pickup-reset`, all three `grasp-test`
runs and `mission`, so the fix covers the reported reset failure and the same
failure mode later in the sequence. It does not change wrist resolution,
perception calibration, Cartesian grasp paths, object descents, spine control
or base navigation.

Please update the branch and retry the same reset without additional overrides:

```bash
git checkout codex/hamburg-native-preflight
git pull
./deploy/hamburg/run_hamburg.sh pickup-reset --execute \
  --output /tmp/pickup-reset.json \
  --output-dir /tmp/pickup-reset-evidence
```

If it passes, please confirm that the saved left-wrist image shows the pickup
table and approximately all three utensils, then continue cup, bowl and plate
as previously listed. If it does not pass, please send the new JSON and terminal
output; the added lag events will show whether to lower only the posture speed
or adjust the measured venue limit.
