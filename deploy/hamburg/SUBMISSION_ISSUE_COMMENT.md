Dear EBiM Task 3 organizers,

Thank you for providing the Hamburg preflight and interface information. We
have prepared native ROS 2 Humble entrypoints for two staged trials. We suggest
running the stationary observation-to-grasp checks first and proceeding to the
complete mission only after all three utensils pass.

## 1. Stationary observation-to-grasp checks

Please place the cup, food bowl and plate in the normal pickup arrangement.
Before observation, each grasp test resets the arms to the pickup configuration
used for the Shanghai trial. From this reset pose, the left wrist camera should
have an approximate view of all three objects. Each command below observes and
grasps only the selected utensil. This test does not move the mobile base.

```bash
./deploy/hamburg/run_hamburg.sh check \
  --output /tmp/hamburg-check.json

./deploy/hamburg/run_hamburg.sh grasp-test --object cup --execute \
  --output /tmp/cup-grasp.json --output-dir /tmp/cup-evidence
./deploy/hamburg/run_hamburg.sh grasp-test --object bowl --execute \
  --output /tmp/bowl-grasp.json --output-dir /tmp/bowl-evidence
./deploy/hamburg/run_hamburg.sh grasp-test --object plate --execute \
  --output /tmp/plate-grasp.json --output-dir /tmp/plate-evidence
```

Please reset the utensil arrangement between runs. If a trial fails, please
send its JSON report and saved wrist image; these identify whether the problem
is the reset view, perception, arm response or gripper feedback.

## 2. Complete closed-loop mission

Once all three stationary grasp checks pass, please return the robot to the
official Hamburg starting position. First inspect the resolved mission plan,
then start the physical run:

```bash
./deploy/hamburg/run_hamburg.sh mission
./deploy/hamburg/run_hamburg.sh mission --execute \
  --output /tmp/hamburg-stage1.json \
  --output-dir /tmp/hamburg-stage1-evidence
```

The complete mission includes base navigation from the official start, pickup
observation and grasp, head-camera letter observation, delivery and placement,
return for the next utensil, and the final stop after the plate.

For reference, the pickup and placement arrangement used during the Shanghai
trial can be seen in the complete test video:

https://github.com/Speidel0402/tmr-mobile-manipulation/releases/download/stage1-pre-submission/ebim-task3-phase2-stage1-official-test.mp4

The initial Hamburg run reuses settings demonstrated in Shanghai where they are
applicable. This is a starting reference rather than a claim that the two venues
generalize. Hamburg room dimensions, doorway, table height and pose, official
start, travel route and letter-station layout may all differ from Shanghai and
must be checked independently.

If the stationary grasps pass but the complete mission fails, please send the
mission JSON, evidence images and the relevant Hamburg room/table/door
measurements. We can then adjust the affected mission phase directly instead of
applying one global scale factor or changing the already working grasp setup.
