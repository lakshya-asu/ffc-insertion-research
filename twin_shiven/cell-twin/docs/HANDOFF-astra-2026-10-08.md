# Handoff to Astra: the YAM insertion twin, where it stands, and what the NUC side has to do

Written 2026-10-08 by the Mac-side agent for Astra (the agent on the NUC) and for Shiven. Everything below is
what we believe to be true and the evidence for it. Nothing in it is verified against the cell by anyone who can
touch the cell. That verification is your first job. Please do not take any claim here on trust.

How to use this document, in order:

1. Read it all. Then read the four files it points at under "Files to read".
2. Verify each item in "Claims to verify on the cell" and write down, per item, VERIFIED / WRONG (with the correct
   value) / CANNOT CHECK (with what you would need).
3. Before doing any of the build work in "Work for the NUC side", write Shiven a plain-words plan of how YOU would do
   it, step by step, including where you disagree with this handoff and why. He will take it from there. Keep that
   plan free of code and of internal names; one page; no em dashes anywhere in anything you write for him.
4. Only then build, in the order given, and only on the NUC. Never change the running cell's configuration,
   calibrations or station files without Shiven saying so in that session.

## 1. What the project is

The cell: two i2rt YAM arms facing each other across a table, stock linear_4310 grippers with the triangular nylon
fingers, an Intel D405 on an L bracket above each gripper looking down the fingers, a D435 scene camera above the
mating point, an Arducam on the table for pickups. The left arm holds the wired female 5-pin 2.54 mm housing
(labelled 5V RX TX GND RES), the right arm holds the male plug housing with five pins protruding 6 mm. The task is
to mate them. Pickups work. ACT (LeRobot, three cameras, 12-D state) does the approach and fails the last
millimetres: the male housing arrives 1 to 5 mm off laterally and 1 to 8 degrees tilted because of how it was picked,
and the pins miss, jam or seat partially.

The plan: keep ACT for everything it does well; add a sensor-only contact-phase stage trained in a MuJoCo twin of
the cell; first as a micro-alignment stage run from the preinsertion2 pose BEFORE ACT (touch the housing, null the
wrist moment from the pick tilt, centre across the row, back off 3 mm, hand over), later as a contact policy that
takes over FROM ACT at the first torque rise. Policies read only what the cell measures: joint encoders, motor
torque, their own commands. Simulator truth is used only to grade results offline.

## 2. What exists on the Mac (repo ~/Professional/cell-twin, git, main branch)

- `cell_twin/scene.py`: the twin. Table, two arms (vendored Menagerie i2rt YAM with linear_4310 grippers), real
  controller gains (MIT mode kp 80/80/80/10/10/10, kd 5/5/5/1.5/1.5/1.5, gravity feedforward kept), both connector
  halves built from datasheet dimensions (housing 13.36 x 2.54 x 14 mm, pins 0.64 mm square, 6 mm, cavities 1.2 mm
  with 0.4 mm chamfer, contact plates calibrated so a square push seats with the real ~3 Nm rise on right joint 2),
  rigid grasp (pads on the housing end faces, weld), three cameras placed by registration against real frames.
- `cell_twin/layout.py`: base placement and a gripper mount roll folded into joint 6 (see claims 4 and 5).
- `cell_twin/bench.py`, `controllers.py`: the insertion bench (reset at the real seated state backed off 10 mm with a
  sampled pick error, closed-loop tool servo with the real gains, torque sensing with scale, bias, measured noise,
  low-pass and latency, randomisation per episode), the scripted sensor-only expert, a seat detector (being rebuilt).
- `cell_twin/env.py`, `episode_writer.py`, `datasets.py`, `models.py`, `scripts/collect_expert.py`,
  `label_contact.py`, `train_bc.py`, `train_classifier.py`, `eval_policy.py`: the gym-style environment with a
  sensor-only observation (16-tick window of tool pose relative to contact onset, external torque, tracking error,
  last action, time) and a hand-built reward, the episode writer (sensor record plus a separate truth file), parallel
  collection, six-class contact labels from truth, the torque MLP and the contact CNN, training and grid evaluation.
- `app/`: a local web app (http://127.0.0.1:8771) with a live bench, experiment sweeps and camera checks.
- `docs/SPEC-training-v1.md`: the build contract and the rulings so far. `docs/reports/agent-a-v1.md` and
  `agent-b-v1.md`: the builders' reports with every number and the command that produced it.
- Research: `/Users/shivenagarwal/GamifyAssessment/docs/research/2026-10/contact-policy-training.md` (what to
  train and how, with the literature), `sim-real-image-transfer.md` (masking and image techniques for later),
  `sim-training-explainer.html` (the explainer Shiven has), `fanuc-cell-plan.html` (the new-cell plan).
- Inputs copied from the NUC on 2026-10-06 under `inputs/nuc/` (23 demo episodes with the three videos, 184 policy
  rollouts, poses, calibrations, station.yaml, vendored driver and i2rt config). Nothing was written to the NUC.

Numbers as of today (sim graded against sim unless stated):

| Thing | Number | Where |
|---|---|---|
| Camera registration, 36 picked points, two poses, three cameras | 4.7 px rms, fingertips within 2 to 10 px | out/register_all_report.txt, out/camgrid/verify_*.png |
| Bench speed, one process | 178 ticks per second, about 6x real time, 30 episodes per minute | agent-a-v1.md |
| Scripted expert, clean bench, slip 0 to 4 mm by tilt 0 to 8 deg | 295/300 = 98.3 percent | agent-a-v1.md |
| Scripted expert, full randomisation, stage 2 | 251/400 = 62.8 percent | out/stage2_expert_weld001.json |
| Seat detector v1 | misses 96 percent of seats, being rebuilt | same |
| Torque noise from 252 static holds, right arm joints 1 to 6, Nm | 0.039, 0.238, 0.109, 0.053, 0.020, 0.013 | out/torque_noise.json |
| Real demos | 508 total on the NUC, 23 copied; 184 rollouts, 24 satisfied, 10 failed, 150 uncertain | NUC episode-studio catalog |

## 3. Claims to verify on the cell (your first job)

For each: VERIFIED / WRONG with the right value / CANNOT CHECK with what you need. Measure, do not reason.

1. Which arm holds which half. We say: left (fixture) arm holds the wired female, right (learned) arm holds the male
   with pins. In the wrist videos, right_wrist.mp4 shows the pins on the NEAR housing at the bottom of the frame.
2. The wrist camera mount. We say: D405 on a printed L bracket bolted to the gripper's flat base at the finger roots,
   optical centre about 86 mm above the finger axis (link_6 +y), 37 mm forward of the flange, pitched 23 degrees
   down toward the tips, same bracket on both arms. Please measure the lens centre from the flange face and from the
   flat base plate with a caliper, and the tilt of the camera face against the plate.
3. The recording path. We say: the D405 streams are recorded at 640x480 and the fit prefers a 50 degree vertical
   field of view (a slight crop of the native 58). The D435 scene camera is 640x480 at 58 degrees by our fit, 0.48 m
   above the table, nearly straight down, with the RIGHT arm on the image LEFT. Please read the actual RealSense
   stream configuration (resolution, whether the colour frame is cropped or scaled) and the scene camera's mount
   height and tilt.
4. The right arm's base. To make the 16 seated demos' tool axes line up and match the camera views, the twin needs the
   right base rolled about 24 degrees and raised 35 mm relative to a level plate 1.25 m from the left one. Shiven
   says the plates are about 120 cm apart and level. A 24 degree roll is not physical. The likely cause is joint
   calibration on the right arm. Please put the right arm at its calibration pose and compare the reported joint
   angles with what that pose should read, and check whether its joint zeros were set differently from the left arm.
5. The gripper roll. With the recorded joint 6 values the twin's two pinch axes differ by 27 degrees about the mating
   axis, which cannot be true for mated parts. We fold a -27 degree offset into the right arm's joint 6. Please check
   how each gripper is bolted to its flange (bolt pattern angle) and whether the two differ.
6. The torque reading. We say: the recorded "effort" is the motor's output torque and INCLUDES the gravity and
   friction feedforward (the free-hang baseline on right joint 2 is about -9.8 Nm). Please confirm from the driver
   what the effort field is (current times torque constant, or the commanded torque) and whether the feedforward is
   inside it. This decides the external-torque formula in the deployment adapter.
7. The seated geometry. We measured a flange-to-flange distance of 304.8 mm along the left tool axis at seat (std
   0.95 mm over 16 demos) and 318.3 mm at the preinsertion2 start. Please caliper the flange-to-flange distance at the
   seated pose if the arms can be put there safely.
8. The pick-error distribution. We assumed 1 to 5 mm lateral and 1 to 8 degrees at first contact. Please estimate it
   from the NUC data: for every satisfied rollout and demo, the tool pose at contact onset minus the seated pose, from
   forward kinematics. We could not do it without the full set.
9. Latency and sensing. We randomise 0 to 2 ticks of latency at 30 Hz and the measured noise above. Please measure
   command-to-encoder latency and the torque reading's quantisation and update rate from the driver.
10. The female housing on the cell: is it rigidly held, or does the fixture arm's grip or the housing flex? We model
    a rigid grip with a small tilt randomisation (0 to 2 degrees).

## 4. Work for the NUC side, in order

Write the plan for Shiven first (step 3 of "How to use this document"), then:

1. Record D405 depth and all three RGB streams on every run from now on, including policy rollouts. None of the 184
   rollouts have images and no depth was ever recorded. Everything on the image side waits on this.
2. Cut real contact segments. From the 508 demos: the window from first torque rise (the rule in
   `cell_twin/controllers.py`, contact onset) to the seat spike, with joints, commands, torques and the three
   videos. Give us 20 clean ones first, then all. These feed the trace check: we replay their commanded joint
   trajectories in the bench from the matching pick error and compare torque traces (per-joint RMSE, seat spike
   size and timing). If the bench's seat spike is off by more than 20 percent we recalibrate before trusting any
   sim number. This is the gate for everything else.
3. The deployment adapter, as a module beside the ACT runner, not inside it:
   - a 30 Hz loop that reads joints and torques, computes the 24-feature observation exactly as `env.py` does (tool
     pose from forward kinematics relative to the pose at contact onset, external torque = effort minus the model
     torque where the model includes the gravity and friction feedforward if claim 6 says the effort includes it,
     tracking error, last action, time), keeps the 16-tick window, runs the policy (a 3 x 256 MLP, torch, CPU) and
     maps its bounded tool-frame deltas (0.3 mm lateral, 0.5 mm push, 0.5 degree per tick) through damped
     least-squares inverse kinematics to joint targets for the same driver ACT uses;
   - a contact onset rule and a seat detector identical to the sim's, both from observables only;
   - a mode switch: ACT / alignment stage / contact policy / human, with the reason logged;
   - hand-off to ACT after the alignment stage backs off 3 mm;
   - per-tick logging of everything above plus the classifier's six probabilities, with one clock for camera, state
     and command timestamps; per-episode outcome label (S, S-late, F-miss, F-jam, F-force, F-drop, X) and the
     continuity tester result.
   Please check how the driver's lease model lets a second controller stream joint targets while ACT holds the arm.
4. Shadow mode: run ACT for 50 trials under the rubric with the continuity tester while the classifier and the seat
   detector only log. Outputs: the ACT baseline with a Wilson interval, the detector's real false positive and false
   negative rates, the switch-latency distribution. No switching yet.
5. The alignment stage live for 50 trials from the same start poses, compared with Fisher's exact test. 30 to 60
   percent clears it; 30 to 45 percent does not.
6. Optional, if the NUC has cores to spare: run collection there. `scripts/collect_expert.py --n N --procs P --stage 2
   --noise 0.3` needs only the repo, the vendored assets and the three registration files in `out/`. Modal with many
   CPU containers is the other option; the Mac cannot run more than 3 processes (it kernel-panicked twice today
   under 8).

## 5. Rules that bind both sides

- Sensor-only: nothing from simulator truth or from the cell's privileged state goes into a policy observation, a
  reward, or a stage switch. Truth is for grading and for supervised labels only.
- Do not change the running cell, its station file, calibrations or driver without Shiven in the loop.
- Every number comes with the command or the measurement that produced it. Mark estimates ESTIMATE and anything
  unverified UNVERIFIED.
- No em dashes in anything written for Shiven.

## 6. Files to read, in this order

1. `docs/SPEC-training-v1.md` (the contract and rulings)
2. `docs/reports/agent-a-v1.md` and `docs/reports/agent-b-v1.md` (what was built and measured)
3. `cell_twin/env.py` (the observation and reward the adapter must reproduce)
4. `/Users/shivenagarwal/GamifyAssessment/docs/research/2026-10/contact-policy-training.md` sections 1.2, 2.3, 5.2, 7.2

## 7. Open questions only Shiven can answer

1. May a classifier trained on bench-truth labels be used as the RL reward, or only the hand-built detector? (We
   proceeded with the hand-built detector for the reward and the classifier for switching.)
2. Does the cell have a continuity tester or another objective seat check we can log automatically?
3. Who operates during the 50-trial shadow and live runs, and when is cell time available?
