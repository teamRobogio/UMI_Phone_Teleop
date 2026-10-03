# Handoff

State of the project as of 2026-10-02. See `README.md` for setup and running.

## Goal

Collect end-effector-space demonstrations for an SO-101 and train ACT / Diffusion Policy on them,
deploying through LeRobot's FK → policy → IK loop (`examples/phone_to_so100/evaluate.py`).

Two data paths were considered:

- **A. Live phone teleop (working).** The iPhone drives the SO-101 in real time; the dataset holds
  the robot's camera and the robot's FK gripper pose. This is what the scripts here do.
- **B. Robot-free UMI-style recording (not started).** Record with the phone alone (Sensor Recorder
  Pro `.zip`: RGB video + ARKit pose) and convert to a LeRobot dataset. Needs a converter, a camera
  mount matching the robot, a camera→gripper calibration, gripper-width capture, and relative
  (not absolute) state. `arkit_3d_viewer.py` is the only piece built for this so far.

## What works

- SO-101 FK/IK with LeRobot's `RobotKinematics` on native Windows (placo from conda-forge).
  FK→IK round trip reconstructs joints with 0.000 mm / 0.000° error.
- Phone teleop with `phone_to_so101/teleoperate.py` for 30+ minute sessions.
- Recording with `phone_to_so101/record.py`. Three local datasets were recorded in
  `~/.cache/huggingface/lerobot/local/`: `phone_so101_test` (20 s), `phone_so101_test2` (60 s),
  `phone_so101_test3` (60 s). Schema:
  - `observation.state` and `action`: float32[7] =
    `ee.x, ee.y, ee.z, ee.wx, ee.wy, ee.wz, ee.gripper_pos` (metres, rotation vector, gripper %)
  - `observation.images.front`: 480×640×3 video, 30 fps
  - Recorded with LeRobot's original phone mapping (see below), so teleop felt awkward, but the
    data itself is consistent: it records what the robot actually did.

## Changes compared with upstream LeRobot

1. `patches/lerobot-v0.6.0-phone-feedback-timeout.patch` (LeRobot core, 2 lines):
   `IOSPhone._read_current_pose` returned `fbk[0]` without checking for `None`, so one late HEBI
   packet crashed the run. It now returns "no pose" for that cycle.
2. `phone_to_so101/record.py`: copy of `examples/phone_to_so100/record.py`, settings only
   (SO-101 classes, COM port, calibration id `so-arm101`, URDF path, dataset name, 60 s episodes,
   `push_to_hub` disabled).
3. `phone_to_so101/teleoperate.py`: copy of `examples/phone_to_so100/teleoperate.py` with settings
   plus control changes, all in the script (toggled by constants at the top):
   - **No-pose hold:** skip the cycle when the phone returns no pose (ARKit lost tracking), instead
     of crashing with `KeyError: 'phone.enabled'`.
   - `PHONE_TOP_FORWARD`: LeRobot's mapping (`MapPhoneActionToRobotAction`: `target_x = -pos[1]`)
     expects the phone's charging port to point forward, contrary to its own printed instruction.
     The script rotates phone motion 180° about the screen normal so the natural grip works.
   - `REMAP_ROTATION_AXES` + `PITCH_SIGN / ROLL_SIGN / YAW_SIGN`: nose tilt → wrist pitch, roll →
     gripper twist, flat turn → sideways rotation. All signs are currently `1`; set by hand on the
     real robot.
   - `B1_TOGGLE`: tap B1 to follow, tap again to hold, instead of holding B1. Requires a B1 release
     after calibration before the first toggle.

## Known issues / gotchas

- **`record.py` does not have the teleop control changes yet** (top-forward, rotation remap,
  toggle, no-pose hold). Port them before recording more data so recordings match teleop.
- **Rotation is not re-zeroed on B1.** LeRobot re-zeroes phone *position* on each B1 press but
  measures rotation from the initial calibration pose. Pressing B1 with the phone tilted makes the
  gripper jump by that tilt. Re-zeroing rotation would be a one-line change in `teleop_phone.py`.
- **Motor bus dropouts.** Intermittent `There is no status packet!` on sync read, once on motor 5
  (`wrist_roll`) at connect. Static ping tests pass (0/300 failures), so it is probably a loose
  wrist cable or a power dip. LeRobot reads with a single try, so one bad packet ends the session.
- **Tracking jumps.** A 21 cm single-step EE jump once hit `EEBoundsAndSafety` (max 10 cm) and
  aborted teleop. Likely ARKit stutter from a hot, low-battery phone. Proposed (not built): switch
  to HOLD on jumps > 5 cm and use `raise_on_jump=False` with a 2 cm step limit as a backstop.
- **Loose safety limits** (from the examples): EE bounds ±1 m, max step 0.10 m (teleop) /
  0.20 m (record). An early run overloaded motor 4 (`wrist_flex`).
- **Follower calibration mismatch prompt.** LeRobot asks whether to use the calibration file when
  the motors' stored values differ. The scripts are launched with Enter piped in
  (`printf '\n' | python ...`) to accept the existing `so-arm101.json`.
- **Windows setup traps:** placo only via conda-forge; conda Pillow fails to load (use pip Pillow);
  `RobotKinematics` needs a URDF path that includes a directory; `rerun.exe` must be on `PATH`;
  run with `PYTHONUTF8=1`; COM ports re-enumerate between sessions.
- **Ending a session by killing the process leaves motor torque on** (the arm holds its pose).
  Support the arm before cutting power.

## Suggested next steps

1. Port the teleop control changes into `record.py`, then record 20–50 episodes of one task.
2. Smoke-test training (few hundred steps) with `lerobot-train` on one of the datasets
   (`--policy.type=act` or `diffusion`; install the `training`/`diffusion` extras). Check input and
   output dimensions in the logged config (state 7, action 7, one camera).
3. Offline check: load the checkpoint, run one frame, push the predicted EE action through IK
   without commanding the robot.
4. Deploy with a copy of `examples/phone_to_so100/evaluate.py` (FK observation → policy → IK),
   with tight EE bounds and step limits.
5. Optional: proportional gripper trigger using the leader arm's gripper servo (torque off, read
   position, send as follower gripper position).
