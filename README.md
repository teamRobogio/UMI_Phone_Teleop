# UMI Phone Teleop

Control an **SO-101 robot arm with an iPhone** and record **end-effector (6-DoF) datasets** in the
LeRobot format, plus a small viewer for phone trajectories recorded with **Sensor Recorder Pro**.

Built on [LeRobot v0.6.0](https://github.com/huggingface/lerobot) and its
`examples/phone_to_so100` scripts. No LeRobot code is rewritten; the scripts here are copies of
those examples with settings and a few small control tweaks.

## How it works

```
iPhone (HEBI Mobile I/O app, ARKit pose)          SO-101 follower arm
   phone motion since you tapped B1 ──┐            joint encoders
                                      │                  │
                                      ▼                 FK ──► current gripper pose
        gripper pose when B1 was tapped + phone motion
                                      │
                                      ▼
                     target gripper pose (x, y, z + rotation)
                                      │
                                     IK ──► joint targets ──► motors
```

- The phone app streams its **ARKit pose** (position + orientation) and buttons over Wi-Fi.
- Only phone **motion** after you press B1 is used; it is added to the gripper's pose at that moment.
- LeRobot's **inverse kinematics** (via `placo` and the SO-101 URDF) turns the target gripper pose
  into joint angles.
- In `teleoperate.py` the gripper follows a **hand trigger**: an SC09 servo on a 3D-printed gun
  that also holds the phone, read on its own serial bus board. The trigger position maps smoothly
  onto the SO-101 gripper (released = half open, full squeeze = fully closed).
- When recording, each frame saves the robot camera image, the robot's gripper pose
  (`observation.state`) and the commanded gripper pose (`action`), so policies such as ACT or
  Diffusion learn in gripper space, not joint space.

## Files

| file | what it is |
|---|---|
| `phone_to_so101/teleoperate.py` | phone → SO-101 live control, no recording, no time limit |
| `phone_to_so101/record.py` | phone → SO-101 control **and** recording to a LeRobot dataset |
| `phone_to_so101/trigger.py` | reads the SC09 trigger servo and maps it to the gripper |
| `phone_to_so101/calibrate_trigger.py` | GUI to calibrate the trigger (open / closed positions) |
| `phone_to_so101/trigger_calibration.json` | saved trigger calibration (recalibrate for your build) |
| `arkit_3d_viewer.py` | 3D viewer for Sensor Recorder Pro ARKit recordings (`.zip`) |
| `make_test_recording.py` | creates a fake recording to test the viewer |
| `patches/` | one small fix to apply to LeRobot (see setup step 3) |

## Setup (Windows)

You need: an SO-101 follower arm (calibrated), a USB camera, an iPhone on the same Wi-Fi as the PC,
Miniconda, and git.

1. **Get the dependencies** (inside this repo folder):
   ```
   git clone --depth 1 --branch v0.6.0 https://github.com/huggingface/lerobot.git third_party/lerobot
   git clone --depth 1 --filter=blob:none --sparse https://github.com/TheRobotStudio/SO-ARM100.git third_party/SO-ARM100
   git -C third_party/SO-ARM100 sparse-checkout set Simulation/SO101
   ```
2. **Create the Python environment.** `placo` (the IK library) has no Windows pip wheels, so it
   comes from conda-forge. The conda Pillow build does not load on Windows, so it is swapped for
   the pip one:
   ```
   conda create -n lerobot-phone -c conda-forge --override-channels python=3.12 placo
   conda activate lerobot-phone
   conda remove --force pillow
   pip install "pillow>=10,<13"
   pip install -e "third_party/lerobot[feetech,phone,core_scripts]"
   ```
3. **Apply the LeRobot patch** (stops a crash when one phone Wi-Fi packet is late):
   ```
   git -C third_party/lerobot apply ../../patches/lerobot-v0.6.0-phone-feedback-timeout.patch
   ```
4. **Edit the settings** at the top of `phone_to_so101/teleoperate.py` and `record.py`:
   - `port="COM14"`: your follower's COM port (it can change between sessions; check with
     `python -m serial.tools.list_ports`)
   - `id="so-arm101"`: the name of your follower calibration
   - `urdf_path=...`: full path to `third_party/SO-ARM100/Simulation/SO101/so101_new_calib.urdf`
   - camera index (`index_or_path=0`) in `record.py`
5. **On the iPhone**, install **HEBI Mobile I/O** from the App Store, open it, allow local network
   access, and keep it open in the foreground.
6. **Trigger (optional, teleoperate.py):** connect the SC09 servo (ID 1, 1,000,000 baud) to a
   **second** Feetech serial bus board, separate from the follower's.
   - **Power that board with 5–6 V.** The SC09 is a 4.8–6 V servo; on the robot's ~12 V supply it
     reports "Input voltage error" (and can be damaged).
   - Run `python phone_to_so101\calibrate_trigger.py`, enter the board's COM port, press
     **Connect**. Use the live position bar to fit the horn so the whole squeeze stays between
     ~100 and ~920 (away from the dead zone at 0 / 1023), then hold the trigger where the gripper
     should be open → **OK - open**, squeeze fully → **OK - closed**, check the preview, **Save**.
   - Tune in `teleoperate.py`: `GRIPPER_OPEN` (released, default 50 = half open),
     `GRIPPER_CLOSED` (0), `TRIGGER_FULL_CLOSE_AT` (0.85 = fully closed at 85% of the squeeze).
     Set `USE_TRIGGER = False` to go back to the A3 slider.

## Running

Activate the environment and make sure `rerun.exe` (the live viewer) is on `PATH`:

```
conda activate lerobot-phone
set PYTHONUTF8=1
python phone_to_so101\teleoperate.py      # live control
python phone_to_so101\record.py           # control + record one episode
```

Then:

1. When asked, hold the **phone itself flat, screen up, top edge pointing the way the robot faces**
   (charging port toward you) and **hold B1** until "Calibration done". Release B1. With the phone
   in the gun, tilt the gun so the phone is flat: all tilts are measured from this pose.
2. **Teleop:** tap **B1** to start following the phone, tap again to stop and hold.
   **Record:** hold **B1** while moving (the toggle is not in `record.py` yet).
3. **Gripper:** the trigger in teleop (works while following or holding; the gripper moves to the
   trigger position as soon as the script starts). In `record.py`, the **A3** slider (speed, not
   position).
4. Stop: `Ctrl+C` in teleop. In record: **→** ends the episode, **Esc** stops and saves.

Phone motion → robot (teleoperate.py):

| phone | robot |
|---|---|
| move forward / left / up | gripper forward / left / up (scaled ×0.5) |
| tilt nose down / up | wrist down / up |
| roll left / right (long axis) | gripper twist |
| flat turn | sideways gripper rotation |
| trigger released → squeezed | gripper half open → fully closed (smooth) |

Recordings are saved to `%USERPROFILE%\.cache\huggingface\lerobot\local\<name>`.

## Tips

- ARKit tracks with the rear camera: keep it **uncovered** and pointed at a **textured** surface
  (hold the phone 40+ cm above a patterned table or mat), in good light.
- A **hot or low-battery phone** makes tracking stutter, which can cause jumps. Keep it cool and
  charged, case off, Low Power Mode off.
- Close LeLab (or anything else) that uses the robot's COM port or the camera before running.
- COM ports can swap when boards are replugged. If the follower fails with "incorrect model
  numbers", check which board is which (`python -m serial.tools.list_ports`) and fix the ports.

## ARKit trajectory viewer

```
python arkit_3d_viewer.py                    # pick a Sensor Recorder Pro .zip
python arkit_3d_viewer.py recording.zip --snapshot out.png
```

Shows the phone's 3D path with Z up, an animated phone box and camera-direction arrow, and warns
when ARKit tracking was not `normal`. Needs only `numpy pandas matplotlib`.
