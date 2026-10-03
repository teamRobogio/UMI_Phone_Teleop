"""
Generates a fake Sensor Recorder Pro ZIP with an arkit_pose.csv so the
viewer can be tested without a phone.

Motion (in real-world terms): the phone is held in landscape, tilted
30 deg down, and moves 40 cm forward, 20 cm left and 15 cm up along a
wavy path while turning 90 deg left. ARKit world is Y-up, -Z forward.

Run:
    python make_test_recording.py          # writes test_recording.zip
"""

import io
import zipfile

import numpy as np
import pandas as pd


def axis_angle_quat(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    s = np.sin(angle / 2)
    return np.array([np.cos(angle / 2), *(axis * s)])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def main(out_path="test_recording.zip", fps=60.0, duration=6.0):
    t = np.arange(0, duration, 1 / fps)
    s = t / duration

    # Start at an arbitrary world position and heading, to prove the
    # viewer moves it to the origin.
    start = np.array([1.2, 0.9, -0.4])
    start_yaw = np.deg2rad(35)

    # Motion in a heading-aligned frame: x right, y up, z back (forward = -z)
    local = np.stack([
        -0.20 * s + 0.03 * np.sin(4 * np.pi * s),
        0.15 * s,
        -0.40 * s,
    ], axis=1)
    c, si = np.cos(start_yaw), np.sin(start_yaw)
    R_yaw = np.array([[c, 0, si], [0, 1, 0], [-si, 0, c]])
    pos = start + (R_yaw @ local.T).T

    tilt = axis_angle_quat([1, 0, 0], np.deg2rad(-30))   # pitch camera down
    quats = []
    for si_ in s:
        yaw = axis_angle_quat([0, 1, 0], start_yaw + np.deg2rad(90) * si_)
        quats.append(quat_mul(yaw, tilt))
    quats = np.array(quats)

    df = pd.DataFrame({
        "timestamp_s": 1000.0 + t,
        "tx_m": pos[:, 0], "ty_m": pos[:, 1], "tz_m": pos[:, 2],
        "qw": quats[:, 0], "qx": quats[:, 1], "qy": quats[:, 2], "qz": quats[:, 3],
    })

    buf = io.StringIO()
    df.to_csv(buf, index=False)
    with zipfile.ZipFile(out_path, "w") as z:
        z.writestr("recording_test/arkit_pose.csv", buf.getvalue())
    print(f"Wrote {out_path} ({len(df)} frames)")


if __name__ == "__main__":
    main()
