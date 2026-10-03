"""
Sensor Recorder Pro — ARKit 6-DoF Trajectory Viewer (UMI test)

What it does:
1. Opens a file picker for your Sensor Recorder Pro .zip (or arkit_pose.csv)
2. Finds arkit_pose.csv automatically
3. Makes the starting phone pose the origin
4. Displays the full XYZ trajectory (Z up, like the real world)
5. Animates a 3D phone box + camera-direction arrow along the trajectory
6. Uses the recorded quaternion to rotate the phone in 3D

Requirements:
    pip install numpy pandas matplotlib

Run:
    python arkit_3d_viewer.py                    # file picker
    python arkit_3d_viewer.py recording.zip      # direct path
    python arkit_3d_viewer.py recording.zip --snapshot out.png   # no window, save final frame
"""

import argparse
import os
import sys
import tempfile
import zipfile

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============================================================
# SETTINGS
# ============================================================

# Phone box in the ARKit *camera* frame, in meters.
# ARKit's camera frame is landscape: +X = long side, +Y = short side,
# camera looks along -Z.
PHONE_LONG = 0.150
PHONE_SHORT = 0.075
PHONE_DEPTH = 0.010

# Length of the camera-direction arrow (m)
ARROW_LENGTH = 0.08

# 1 = every recorded frame, 2 = every second frame, ...
FRAME_SKIP = 1

# Display units
USE_CM = True

# How the start pose becomes the origin:
#   "yaw"   - remove starting position + heading only. Gravity stays
#             "up", so the floor stays flat. Best for UMI.
#   "full"  - express everything in the full starting phone frame
#             (axes tilt with however the phone was held at start).
#   "world" - remove starting position only, keep ARKit world axes.
ORIGIN_FRAME = "yaw"

# Fallback playback rate when the CSV has no timestamp column
DEFAULT_FPS = 30.0


# ============================================================
# FILE SELECTION
# ============================================================

def select_file():
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    filename = filedialog.askopenfilename(
        title="Select Sensor Recorder Pro ZIP (or arkit_pose.csv)",
        filetypes=[("ZIP / CSV", "*.zip *.csv"), ("All files", "*.*")],
    )
    root.destroy()

    if not filename:
        raise SystemExit("No file selected.")
    return filename


def find_arkit_pose(path):
    if path.lower().endswith(".csv"):
        return path

    temp_dir = tempfile.mkdtemp(prefix="sensor_recorder_")
    with zipfile.ZipFile(path, "r") as z:
        z.extractall(temp_dir)

    for root, _dirs, files in os.walk(temp_dir):
        for file in files:
            if file.lower() == "arkit_pose.csv":
                found = os.path.join(root, file)
                print(f"Found ARKit pose file:\n{found}")
                return found

    raise FileNotFoundError(
        "Could not find arkit_pose.csv inside the ZIP.\n"
        "Make sure the recording was made using ARKit mode."
    )


# ============================================================
# ROTATION HELPERS
# ============================================================

def quaternion_to_matrix(qw, qx, qy, qz):
    q = np.array([qw, qx, qy, qz], dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.eye(3)
    w, x, y, z = q / norm

    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w),     2 * (x*z + y*w)],
        [2 * (x*y + z*w),     1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w),     2 * (y*z + x*w),     1 - 2 * (x*x + y*y)],
    ])


# Camera-frame conventions: (forward axis, up axis) in camera coordinates.
#   arkit      - Apple camera frame: x right, y up, z back (looks along -Z)
#   opencv_rdf - x right, y down, z forward (Sensor Recorder Pro default)
CAMERA_CONVENTIONS = {
    "arkit":      (np.array([0, 0, -1.0]), np.array([0, 1.0, 0])),
    "opencv_rdf": (np.array([0, 0, 1.0]),  np.array([0, -1.0, 0])),
}


def yaw_only_rotation(R, convention):
    """Rotation about ARKit world +Y (up) matching the heading of camera pose R."""
    cam_forward, cam_up = CAMERA_CONVENTIONS[convention]
    # If the camera points almost straight up/down, use the camera's
    # up axis for heading instead.
    forward = R @ cam_forward
    if np.hypot(forward[0], forward[2]) < 0.2:
        forward = R @ cam_up

    yaw = np.arctan2(-forward[0], -forward[2])   # 0 when facing world -Z
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [c,  0, s],
        [0,  1, 0],
        [-s, 0, c],
    ])


# ARKit world is Y-up (X right, Y up, Z toward the viewer).
# Matplotlib draws Z up, so map ARKit (x, y, z) -> plot (x, -z, y):
# plot X = right, plot Y = forward, plot Z = up.
ARKIT_TO_PLOT = np.array([
    [1, 0,  0],
    [0, 0, -1],
    [0, 1,  0],
], dtype=float)


# ============================================================
# PHONE GEOMETRY (ARKit camera frame)
# ============================================================

def create_phone_vertices():
    w = PHONE_LONG / 2
    h = PHONE_SHORT / 2
    d = PHONE_DEPTH / 2
    return np.array([
        [-w, -h, -d], [w, -h, -d], [w, h, -d], [-w, h, -d],
        [-w, -h,  d], [w, -h,  d], [w, h,  d], [-w, h,  d],
    ])


PHONE_VERTICES = create_phone_vertices()

PHONE_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]

def camera_arrow(convention):
    forward, _up = CAMERA_CONVENTIONS[convention]
    return np.array([[0, 0, 0], forward * ARROW_LENGTH])


# ============================================================
# LOAD + TRANSFORM
# ============================================================

def find_time_column(df):
    # Prefer the monotonic sensor clock over wall-clock UTC
    for name in ("sensor_sec", "timestamp_s", "timestamp", "time"):
        if name in df.columns:
            return name
    for col in df.columns:
        if "time" in col.lower() or col.lower().endswith("_sec"):
            return col
    return None


def read_header_metadata(csv_path):
    """Parse the '# key,value,...' lines Sensor Recorder Pro puts above the header."""
    meta = {}
    with open(csv_path, encoding="utf-8") as f:
        for line in f:
            if not line.startswith("#"):
                break
            parts = [p.strip() for p in line[1:].split(",")]
            if parts and parts[0]:
                meta[parts[0]] = parts[1:]
    return meta


def detect_camera_convention(meta):
    values = meta.get("camera_coordinates", [])
    if values and values[0] in CAMERA_CONVENTIONS:
        return values[0]
    # Older exports without the header used Apple's camera frame
    return "arkit"


def load_trajectory(csv_path):
    meta = read_header_metadata(csv_path)
    convention = detect_camera_convention(meta)
    df = pd.read_csv(csv_path, comment="#")
    print(f"\nColumns found:\n{list(df.columns)}")
    print(f"Camera frame convention: {convention}")

    required = ["tx_m", "ty_m", "tz_m", "qw", "qx", "qy", "qz"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(
            f"Required column(s) {missing} not found.\n"
            f"Available columns:\n{list(df.columns)}"
        )

    df = df.dropna(subset=required).reset_index(drop=True)

    if "tracking_state" in df.columns:
        counts = df["tracking_state"].value_counts()
        print("Tracking state: " + ", ".join(f"{k}={v}" for k, v in counts.items()))
        if counts.get("normal", 0) < len(df):
            print("WARNING: some frames are not 'normal' tracking - "
                  "ARKit position may be unreliable there.")

    positions = df[["tx_m", "ty_m", "tz_m"]].to_numpy(dtype=float)
    rotations = np.array([
        quaternion_to_matrix(*q)
        for q in df[["qw", "qx", "qy", "qz"]].to_numpy(dtype=float)
    ])

    # Reference frame for the origin
    if ORIGIN_FRAME == "full":
        R_ref = rotations[0]
    elif ORIGIN_FRAME == "yaw":
        R_ref = yaw_only_rotation(rotations[0], convention)
    else:
        R_ref = np.eye(3)

    positions_local = (R_ref.T @ (positions - positions[0]).T).T
    rotations_local = R_ref.T @ rotations

    # "full" frame is the phone's own camera frame — show it as-is.
    # The gravity-aligned frames get converted to Z-up for plotting.
    to_plot = np.eye(3) if ORIGIN_FRAME == "full" else ARKIT_TO_PLOT
    positions_plot = (to_plot @ positions_local.T).T
    rotations_plot = to_plot @ rotations_local

    # Timestamps (seconds) for real-time playback
    time_col = find_time_column(df)
    if time_col is not None:
        t = df[time_col].to_numpy(dtype=float)
        t = t - t[0]
        # Guess units: nanoseconds / milliseconds -> seconds
        span = t[-1] if len(t) > 1 else 0
        if span > 1e7:
            t = t / 1e9
        elif span > 1e4:
            t = t / 1e3
    else:
        t = np.arange(len(df)) / DEFAULT_FPS

    return positions_plot, rotations_plot, t, time_col, convention


def print_summary(positions, t, time_col):
    distance_from_start = np.linalg.norm(positions, axis=1)
    path_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
    duration = t[-1] if len(t) > 1 else 0
    rate = (len(t) - 1) / duration if duration > 0 else float("nan")

    print("\n==============================")
    print("TRAJECTORY INFORMATION")
    print("==============================")
    print(f"Frames: {len(positions)}   Duration: {duration:.2f} s   "
          f"Rate: {rate:.1f} Hz   (time column: {time_col or 'none'})")
    print(f"Origin frame: {ORIGIN_FRAME}")
    print(f"Maximum distance from start: {distance_from_start.max()*100:.2f} cm")
    print(f"Final distance from start:   {distance_from_start[-1]*100:.2f} cm")
    print(f"Total tracked path length:   {path_length*100:.2f} cm")
    print("\nFinal XYZ relative to start:")
    for axis, value in zip("XYZ", positions[-1]):
        print(f"{axis}: {value*100:.2f} cm")


# ============================================================
# PLOT
# ============================================================

def build_viewer(positions, rotations, t, convention):
    scale, unit = (100.0, "cm") if USE_CM else (1.0, "m")
    trajectory = positions * scale
    arrow = camera_arrow(convention)

    if ORIGIN_FRAME == "full":
        axis_names = (("X (phone right)", "Y (phone down)", "Z (phone forward)")
                      if convention == "opencv_rdf" else
                      ("X (phone right)", "Y (phone up)", "Z (phone back)"))
    else:
        axis_names = ("X (right)", "Y (forward)", "Z (up)")

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("ARKit 6-DoF Phone Trajectory\nStarting Pose = Origin")
    ax.set_xlabel(f"{axis_names[0]} [{unit}]")
    ax.set_ylabel(f"{axis_names[1]} [{unit}]")
    ax.set_zlabel(f"{axis_names[2]} [{unit}]")

    # Equal axis range so the path isn't distorted
    mins, maxs = trajectory.min(axis=0), trajectory.max(axis=0)
    center = (mins + maxs) / 2
    largest_range = max(np.max(maxs - mins) * 1.4, 0.20 * scale)
    half = largest_range / 2
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1, 1, 1))

    ax.scatter([0], [0], [0], s=80, color="green", label="START")
    ax.text(0, 0, 0, " START")
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
            color="gray", alpha=0.25)

    path_line, = ax.plot([], [], [], linewidth=2, color="tab:blue",
                         label="Movement path")
    current_point, = ax.plot([], [], [], marker="o", markersize=6,
                             color="tab:blue")
    phone_lines = [ax.plot([], [], [], linewidth=2, color="black")[0]
                   for _ in PHONE_EDGES]
    camera_line, = ax.plot([], [], [], linewidth=2, color="red",
                           label="Camera direction")
    status_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                            va="top", family="monospace")
    ax.legend(loc="upper right")

    def set_line(line, pts):
        line.set_data(pts[:, 0], pts[:, 1])
        line.set_3d_properties(pts[:, 2])

    def update(frame):
        set_line(path_line, trajectory[:frame + 1])

        p = trajectory[frame]
        set_line(current_point, p[None, :])

        R = rotations[frame]
        verts = (R @ (PHONE_VERTICES * scale).T).T + p
        for line, (a, b) in zip(phone_lines, PHONE_EDGES):
            set_line(line, verts[[a, b]])
        set_line(camera_line, (R @ (arrow * scale).T).T + p)

        distance = np.linalg.norm(positions[frame])
        status_text.set_text(
            f"Frame: {frame}/{len(trajectory)-1}   t = {t[frame]:.2f} s\n"
            f"X: {p[0]:7.2f} {unit}\n"
            f"Y: {p[1]:7.2f} {unit}\n"
            f"Z: {p[2]:7.2f} {unit}\n"
            f"Distance from start: {distance*100:.2f} cm"
        )
        return [path_line, current_point, camera_line, status_text] + phone_lines

    return fig, update


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", nargs="?", help="Sensor Recorder Pro .zip or arkit_pose.csv")
    parser.add_argument("--snapshot", metavar="PNG",
                        help="Save the final frame to a PNG instead of opening a window")
    args = parser.parse_args()

    path = args.path or select_file()
    csv_path = find_arkit_pose(path)
    positions, rotations, t, time_col, convention = load_trajectory(csv_path)
    print_summary(positions, t, time_col)

    fig, update = build_viewer(positions, rotations, t, convention)

    if args.snapshot:
        update(len(positions) - 1)
        fig.savefig(args.snapshot, dpi=110)
        print(f"\nSaved snapshot: {args.snapshot}")
        return

    frames = range(0, len(positions), FRAME_SKIP)
    # Real-time playback based on median frame interval
    dt = np.median(np.diff(t)) if len(t) > 1 else 1 / DEFAULT_FPS
    interval_ms = max(1, int(1000 * dt * FRAME_SKIP))

    _animation = FuncAnimation(fig, update, frames=frames, interval=interval_ms,
                               blit=False, repeat=True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    sys.exit(main())
