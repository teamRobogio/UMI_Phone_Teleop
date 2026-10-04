"""GUI to calibrate the SC09 trigger against the SO-101 gripper range.

1. Connect to the trigger's serial bus board.
2. Hold the trigger where the gripper should be fully OPEN, press "OK - open".
3. Hold the trigger where the gripper should be fully CLOSED, press "OK - closed".
4. Move the trigger and check the live gripper preview, then press "Save".
Saved to trigger_calibration.json next to this script.
"""

import statistics
import time
import tkinter as tk
from tkinter import messagebox, ttk

from trigger import (
    CALIBRATION_FILE,
    SO101_GRIPPER_CLOSED,
    SO101_GRIPPER_OPEN,
    TRIGGER_ID,
    TRIGGER_PORT,
    Trigger,
    save_calibration,
    trigger_to_gripper,
)

POLL_MS = 50
MIN_TRAVEL = 20  # counts; less than this means the trigger did not move between the two positions


class CalibrationApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.trigger = None
        self.raw = None
        self.open_raw = None
        self.closed_raw = None

        root.title("Trigger calibration")
        root.resizable(False, False)
        frame = ttk.Frame(root, padding=16)
        frame.grid()

        ttk.Label(frame, text="Port").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value=TRIGGER_PORT)
        ttk.Entry(frame, textvariable=self.port_var, width=10).grid(row=0, column=1, sticky="w")
        ttk.Label(frame, text="Servo ID").grid(row=0, column=2, sticky="w", padx=(12, 0))
        self.id_var = tk.StringVar(value=str(TRIGGER_ID))
        ttk.Entry(frame, textvariable=self.id_var, width=5).grid(row=0, column=3, sticky="w")
        self.connect_btn = ttk.Button(frame, text="Connect", command=self.connect)
        self.connect_btn.grid(row=0, column=4, padx=(12, 0))

        # Live position on the full servo range, to place the horn so the whole trigger travel
        # avoids the dead zone at 0 / 1023.
        live = ttk.Frame(frame)
        live.grid(row=1, column=0, columnspan=5, sticky="w", pady=(14, 10))
        self.raw_label = ttk.Label(live, text="Trigger position: -", font=("Segoe UI", 14))
        self.raw_label.grid(row=0, column=0, sticky="w")
        self.raw_bar = ttk.Progressbar(live, length=460, maximum=1023)
        self.raw_bar.grid(row=1, column=0, sticky="w", pady=(2, 2))
        ttk.Label(
            live,
            text="Servo range 0-1023. Keep the whole squeeze between ~100 and ~920 (ideally around 512).",
            foreground="gray",
        ).grid(row=2, column=0, sticky="w")

        ttk.Label(frame, text="1. Hold the trigger where the gripper should be fully OPEN").grid(
            row=2, column=0, columnspan=4, sticky="w"
        )
        self.open_btn = ttk.Button(frame, text="OK - open", command=self.set_open, state="disabled")
        self.open_btn.grid(row=2, column=4, pady=4)
        self.open_label = ttk.Label(frame, text="open: not set")
        self.open_label.grid(row=3, column=0, columnspan=5, sticky="w")

        ttk.Label(frame, text="2. Hold the trigger where the gripper should be fully CLOSED").grid(
            row=4, column=0, columnspan=4, sticky="w", pady=(10, 0)
        )
        self.closed_btn = ttk.Button(frame, text="OK - closed", command=self.set_closed, state="disabled")
        self.closed_btn.grid(row=4, column=4, pady=(10, 4))
        self.closed_label = ttk.Label(frame, text="closed: not set")
        self.closed_label.grid(row=5, column=0, columnspan=5, sticky="w")

        ttk.Label(frame, text="3. Move the trigger and check the SO-101 gripper preview").grid(
            row=6, column=0, columnspan=5, sticky="w", pady=(10, 2)
        )
        self.preview = ttk.Progressbar(frame, length=360, maximum=100)
        self.preview.grid(row=7, column=0, columnspan=4, sticky="w")
        self.preview_label = ttk.Label(frame, text="gripper: -")
        self.preview_label.grid(row=7, column=4)

        self.save_btn = ttk.Button(frame, text="Save", command=self.save, state="disabled")
        self.save_btn.grid(row=8, column=4, pady=(14, 0))
        self.status = ttk.Label(frame, text="Connect to the trigger to start.", foreground="gray")
        self.status.grid(row=8, column=0, columnspan=4, sticky="w", pady=(14, 0))

        root.protocol("WM_DELETE_WINDOW", self.close)

    def connect(self):
        try:
            trigger = Trigger(self.port_var.get().strip(), int(self.id_var.get()))
            trigger.connect()
        except Exception as e:  # noqa: BLE001 - show any connection problem to the user
            messagebox.showerror("Connection failed", str(e))
            return
        self.trigger = trigger
        self.connect_btn.configure(state="disabled")
        self.open_btn.configure(state="normal")
        self.closed_btn.configure(state="normal")
        self.status.configure(text="Connected. Set the open and closed positions.")
        self.poll()

    def poll(self):
        try:
            self.raw = self.trigger.read_raw()
            self.raw_label.configure(text=f"Trigger position: {self.raw}")
            self.raw_bar["value"] = self.raw
            if self.open_raw is not None and self.closed_raw is not None:
                g = trigger_to_gripper(self.raw, {"open_raw": self.open_raw, "closed_raw": self.closed_raw})
                self.preview["value"] = g
                state = "open" if g >= SO101_GRIPPER_OPEN - 1 else "closed" if g <= SO101_GRIPPER_CLOSED + 1 else ""
                self.preview_label.configure(text=f"gripper: {g:5.1f} {state}")
        except ConnectionError:
            self.raw_label.configure(text="Trigger position: (read failed)")
        self.root.after(POLL_MS, self.poll)

    def capture(self) -> int:
        """Median of a short burst of reads, so a shaky hand does not shift the value."""
        values = []
        t_end = time.perf_counter() + 0.4
        while time.perf_counter() < t_end:
            try:
                values.append(self.trigger.read_raw())
            except ConnectionError:
                pass
            time.sleep(0.02)
        if not values:
            raise ConnectionError("No readings from the trigger.")
        return int(statistics.median(values))

    def set_open(self):
        self.open_raw = self.capture()
        self.open_label.configure(text=f"open: {self.open_raw}")
        self.update_save_state()

    def set_closed(self):
        self.closed_raw = self.capture()
        self.closed_label.configure(text=f"closed: {self.closed_raw}")
        self.update_save_state()

    def update_save_state(self):
        if self.open_raw is None or self.closed_raw is None:
            return
        travel = abs(self.closed_raw - self.open_raw)
        if travel < MIN_TRAVEL:
            self.save_btn.configure(state="disabled")
            self.status.configure(text=f"Travel only {travel} counts. Move the trigger further and set again.")
        else:
            self.save_btn.configure(state="normal")
            self.status.configure(text=f"Travel {travel} counts. Check the preview, then Save.")

    def save(self):
        save_calibration(self.open_raw, self.closed_raw, self.port_var.get().strip(), int(self.id_var.get()))
        self.status.configure(text=f"Saved to {CALIBRATION_FILE.name}.")
        print(f"Saved trigger calibration: open={self.open_raw}, closed={self.closed_raw}", flush=True)

    def close(self):
        if self.trigger is not None:
            self.trigger.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    CalibrationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
