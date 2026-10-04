"""SC09 servo used as a gripper trigger, read over its own Feetech serial bus board.

The servo runs with torque off so it moves freely; only its position is read.
SC09 is an SCS-series servo (LeRobot model "scs0009", Feetech protocol 1, 0-1023 counts).
The trigger position is mapped linearly onto the SO-101 gripper range, so the gripper follows it
smoothly: trigger at its "open" position -> gripper fully open, at its "closed" position -> fully closed.
"""

import json
from contextlib import contextmanager
from pathlib import Path

from scservo_sdk.scservo_def import SCS_GETEND, SCS_SETEND

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

TRIGGER_PORT = "COM16"  # serial bus board the trigger servo is plugged into (not the follower's)
TRIGGER_ID = 1  # servo ID (SC09 factory default is 1)
CALIBRATION_FILE = Path(__file__).with_name("trigger_calibration.json")

# SO-101 follower gripper range (LeRobot RANGE_0_100 normalisation).
SO101_GRIPPER_OPEN = 100.0
SO101_GRIPPER_CLOSED = 0.0


@contextmanager
def _protocol1_byte_order():
    """The Feetech SDK keeps byte order in one process-wide global that every PacketHandler overwrites,
    so the SC09 (protocol 1) and the follower's STS3215s (protocol 0) cannot share a process as-is.
    Switch to protocol 1 only while talking to the trigger, then restore the previous byte order."""
    previous = SCS_GETEND()
    SCS_SETEND(1)
    try:
        yield
    finally:
        SCS_SETEND(previous)


class Trigger:
    def __init__(self, port: str = TRIGGER_PORT, motor_id: int = TRIGGER_ID):
        with _protocol1_byte_order():
            self.bus = FeetechMotorsBus(
                port=port,
                motors={"trigger": Motor(motor_id, "scs0009", MotorNormMode.RANGE_0_100)},
                protocol_version=1,
            )

    def connect(self) -> None:
        with _protocol1_byte_order():
            # No handshake: it also checks firmware versions, not needed for a single read-only servo.
            self.bus.connect(handshake=False)
            if self.bus.ping("trigger", num_retry=3) is None:
                self.bus.disconnect(disable_torque=False)
                raise ConnectionError(
                    f"No SC09 trigger with ID {self.bus.motors['trigger'].id} on {self.bus.port}. "
                    "Check the port, ID, power to the bus board and baud rate (1,000,000)."
                )
            self.bus.disable_torque(num_retry=3)

    def read_raw(self) -> int:
        with _protocol1_byte_order():
            return int(self.bus.read("Present_Position", "trigger", normalize=False, num_retry=2))

    def disconnect(self) -> None:
        with _protocol1_byte_order():
            self.bus.disconnect(disable_torque=False)


def save_calibration(open_raw: int, closed_raw: int, port: str, motor_id: int) -> None:
    data = {"port": port, "id": motor_id, "open_raw": open_raw, "closed_raw": closed_raw}
    CALIBRATION_FILE.write_text(json.dumps(data, indent=2))


def load_calibration() -> dict:
    if not CALIBRATION_FILE.exists():
        raise FileNotFoundError(f"{CALIBRATION_FILE.name} not found. Run calibrate_trigger.py first.")
    return json.loads(CALIBRATION_FILE.read_text())


def trigger_to_gripper(
    raw: int,
    calibration: dict,
    gripper_open: float = SO101_GRIPPER_OPEN,
    gripper_closed: float = SO101_GRIPPER_CLOSED,
    full_close_at: float = 1.0,
) -> float:
    """Linear map of the trigger position onto the gripper range, clamped to the calibrated travel.

    full_close_at: fraction of the trigger travel at which the gripper is already fully closed
    (e.g. 0.85 leaves the last 15% of the squeeze as margin). Works whichever way the servo turns.
    """
    open_raw, closed_raw = calibration["open_raw"], calibration["closed_raw"]
    t = (raw - open_raw) / (closed_raw - open_raw)  # 0 at trigger-open, 1 at trigger-closed
    t = min(max(t / full_close_at, 0.0), 1.0)
    return gripper_open + t * (gripper_closed - gripper_open)
