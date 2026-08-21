r"""Scan the Feetech servo bus and report which motor IDs are already programmed.

Usage:  .venv\python.exe scan_bus.py [COM3] [--all-ids]

Pings each ID individually rather than using a broadcast ping. A broadcast makes
every motor answer at once, so several factory-default motors (all ID 1) collide
into garbage. Addressing one ID at a time means distinct IDs answer cleanly --
which is what tells you whether a previous owner already ran setup-motors.
"""

import sys

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.feetech.tables import MODEL_NUMBER_TABLE

SO101_JOINTS = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
MODEL_NAMES = {nb: name for name, nb in MODEL_NUMBER_TABLE.items()}

PING_TIMEOUT_MS = 30  # a reply at >=1 Mbps takes microseconds; 1000ms default is far too slow to sweep

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
all_ids = "--all-ids" in sys.argv

bus = FeetechMotorsBus(port, {})
bus._connect(handshake=False)
print(f"Opened {port}\n")

found_any = {}
for baudrate in bus.available_baudrates:
    bus.set_baudrate(baudrate)
    bus.set_timeout(PING_TIMEOUT_MS)
    # Full sweep at lerobot's default baudrate; just the plausible range elsewhere.
    id_range = range(254) if (all_ids or baudrate == bus.default_baudrate) else range(1, 21)
    hits = {}
    for motor_id in id_range:
        model_nb = bus.ping(motor_id)
        if model_nb is not None:
            hits[motor_id] = model_nb
    if hits:
        found_any[baudrate] = hits
        tag = "  <- lerobot default" if baudrate == bus.default_baudrate else ""
        print(f"baudrate {baudrate}{tag}")
        for motor_id, model_nb in sorted(hits.items()):
            model = MODEL_NAMES.get(model_nb, f"unknown model {model_nb}")
            joint = SO101_JOINTS.get(motor_id, "not an SO-101 id")
            print(f"    id {motor_id:>3}  {model:<10}  {joint}")

bus.port_handler.closePort()

print("\n=== verdict ===")
if not found_any:
    print("Nothing responded. Check: power supply plugged in, USB cable, 3-pin servo")
    print("cable, and (Waveshare board) both jumpers set to the B / USB channel.")
    raise SystemExit(1)

expected = set(SO101_JOINTS)
at_default = found_any.get(bus.default_baudrate, {})
if set(at_default) >= expected:
    print(f"All 6 SO-101 ids present at {bus.default_baudrate} baud -- already configured.")
    print("You can SKIP lerobot-setup-motors and go straight to lerobot-calibrate.")
elif at_default:
    missing = sorted(expected - set(at_default))
    print(f"Partially configured at {bus.default_baudrate} baud.")
    print(f"Present: {sorted(at_default)}   Missing: {missing}")
    print("Missing ones are unconfigured (still id 1, colliding) or not connected.")
else:
    print(f"Motors answered, but not at {bus.default_baudrate} baud -- they need reprogramming.")
