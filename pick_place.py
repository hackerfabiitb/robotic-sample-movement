r"""Shuttle an object between a series of positions with the SO-101.

Everything is configured in the CONFIG block below -- no command-line arguments.
Run it with:

    .venv\python.exe pick_place.py

For each leg the arm picks the object up at one position and sets it down at the
next, then carries on to the following pair, looping for CYCLES passes.

Each pick and place goes via a hover point above the target rather than driving
straight at it. Approaching from directly above keeps the gripper from sweeping
the object sideways off the table on the way in, and lifting before travelling
keeps it from dragging.
"""

from __future__ import annotations

import time

import numpy as np

import arm
from kinematics import ARM_JOINTS, SO101Kinematics

# ============================== CONFIG ==============================

PORT = "COM3"
ROBOT_ID = "arm0"

# Set True to solve and print the whole routine without moving the arm.
DRY_RUN = False

# Waypoints the object is carried between, in metres in `base_link`:
# +X forward out of the base, +Z up, origin at the base plate.
#
# Z is the height the gripper closes at, so it depends on your object and how
# high your table sits relative to the base plate. To find it: hold the object
# where you want it, back-drive the arm so the gripper is around it, and run
#   .venv\python.exe move_ee.py --show
# then copy the reported Z here.
POSITIONS: list[tuple[str, tuple[float, float, float]]] = [
    ("pos1", (0.22, -0.10, 0.04)),
    ("pos2", (0.22, 0.10, 0.04)),
    ("pos3", (0.28, 0.00, 0.04)),
]

# How far above a waypoint to hover before descending, metres.
HOVER_DZ = 0.06

# Pose the arm rests at between legs and at the end.
HOME_XYZ = (0.25, 0.0, 0.20)

GRIPPER_OPEN = 90.0
GRIPPER_CLOSED = 5.0

# How many times to run through the whole chain of positions.
CYCLES = 1

# Seconds for each motion. Travel moves are longer since they cover more ground.
T_TRAVEL = 3.0
T_DESCEND = 1.5
T_GRIPPER = 1.0

# Pause after the jaw moves, so it has actually gripped before the arm lifts.
GRIP_SETTLE = 0.5

# Cap on how far a goal may lead the measured position, in degrees.
MAX_STEP = 12.0

# ====================================================================


def hover(xyz: tuple[float, float, float]) -> np.ndarray:
    return np.array([xyz[0], xyz[1], xyz[2] + HOVER_DZ])


def solve_all(kin: SO101Kinematics) -> dict[str, np.ndarray]:
    """Solve every waypoint up front so a bad target fails before the arm moves."""
    targets: dict[str, np.ndarray] = {"home": np.array(HOME_XYZ)}
    for name, xyz in POSITIONS:
        targets[f"{name}"] = np.array(xyz)
        targets[f"{name} hover"] = hover(xyz)

    print("checking every waypoint is reachable")
    print(f"  {'waypoint':<16}{'x':>8}{'y':>8}{'z':>8}{'err mm':>9}")
    print("  " + "-" * 49)
    bad = []
    for label, xyz in targets.items():
        q, ok, err = kin.ik(xyz, q_init=np.zeros(5))
        flag = ""
        if err > 0.005:
            bad.append(label)
            flag = "  UNREACHABLE"
        for n, v in zip(ARM_JOINTS, q):
            lo, hi = kin.limits_deg[n]
            if v <= lo + 0.5 or v >= hi - 0.5:
                flag += f"  {n}@LIMIT"
        print(f"  {label:<16}{xyz[0]:>8.3f}{xyz[1]:>8.3f}{xyz[2]:>8.3f}{err * 1000:>9.2f}{flag}")

    if bad:
        raise SystemExit(f"\nERROR: cannot reach {bad}. Adjust POSITIONS and retry.")
    return targets


def goto(robot, kin: SO101Kinematics, xyz, duration: float, label: str) -> None:
    """Solve IK seeded from the present pose, then interpolate there."""
    joints = arm.read_joints(robot)
    q_now = arm.arm_vector(joints)
    q, ok, err = kin.ik(np.asarray(xyz), q_init=q_now)
    if err > 0.005:
        raise RuntimeError(f"IK failed for {label}: off by {err * 1000:.1f} mm")

    print(f"  -> {label:<18} [{xyz[0]:+.3f} {xyz[1]:+.3f} {xyz[2]:+.3f}]")
    arm.ramp_to(robot, dict(zip(ARM_JOINTS, q)), duration=duration, verbose=False)


def set_gripper(robot, value: float, label: str) -> None:
    print(f"  -> gripper {label}")
    arm.ramp_to(robot, {"gripper": value}, duration=T_GRIPPER, verbose=False)
    time.sleep(GRIP_SETTLE)


def pick(robot, kin, name: str, xyz) -> None:
    print(f"\nPICK at {name}")
    set_gripper(robot, GRIPPER_OPEN, "open")
    goto(robot, kin, hover(xyz), T_TRAVEL, f"{name} hover")
    goto(robot, kin, xyz, T_DESCEND, f"{name} grasp")
    set_gripper(robot, GRIPPER_CLOSED, "close")
    goto(robot, kin, hover(xyz), T_DESCEND, f"{name} lift")


def place(robot, kin, name: str, xyz) -> None:
    print(f"\nPLACE at {name}")
    goto(robot, kin, hover(xyz), T_TRAVEL, f"{name} hover")
    goto(robot, kin, xyz, T_DESCEND, f"{name} release")
    set_gripper(robot, GRIPPER_OPEN, "open")
    goto(robot, kin, hover(xyz), T_DESCEND, f"{name} retreat")


def main() -> int:
    kin = SO101Kinematics()
    solve_all(kin)

    if DRY_RUN:
        print("\nDRY_RUN is True -- every waypoint solved, nothing commanded.")
        print("Set DRY_RUN = False at the top of this file to run it for real.")
        return 0

    missing = arm.preflight(PORT)
    if missing:
        print(f"\nERROR: motor ids {missing} are not responding on {PORT}.")
        print("Check the 12V supply -- USB powers the board, not the servos.")
        return 1

    robot = arm.connect(PORT, ROBOT_ID, max_step=MAX_STEP)
    try:
        print("\nconnected -- torque ON. Ctrl+C to stop.")
        goto(robot, kin, HOME_XYZ, T_TRAVEL, "home")

        for cycle in range(1, CYCLES + 1):
            print(f"\n{'=' * 52}\ncycle {cycle} of {CYCLES}\n{'=' * 52}")
            # Carry the object along the chain: pos1 -> pos2 -> pos3 -> back to pos1.
            for i in range(len(POSITIONS)):
                src_name, src_xyz = POSITIONS[i]
                dst_name, dst_xyz = POSITIONS[(i + 1) % len(POSITIONS)]
                pick(robot, kin, src_name, src_xyz)
                place(robot, kin, dst_name, dst_xyz)

        print("\nreturning home")
        goto(robot, kin, HOME_XYZ, T_TRAVEL, "home")
        print("\ndone.")
    except KeyboardInterrupt:
        print("\n\ninterrupted -- stopping where it is.")
    finally:
        robot.disconnect()
        print("disconnected -- torque OFF, support the arm if it can fall")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
