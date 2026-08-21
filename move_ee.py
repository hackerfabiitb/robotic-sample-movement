r"""Move the SO-101 tool point to a Cartesian position, and open/close the gripper.

    .venv\python.exe move_ee.py --show
    .venv\python.exe move_ee.py --xyz 0.25 0.0 0.15
    .venv\python.exe move_ee.py --gripper open
    .venv\python.exe move_ee.py --xyz 0.25 0.0 0.15 --gripper close

Coordinates are metres in `base_link`: +X forward out of the base, +Z up,
origin at the base plate. Use --show to read the current tool position first.

Motion is interpolated in JOINT space, so the tool traces an arc rather than a
straight line. That is deliberate -- a straight-line Cartesian path can drive
the arm through a singularity, which is not what you want on a first setup.
"""

from __future__ import annotations

import argparse

import numpy as np

import arm
from kinematics import ARM_JOINTS, SO101Kinematics

# Refuse targets below this height unless overridden -- the base plate sits at
# z=0, so anything lower is usually the table.
Z_FLOOR = 0.02

GRIPPER_PRESETS = {"open": 90.0, "close": 5.0, "half": 50.0}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=arm.DEFAULT_PORT)
    p.add_argument("--id", default=arm.DEFAULT_ID)
    p.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"), help="target tool position, metres")
    p.add_argument("--gripper", help="open | close | half | a number 0-100")
    p.add_argument("--show", action="store_true", help="report current joints and tool position, then exit")
    p.add_argument("--duration", type=float, default=2.0)
    p.add_argument("--max-step", type=float, default=12.0)
    p.add_argument("--allow-low", action="store_true", help=f"permit targets below z={Z_FLOOR}m")
    p.add_argument("--wait", type=float, default=0.0, help="seconds to wait for the servos to appear")
    p.add_argument("--dry-run", action="store_true", help="solve and print, but do not move")
    args = p.parse_args()

    if not (args.xyz or args.gripper or args.show):
        p.error("nothing to do: pass --xyz, --gripper, or --show")

    kin = SO101Kinematics()

    # Solve IK before touching the hardware, so a bad target costs nothing.
    target_q = None
    if args.xyz:
        target = np.array(args.xyz, dtype=float)
        if target[2] < Z_FLOOR and not args.allow_low:
            print(f"ERROR: z={target[2]:.3f} is below the {Z_FLOOR} m floor. Pass --allow-low to override.")
            return 1

    missing = arm.preflight(args.port, args.wait)
    if missing:
        print(f"ERROR: motor ids {missing} are not responding on {args.port}.")
        print("Check the 12V supply -- USB powers the board, not the servos.")
        return 1

    robot = arm.connect(args.port, args.id, max_step=args.max_step)
    try:
        joints = arm.read_joints(robot)
        q_now = arm.arm_vector(joints)
        pos_now = kin.fk_position(q_now)

        print("current joints (deg):")
        for name in ARM_JOINTS:
            print(f"    {name:<15}{joints[name]:>8.2f}")
        print(f"    {'gripper':<15}{joints['gripper']:>8.2f}   (0-100)")
        print(f"\ncurrent tool xyz: [{pos_now[0]:+.4f} {pos_now[1]:+.4f} {pos_now[2]:+.4f}] m")

        if args.show:
            return 0

        goal = {}

        if args.xyz:
            # Seed from the present pose so the solution stays near where we are.
            target_q, ok, err = kin.ik(target, q_init=q_now)
            print(f"\nIK target      : [{target[0]:+.4f} {target[1]:+.4f} {target[2]:+.4f}] m")
            print(f"IK residual    : {err * 1000:.3f} mm  ({'converged' if ok else 'best effort'})")
            if err > 0.005:
                print(f"ERROR: cannot reach that point (off by {err * 1000:.1f} mm). Out of workspace?")
                return 1
            print("solved joints (deg):")
            for name, v in zip(ARM_JOINTS, target_q):
                lo, hi = kin.limits_deg[name]
                flag = "  <-- AT LIMIT" if (v <= lo + 0.5 or v >= hi - 0.5) else ""
                print(f"    {name:<15}{v:>8.2f}   [{lo:>6.1f},{hi:>6.1f}]{flag}")
            goal.update(dict(zip(ARM_JOINTS, target_q)))

        if args.gripper is not None:
            g = GRIPPER_PRESETS.get(args.gripper)
            if g is None:
                try:
                    g = float(args.gripper)
                except ValueError:
                    print(f"ERROR: --gripper must be a number or one of {list(GRIPPER_PRESETS)}")
                    return 1
            g = float(np.clip(g, 0.0, 100.0))
            print(f"\ngripper target : {g:.1f}")
            goal["gripper"] = g

        if args.dry_run:
            print("\n--dry-run: solved only, nothing commanded.")
            return 0

        print()
        final = arm.ramp_to(robot, goal, duration=args.duration)
        reached = kin.fk_position(arm.arm_vector(final))

        if args.xyz:
            miss = np.linalg.norm(reached - target)
            print(f"\nreached tool xyz: [{reached[0]:+.4f} {reached[1]:+.4f} {reached[2]:+.4f}] m")
            print(f"position error  : {miss * 1000:.1f} mm")
            if miss > 0.02:
                print("NOTE: >20mm. Usually gravity droop under load, or a joint against a limit.")
        if args.gripper is not None:
            print(f"gripper reached : {final['gripper']:.1f}")
    finally:
        robot.disconnect()
        print("\ndisconnected -- torque OFF, support the arm if it can fall")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
