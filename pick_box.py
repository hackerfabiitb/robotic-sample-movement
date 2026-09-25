r"""Pick the transparent box off the bench and set it down on the red mat.

Runs the whole sequence in a single connection, deliberately. Each time a
process connects, lerobot's `configure()` drops torque briefly and the arm sags
several centimetres before the goal is re-asserted -- harmless between moves,
but it would drop whatever the gripper is holding. So once the jaw closes on the
box, nothing disconnects until the box is down.

Two things this gets right that the obvious version does not:

*Aim the grip centre, not the tool point.* `fk_position` returns the tip of the
*fixed* jaw, which is one side of the gripper, not the middle of it. Driving the
tool point to the box leaves the box against one jaw with the other 35mm away,
which is exactly what happened on the first attempt. The target here is the
midpoint between the two jaw tips at their closed separation, solved for by
iterating the tool-point command until the grip centre lands on the object.

*Pin the wrist roll.* The wrist camera is on one side of the wrist, and
position-only IK treats roll as free -- it will return a solution ~99 deg away
that rolls the camera underneath, where it fouls the arm. Every pose here is
solved with roll pinned to jog.SAFE_ROLL.

Usage::

    .venv\python.exe pick_box.py --dry-run     # print the plan, move nothing
    .venv\python.exe pick_box.py
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

import arm
from jog import FREE_JOINTS, SAFE_ROLL, ik_fixed_roll, shoot
from kinematics import ARM_JOINTS, SO101Kinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

CALIB_FILE = Path(__file__).parent / "camera_calib.json"

# Pixel positions read off the overhead camera, converted to the table plane by
# the fitted projective camera (see calibrate_camera.py).
BOX_PIXEL = (757.0, 333.0)
MAT_PIXEL = (895.0, 205.0)

BOX_HEIGHT = 0.020        # the box stands about this tall
MAT_THICKNESS = 0.004     # the mat it has to clear on the way down

GRIP_OPEN = 45.0          # ~68mm between the jaw tips: clears a ~33mm box
GRIP_CLOSE = 5.0          # ~30mm: a few mm of squeeze on the box

Z_HOVER = 0.090           # travel height for the grip centre
Z_LIFT = 0.080            # how high to carry the box
MAX_JOINT_JUMP = 75.0


def grip_centre(kin: SO101Kinematics, q: np.ndarray, gripper: float) -> np.ndarray:
    """Midpoint between the two jaw tips -- where an object actually ends up."""
    g = kin.gripper_geometry(q, gripper)
    return 0.5 * (g["fixed"][1] + g["moving"][2])


def solve_for_grip(kin: SO101Kinematics, target, q_seed, gripper: float,
                   passes: int = 12):
    """Tool-point command that puts the *grip centre* on `target`.

    The offset between the tool point and the grip centre is not constant: it
    rotates with the arm, so it cannot be subtracted once. Iterating converges
    in a handful of passes.
    """
    target = np.asarray(target, dtype=float)
    command = target.copy()
    q = np.asarray(q_seed, dtype=float)
    for _ in range(passes):
        q, err = ik_fixed_roll(kin, command, q)
        if err > 0.003:
            return None, None, err
        miss = target - grip_centre(kin, q, gripper)
        if np.linalg.norm(miss) < 0.0005:
            break
        command = command + miss
    return q, command, float(np.linalg.norm(target - grip_centre(kin, q, gripper)))


def attach(config: SO101FollowerConfig) -> SO101Follower:
    """Open the bus WITHOUT running configure(), so torque is never dropped.

    `SO101Follower.connect()` calls `configure()`, which does all its register
    writes inside `with self.bus.torque_disabled()`. The arm sags several
    centimetres during that window -- and if the gripper is holding something,
    it drops it. Everything configure() would write (acceleration, PID, the
    gripper's torque limits) is already set from the session that opened the
    bus before this one, so skipping it costs nothing and keeps the arm up.
    """
    robot = SO101Follower(config)
    robot.bus.connect()
    return robot


def move(robot, kin, label: str, target, gripper: float, q_now, duration=2.5,
         settle=2, dry=False, steps=1):
    """Put the grip centre at `target`, correcting for the arm settling low.

    `steps` splits the path into that many Cartesian sub-moves, each solved
    seeded from the pose before it. A single long move across the bench asks IK
    for a pose far from the seed and it can come back on a different branch --
    a ~85 deg joint swing for two points 16cm apart, which the jump check
    rightly refuses. Walking there in short hops keeps the same branch.
    """
    if steps > 1:
        start = grip_centre(kin, np.asarray(q_now, dtype=float), gripper)
        target = np.asarray(target, dtype=float)
        for i in range(1, steps + 1):
            waypoint = start + (target - start) * (i / steps)
            q_now = move(robot, kin, f"{label} [{i}/{steps}]", waypoint, gripper,
                         q_now, duration=max(0.8, duration / steps),
                         settle=(settle if i == steps else 0), dry=dry)
        return q_now

    q, command, err = solve_for_grip(kin, target, q_now, gripper)
    if q is None or err > 0.003:
        raise SystemExit(f"{label}: cannot place the grip centre there "
                         f"(off by {(err or 9.9) * 1000:.1f} mm)")
    jump = float(np.abs(q[FREE_JOINTS] - np.asarray(q_now)[FREE_JOINTS]).max())
    if jump > MAX_JOINT_JUMP:
        raise SystemExit(f"{label}: needs {jump:.0f} deg on one joint -- that is "
                         "an IK branch change, not a short move")
    print(f"  {label}: grip centre -> ({target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f})"
          f"  [{jump:.0f} deg]")
    if dry:
        return q

    arm.ramp_to(robot, dict(zip(ARM_JOINTS, q)), duration=duration, verbose=False)
    command = np.asarray(command, dtype=float)
    for _ in range(settle):
        q_is = arm.arm_vector(arm.read_joints(robot))
        miss = np.asarray(target, dtype=float) - grip_centre(kin, q_is, gripper)
        if np.linalg.norm(miss) < 0.003:
            break
        command = command + 0.6 * miss       # under-relaxed; full gain oscillates
        q_fix, e = ik_fixed_roll(kin, command, q_is)
        if e > 0.003:
            break
        arm.ramp_to(robot, dict(zip(ARM_JOINTS, q_fix)), duration=1.2, verbose=False)

    q_is = arm.arm_vector(arm.read_joints(robot))
    reached = grip_centre(kin, q_is, gripper)
    print(f"      reached ({reached[0]:.4f}, {reached[1]:.4f}, {reached[2]:.4f})"
          f"   off by {np.linalg.norm(np.asarray(target) - reached) * 1000:.1f} mm")
    return q_is


def jaw(robot, value: float, label: str, dry=False):
    print(f"  {label}: gripper -> {value:.0f}")
    if not dry:
        arm.ramp_to(robot, {"gripper": float(value)}, duration=1.2, verbose=False)
        time.sleep(0.5)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--approach-only", action="store_true",
                        help="stop with the jaws around the box, before closing")
    parser.add_argument("--box", nargs=2, type=float, default=None,
                        help="override the box x y in metres")
    parser.add_argument("--mat", nargs=2, type=float, default=None)
    parser.add_argument("--grip", type=float, default=GRIP_CLOSE)
    parser.add_argument("--steps", type=int, default=5,
                        help="sub-moves for long travels, to hold the IK branch")
    parser.add_argument("--port", default=arm.DEFAULT_PORT)
    parser.add_argument("--id", default=arm.DEFAULT_ID)
    args = parser.parse_args()

    kin = SO101Kinematics()
    from calibrate_camera import pixel_to_plane
    P = np.asarray(json.loads(CALIB_FILE.read_text())["projection"], dtype=float)

    box = (np.array([*args.box, BOX_HEIGHT * 0.5]) if args.box else
           pixel_to_plane(P, *BOX_PIXEL, z=BOX_HEIGHT * 0.5))
    mat = (np.array([*args.mat, 0.0]) if args.mat else
           pixel_to_plane(P, *MAT_PIXEL, z=0.0))
    # Set the box down with its base just clear of the mat, then let go.
    place = np.array([mat[0], mat[1], MAT_THICKNESS + BOX_HEIGHT * 0.5 + 0.004])

    print(f"box  at ({box[0]:.4f}, {box[1]:.4f})   pick at z={box[2]:.3f}")
    print(f"mat  at ({mat[0]:.4f}, {mat[1]:.4f})   place at z={place[2]:.3f}")

    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.id, max_relative_target=None,
        disable_torque_on_disconnect=False))
    if not args.dry_run:
        robot.connect(calibrate=False)
    try:
        if args.dry_run:
            q = np.array([9.0, -50.0, 75.0, 20.0, SAFE_ROLL])
        else:
            held = arm.read_joints(robot)
            arm.tune_servos(robot)
            robot.send_action({f"{k}.pos": v for k, v in held.items()})
            time.sleep(0.4)
            q = arm.arm_vector(arm.read_joints(robot))
            print(f"starting from {np.round(q, 1)}")

        jaw(robot, GRIP_OPEN, "open the jaw", args.dry_run)
        q = move(robot, kin, "above the box", [box[0], box[1], Z_HOVER],
                 GRIP_OPEN, q, dry=args.dry_run, steps=args.steps)
        q = move(robot, kin, "down to the box", box, GRIP_OPEN, q,
                 duration=2.0, dry=args.dry_run)
        # Photographed from inside the sequence, while still connected. A
        # separate process cannot be used to check the pose: connecting sags the
        # arm, so the picture would show something other than what was reached.
        if not args.dry_run:
            shoot("approach")
        if args.approach_only:
            print("stopping before the grip -- check the jaws straddle the box")
            return 0
        jaw(robot, args.grip, "close on the box", args.dry_run)
        q = move(robot, kin, "lift", [box[0], box[1], Z_LIFT], args.grip, q,
                 duration=2.0, dry=args.dry_run, steps=2)
        q = move(robot, kin, "across to the mat", [mat[0], mat[1], Z_LIFT],
                 args.grip, q, duration=4.0, dry=args.dry_run, steps=args.steps)
        if not args.dry_run:
            shoot("over_mat")
        q = move(robot, kin, "down onto the mat", place, args.grip, q,
                 duration=2.0, dry=args.dry_run, steps=2)
        jaw(robot, GRIP_OPEN, "release", args.dry_run)
        q = move(robot, kin, "back off", [mat[0], mat[1], Z_HOVER], GRIP_OPEN, q,
                 duration=2.0, dry=args.dry_run, steps=2)
        if not args.dry_run:
            shoot("placed")
        print("done -- torque left on, arm parked above the mat")
    finally:
        if not args.dry_run:
            robot.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
