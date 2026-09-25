r"""Step the arm to a Cartesian target with the wrist roll pinned, and look.

Built for working a task out interactively: each run moves the tool somewhere,
optionally works the gripper, photographs both cameras, and **leaves torque on**
so the arm holds its pose until the next command. Nothing about it is autonomous.

Why the roll is pinned
----------------------
The wrist camera is bolted to one side of the wrist. Position-only IK treats
wrist_roll as free and will happily return a solution ~99 deg away, which rolls
the camera from the top of the wrist to the underside, where it fouls the arm --
this is exactly how the camera came to hit the arm once already. So every
solution here is constrained to SAFE_ROLL, and the four remaining joints do the
reaching. Verified: with the roll pinned the tool still reaches the whole bench.

Solutions are seeded from the pose the arm is already in, and a move is refused
if it would need a big jump in joint space. Without that, two Cartesian points a
few centimetres apart can land in completely different arm configurations
(wrist_flex +63 deg vs -90 deg was measured) and the arm slams between them.

Usage::

    .venv\python.exe jog.py --xyz 0.244 -0.037 0.08          # move there
    .venv\python.exe jog.py --gripper 45                     # just the jaw
    .venv\python.exe jog.py --xyz 0.244 -0.037 0.02 --gripper 8
    .venv\python.exe jog.py --show                           # photograph, move nothing
    .venv\python.exe jog.py --relax                          # torque off (arm sags)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

import arm
from kinematics import ARM_JOINTS, SO101Kinematics
from lerobot.cameras.configs import Cv2Backends
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# The roll that keeps the wrist camera on top. Read off the arm in the pose the
# user confirmed as safe; do not change it without checking the camera by eye.
SAFE_ROLL = -99.0

# Refuse a move needing more than this much travel on any single joint. A large
# number here means IK has jumped to a different branch, not that the target is
# far away.
MAX_JOINT_JUMP = 75.0

FREE_JOINTS = [0, 1, 2, 3]          # pan, lift, elbow, wrist_flex
Z_FLOOR = 0.005                     # never command the tool below this

SHOT_DIR = Path(__file__).parent / "outputs" / "jog"


def ik_fixed_roll(kin: SO101Kinematics, target, q_init, roll: float = SAFE_ROLL,
                  iters: int = 400, restarts: int = 10, seed: int = 0):
    """Damped least squares on the four joints that are not wrist_roll.

    Four joints for three constraints, so there is still a redundant degree of
    freedom; seeding from the current pose is what picks a nearby solution out
    of it rather than an arbitrary one.
    """
    rng = np.random.default_rng(seed)
    target = np.asarray(target, dtype=float)
    best_q, best_err = None, np.inf
    for attempt in range(restarts):
        q = np.asarray(q_init, dtype=float).copy()
        if attempt > 0:                       # restarts only if the seed failed
            q[FREE_JOINTS] += rng.uniform(-45, 45, 4)
        q[4] = roll
        for _ in range(iters):
            err = target - kin.fk_position(q)
            if np.linalg.norm(err) < 1e-5:
                break
            J = kin.jacobian(q)[:, FREE_JOINTS]
            dq = J.T @ np.linalg.solve(J @ J.T + 0.05**2 * np.eye(3), err)
            q[FREE_JOINTS] = np.clip(q[FREE_JOINTS] + np.degrees(dq) * 0.5, -110, 110)
            q[4] = roll
        e = float(np.linalg.norm(target - kin.fk_position(q)))
        if e < best_err:
            best_q, best_err = q.copy(), e
        if best_err < 1e-4 and attempt == 0:
            break                             # the seeded solve was good enough
    return best_q, best_err


def shoot(tag: str) -> None:
    SHOT_DIR.mkdir(parents=True, exist_ok=True)
    import cv2
    for index, name in ((0, "top"), (1, "wrist")):
        cam = OpenCVCamera(OpenCVCameraConfig(
            index_or_path=index, fps=30, width=1280, height=720,
            fourcc="MJPG", backend=Cv2Backends.DSHOW, warmup_s=1))
        cam.connect()
        try:
            for _ in range(8):
                frame = cam.async_read(timeout_ms=2000)
            path = SHOT_DIR / f"{tag}_{name}.png"
            cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            print(f"  wrote {path}")
        finally:
            cam.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--xyz", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"))
    parser.add_argument("--gripper", type=float, default=None, help="0 closed .. 100 open")
    parser.add_argument("--duration", type=float, default=2.5, help="seconds for the move")
    parser.add_argument("--settle", type=int, default=2,
                        help="corrective passes to cancel gravity droop (0 to disable)")
    parser.add_argument("--tag", default=None, help="filename prefix for the photos")
    parser.add_argument("--show", action="store_true", help="photograph only, move nothing")
    parser.add_argument("--no-shot", action="store_true", help="skip the photos")
    parser.add_argument("--relax", action="store_true", help="torque off and exit")
    parser.add_argument("--port", default=arm.DEFAULT_PORT)
    parser.add_argument("--id", default=arm.DEFAULT_ID)
    args = parser.parse_args()

    kin = SO101Kinematics()
    # Leave torque on when we disconnect: this is a step in a sequence, and the
    # arm dropping between steps would lose the pose (and anything it is holding).
    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.id, max_relative_target=None,
        disable_torque_on_disconnect=args.relax))
    robot.connect(calibrate=False)
    try:
        # Read the pose BEFORE tuning. tune_servos has to drop torque to write
        # the acceleration registers, and the arm sags during that window --
        # reading afterwards captures the sag and then re-asserts it as the goal,
        # so the arm ratchets downward a little on every single command.
        held = arm.read_joints(robot)
        arm.tune_servos(robot)
        robot.send_action({f"{k}.pos": v for k, v in held.items()})
        time.sleep(0.4)
        joints = arm.read_joints(robot)
        q_now = arm.arm_vector(joints)
        here = kin.fk_position(q_now)
        print(f"now: tool ({here[0]:.4f}, {here[1]:.4f}, {here[2]:.4f})  "
              f"roll {joints['wrist_roll']:.1f}  gripper {joints['gripper']:.1f}")

        if args.relax:
            robot.bus.disable_torque()
            print("torque OFF -- the arm will sag")
            return 0

        if args.show:
            shoot(args.tag or "show")
            return 0

        # Hold the current pose before anything else, so enabling torque cannot
        # snap the arm toward a stale goal.
        robot.send_action({f"{k}.pos": v for k, v in joints.items()})
        time.sleep(0.1)

        if args.xyz is not None:
            target = np.asarray(args.xyz, dtype=float)
            if target[2] < Z_FLOOR:
                raise SystemExit(f"z={target[2]:.3f} is below the {Z_FLOOR:.3f} floor")
            q, err = ik_fixed_roll(kin, target, q_now)
            if err > 0.003:
                raise SystemExit(f"unreachable with the roll pinned "
                                 f"(off by {err * 1000:.1f} mm)")
            jump = np.abs(q[FREE_JOINTS] - q_now[FREE_JOINTS]).max()
            if jump > MAX_JOINT_JUMP:
                worst = ARM_JOINTS[int(np.argmax(np.abs(q[FREE_JOINTS] - q_now[FREE_JOINTS])))]
                raise SystemExit(
                    f"refusing: needs {jump:.0f} deg on {worst}, which is an IK "
                    "branch change rather than a short move. Go via an "
                    "intermediate point.")
            print(f"  solve: {np.round(q, 1)}  (err {err * 1000:.2f} mm, "
                  f"largest joint move {jump:.0f} deg)")
            arm.ramp_to(robot, dict(zip(ARM_JOINTS, q)), duration=args.duration,
                        verbose=False)

            # The arm settles low: gravity droops the loaded joints, so where it
            # ends up is reliably a centimetre or so under where it was sent.
            # Measure the miss and aim past it by the same amount. Two passes
            # take ~15mm of error down to ~1mm, which matters when the jaw has
            # to clear a box only 20mm tall.
            command = target.copy()
            for _ in range(args.settle):
                reached = kin.fk_position(arm.arm_vector(arm.read_joints(robot)))
                miss = target - reached
                if np.linalg.norm(miss) < 0.003:
                    break
                # Under-relaxed. At full gain this oscillates: the servos do not
                # respond 1:1 to a shifted command once backlash is taken up, so
                # correcting the whole miss overshoots and the next pass chases
                # it back the other way.
                command = command + 0.6 * miss
                q_fix, e_fix = ik_fixed_roll(kin, command, arm.arm_vector(arm.read_joints(robot)))
                if e_fix > 0.003:
                    break
                print(f"  settling: off by {np.linalg.norm(miss) * 1000:.1f} mm, "
                      f"re-aiming at ({command[0]:.4f}, {command[1]:.4f}, {command[2]:.4f})")
                arm.ramp_to(robot, dict(zip(ARM_JOINTS, q_fix)), duration=1.2,
                            verbose=False)

        if args.gripper is not None:
            arm.ramp_to(robot, {"gripper": float(args.gripper)}, duration=1.0,
                        verbose=False)
            time.sleep(0.3)

        joints = arm.read_joints(robot)
        reached = kin.fk_position(arm.arm_vector(joints))
        print(f"reached: ({reached[0]:.4f}, {reached[1]:.4f}, {reached[2]:.4f})  "
              f"roll {joints['wrist_roll']:.1f}  gripper {joints['gripper']:.1f}")
        if args.xyz is not None:
            print(f"  position error {np.linalg.norm(reached - np.asarray(args.xyz)) * 1000:.1f} mm")
    finally:
        robot.disconnect()

    if not args.no_shot:
        shoot(args.tag or "jog")
    print("torque still ON -- the arm is holding its pose")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
