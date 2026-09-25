r"""Report where the gripper is, in the overhead image and in the world.

Opens and closes the jaw and subtracts the two frames, so the only thing left is
the jaw itself -- the same trick calibrate_camera.py uses, but for a single pose,
without moving the arm anywhere. Prints the jaw's pixel alongside the tool point
and jaw position from forward kinematics.

Two of these at different heights over the same (x, y) give the parallax
gradient: how far a fixed point appears to slide across the image per metre of
height. That is what makes it possible to convert the pixel of something lying
on the bench into a position the arm can reach, using a calibration that was
fitted at gripper height rather than table height.

Usage::

    .venv\python.exe probe.py                 # probe here
    .venv\python.exe probe.py --json out.json # and append the result to a file
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

import arm
from calibrate_camera import (GRIP_CLOSED, GRIP_OPEN, grab, jaw_centroid, jaw_world_point,
                              open_camera)
from kinematics import SO101Kinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

CYCLES = 3
AGREE_PX = 25.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--json", default=None, help="append the reading to this file")
    parser.add_argument("--label", default="", help="note stored with the reading")
    parser.add_argument("--debug-dir", default=None)
    parser.add_argument("--port", default=arm.DEFAULT_PORT)
    parser.add_argument("--id", default=arm.DEFAULT_ID)
    args = parser.parse_args()

    debug_dir = Path(args.debug_dir) if args.debug_dir else None
    if debug_dir:
        debug_dir.mkdir(parents=True, exist_ok=True)

    kin = SO101Kinematics()
    cam = open_camera()
    # Torque stays on through this: the probe is a step inside a sequence and the
    # arm must not sag between the measurement and whatever follows it.
    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.id, max_relative_target=None,
        disable_torque_on_disconnect=False))
    robot.connect(calibrate=False)
    try:
        # See the note in jog.py: tune_servos drops torque briefly and the arm
        # sags, so capture the held pose first and put it straight back.
        held = arm.read_joints(robot)
        arm.tune_servos(robot)
        robot.send_action({f"{k}.pos": v for k, v in held.items()})
        time.sleep(0.4)
        seen = []
        for c in range(CYCLES):
            arm.ramp_to(robot, {"gripper": GRIP_OPEN}, duration=0.6, verbose=False)
            time.sleep(0.4)
            opened = grab(cam)
            arm.ramp_to(robot, {"gripper": GRIP_CLOSED}, duration=0.6, verbose=False)
            time.sleep(0.4)
            closed = grab(cam)
            dbg = (debug_dir / f"probe_{c}.png") if debug_dir else None
            found = jaw_centroid(opened, closed, dbg)
            if found is not None:
                seen.append(found)

        q = arm.arm_vector(arm.read_joints(robot))
        tcp = kin.fk_position(q)
        jaw = jaw_world_point(kin, q)
        # Leave the jaw open, which is the state a following approach wants.
        arm.ramp_to(robot, {"gripper": GRIP_OPEN}, duration=0.6, verbose=False)
    finally:
        robot.disconnect()
        cam.disconnect()

    if not seen:
        raise SystemExit("jaw not found in any difference image")
    pts = np.asarray(seen)
    median = np.median(pts, axis=0)
    agree = pts[np.linalg.norm(pts - median, axis=1) <= AGREE_PX]
    if len(agree) < 2:
        raise SystemExit(f"jaw pixel disagreed across cycles: {np.round(pts, 1).tolist()}")
    pixel = agree.mean(axis=0)

    print(f"jaw pixel   ({pixel[0]:7.1f}, {pixel[1]:7.1f})   from {len(agree)}/{CYCLES} cycles")
    print(f"tool point  ({tcp[0]:7.4f}, {tcp[1]:7.4f}, {tcp[2]:7.4f})")
    print(f"jaw world   ({jaw[0]:7.4f}, {jaw[1]:7.4f}, {jaw[2]:7.4f})")

    if args.json:
        path = Path(args.json)
        rows = json.loads(path.read_text()) if path.exists() else []
        rows.append({"label": args.label, "pixel": pixel.tolist(),
                     "tcp": tcp.tolist(), "jaw": jaw.tolist(),
                     "joints": q.tolist(), "when": time.time()})
        path.write_text(json.dumps(rows, indent=2))
        print(f"appended to {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
