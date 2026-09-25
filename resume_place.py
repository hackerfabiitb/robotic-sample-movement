r"""Finish the pick: carry the already-gripped box to the mat and set it down.

Used when a run stopped with the box in the jaws. Attaches at the bus level
(see pick_box.attach) so torque is never dropped and the box is not let go.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

import arm
from calibrate_camera import pixel_to_plane
from jog import shoot
from kinematics import SO101Kinematics
from lerobot.robots.so_follower import SO101FollowerConfig
from pick_box import (BOX_HEIGHT, GRIP_OPEN, MAT_PIXEL, MAT_THICKNESS, Z_HOVER,
                      Z_LIFT, attach, grip_centre, jaw, move)

p = argparse.ArgumentParser()
p.add_argument("--grip", type=float, default=5.0)
p.add_argument("--steps", type=int, default=5)
args = p.parse_args()

kin = SO101Kinematics()
P = np.asarray(json.loads(Path("camera_calib.json").read_text())["projection"], float)
mat = pixel_to_plane(P, *MAT_PIXEL, z=0.0)
place = np.array([mat[0], mat[1], MAT_THICKNESS + BOX_HEIGHT * 0.5 + 0.004])

robot = attach(SO101FollowerConfig(port=arm.DEFAULT_PORT, id=arm.DEFAULT_ID,
                                   max_relative_target=None,
                                   disable_torque_on_disconnect=False))
try:
    q = arm.arm_vector(arm.read_joints(robot))
    here = grip_centre(kin, q, args.grip)
    print(f"holding at grip centre ({here[0]:.4f}, {here[1]:.4f}, {here[2]:.4f})")
    print(f"mat at ({mat[0]:.4f}, {mat[1]:.4f}), placing at z={place[2]:.3f}")

    q = move(robot, kin, "across to the mat", [mat[0], mat[1], Z_LIFT], args.grip, q,
             duration=4.0, steps=args.steps)
    shoot("over_mat")
    q = move(robot, kin, "down onto the mat", place, args.grip, q, duration=2.0, steps=2)
    jaw(robot, GRIP_OPEN, "release")
    shoot("released")
    q = move(robot, kin, "back off", [mat[0], mat[1], Z_HOVER], GRIP_OPEN, q,
             duration=2.0, steps=2)
    shoot("done")
    print("done")
finally:
    robot.disconnect()
