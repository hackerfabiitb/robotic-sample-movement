r"""Shared connect/read/move helpers for the SO-101 follower.

Keeps the safety behaviour in one place: torque is asserted at the current pose
on connect, every move is interpolated rather than stepped, and torque is always
released on the way out.
"""

from __future__ import annotations

import time

import numpy as np

from kinematics import ARM_JOINTS
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

DEFAULT_PORT = "COM3"
DEFAULT_ID = "arm0"

# The gripper is geared slowly and lags its goal badly, so a cap tight enough to
# be useful on the arm joints throttles it every single step. lerobot accepts a
# per-motor mapping, so give the jaw plenty of room and keep the arm restrained.
GRIPPER_MAX_STEP = 100.0

# Servo tuning, measured on this arm (see README "Jitter").
#
# lerobot writes Acceleration = Maximum_Acceleration = 254, i.e. no smoothing at
# all, so each commanded goal is chased at full acceleration. Backing that off
# measured ~25% smoother motion AND less lag (1.23 deg -> 0.75 deg), because the
# servo tracks the ramp instead of snapping past it.
#
# D = 0 was marginally better than lerobot's 32. P is left at lerobot's 16;
# raising it to 32 measurably worsened both roughness and at-rest hold.
SERVO_ACCELERATION = 24
SERVO_D_COEFFICIENT = 0


def smoothstep(a: float) -> float:
    return a * a * (3.0 - 2.0 * a)


def preflight(port: str = DEFAULT_PORT, timeout: float = 0.0) -> list[int]:
    """Return the ids that are NOT answering, so callers can fail readably."""
    from lerobot.motors.feetech import FeetechMotorsBus

    deadline = time.time() + timeout
    while True:
        bus = FeetechMotorsBus(port, {})
        try:
            bus._connect(handshake=False)
            bus.set_baudrate(bus.default_baudrate)
            bus.set_timeout(50)
            missing = [i for i in range(1, 7) if bus.ping(i) is None]
        finally:
            try:
                bus.port_handler.closePort()
            except Exception:
                pass
        if not missing or time.time() >= deadline:
            return missing
        print(f"  waiting for motor ids {missing} ...")
        time.sleep(2.0)


def expand_max_step(max_step: float | dict | None) -> dict | None:
    """Turn a scalar cap into a per-motor mapping that spares the gripper."""
    if max_step is None or isinstance(max_step, dict):
        return max_step
    limits = {name: float(max_step) for name in ARM_JOINTS}
    limits["gripper"] = GRIPPER_MAX_STEP
    return limits


def connect(port: str = DEFAULT_PORT, robot_id: str = DEFAULT_ID, max_step: float | None = 12.0):
    """Connect and immediately re-assert the present pose as the goal.

    Torque switches ON during connect, so without this a stale Goal_Position left
    in the servos from a previous session could yank the arm on the first write.
    """
    robot = SO101Follower(
        SO101FollowerConfig(port=port, id=robot_id, max_relative_target=expand_max_step(max_step))
    )
    robot.connect(calibrate=False)
    tune_servos(robot)
    robot.send_action({f"{k}.pos": v for k, v in read_joints(robot).items()})
    time.sleep(0.1)
    return robot


def tune_servos(robot, acceleration: int = SERVO_ACCELERATION,
                d_coefficient: int = SERVO_D_COEFFICIENT) -> None:
    """Re-tune for smoother motion, overriding what `configure()` just wrote.

    Must run after connect: lerobot's own `configure()` sets acceleration to the
    maximum every time, so this has to come afterwards to stick.
    """
    was_enabled = True
    try:
        robot.bus.disable_torque()
        was_enabled = False
        for motor in robot.bus.motors:
            robot.bus.write("Acceleration", motor, acceleration, num_retry=5)
            robot.bus.write("Maximum_Acceleration", motor, acceleration, num_retry=5)
            robot.bus.write("D_Coefficient", motor, d_coefficient, num_retry=5)
    finally:
        if not was_enabled:
            robot.bus.enable_torque()


def read_joints(robot) -> dict[str, float]:
    return {k.removesuffix(".pos"): v for k, v in robot.get_observation().items() if k.endswith(".pos")}


def arm_vector(joints: dict[str, float]) -> np.ndarray:
    """The five positioning joints, ordered to match the kinematics chain."""
    return np.array([joints[n] for n in ARM_JOINTS])


def ramp_to(robot, target: dict[str, float], duration: float = 3.0, rate: float = 30.0,
            verbose: bool = True) -> dict[str, float]:
    """Interpolate every named joint from where it is now to `target`, then settle."""
    start = read_joints(robot)
    # Command every motor on every step, holding the ones not named at their
    # current position. lerobot requires a dict-valued max_relative_target to
    # have exactly the same keys as the action, and holding is safer anyway.
    goal = dict(start)
    goal.update({k: v for k, v in target.items() if k in start})

    n = max(1, int(duration * rate))
    period = 1.0 / rate
    if verbose:
        print(f"  moving over {duration:.1f}s ({n} steps)")

    for i in range(1, n + 1):
        a = smoothstep(i / n)
        robot.send_action({f"{k}.pos": start[k] + (v - start[k]) * a for k, v in goal.items()})
        time.sleep(period)

    time.sleep(0.4)
    return read_joints(robot)
