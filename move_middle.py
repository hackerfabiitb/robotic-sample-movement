#!/usr/bin/env python
r"""Move every joint to the middle of its calibrated range.

A first motion test: it exercises all six servos over a short, slow, fully
interpolated path, so you can confirm each one is wired, powered and responding.

    .venv\python.exe move_middle.py --dry-run    # read only, no motion
    .venv\python.exe move_middle.py              # actually move

The "middle" comes straight from the calibration file's range_min/range_max.
Note that lerobot's normalization already centres on that midpoint, so the
command values are 0.0 deg for the five body joints and 50.0 for the gripper --
both of which unnormalize back to raw encoder (range_min + range_max) / 2.
"""

import argparse
import json
import time

from lerobot.motors.motors_bus import MotorNormMode
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS

# Value that maps to the centre of the calibrated range, per normalization mode.
MIDDLE = {
    MotorNormMode.DEGREES: 0.0,
    MotorNormMode.RANGE_M100_100: 0.0,
    MotorNormMode.RANGE_0_100: 50.0,
}


def smoothstep(a: float) -> float:
    """Ease-in/ease-out so the arm doesn't jerk at the start or overshoot at the end."""
    return a * a * (3.0 - 2.0 * a)


def wait_for_bus(port: str, timeout: float = 0.0) -> list[int]:
    """Ping ids 1-6 until every one answers or `timeout` elapses; return missing ids.

    Gives a readable diagnosis instead of lerobot's handshake traceback, and lets
    the script sit waiting for you to switch the servo supply on.
    """
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


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="COM3")
    p.add_argument("--id", default="arm0", help="robot id, i.e. calibration file name")
    p.add_argument("--duration", type=float, default=5.0, help="seconds for the move (default: 5)")
    p.add_argument("--rate", type=float, default=30.0, help="command Hz (default: 30)")
    p.add_argument(
        "--max-step",
        type=float,
        default=12.0,
        help="safety cap on how far a goal may lead the measured position (default: 12)",
    )
    p.add_argument("--dry-run", action="store_true", help="connect and read, but never move")
    p.add_argument("--wait", type=float, default=0.0,
                   help="seconds to wait for the servos to appear (e.g. while you switch power on)")
    args = p.parse_args()

    missing = wait_for_bus(args.port, args.wait)
    if missing:
        print(f"ERROR: motor ids {missing} are not responding on {args.port}.")
        print()
        print("The USB adapter itself is fine -- the COM port exists, which is why this")
        print("isn't a cable-or-driver problem. But the servos are silent, and they are")
        print("powered by the 12V barrel jack, NOT by USB. Check that the supply is on")
        print("and the jack is seated, then re-run. Use --wait 60 to have this script")
        print("poll while you switch it on.")
        return 1

    calib_path = HF_LEROBOT_CALIBRATION / ROBOTS / "so_follower" / f"{args.id}.json"
    calib = json.loads(calib_path.read_text())
    print(f"calibration: {calib_path}\n")

    config = SO101FollowerConfig(
        port=args.port,
        id=args.id,
        max_relative_target=None if args.dry_run else args.max_step,
    )
    robot = SO101Follower(config)

    # calibrate=False: never fall into the interactive re-calibration prompt.
    robot.connect(calibrate=False)
    print("connected -- torque is now ON, the arm is holding position\n")

    try:
        obs = robot.get_observation()
        start = {k.removesuffix(".pos"): v for k, v in obs.items() if k.endswith(".pos")}

        target, units = {}, {}
        for name, motor in robot.bus.motors.items():
            target[name] = MIDDLE[motor.norm_mode]
            units[name] = "deg" if motor.norm_mode is MotorNormMode.DEGREES else "0-100"

        print(f"{'joint':<15}{'raw min':>9}{'raw max':>9}{'raw mid':>9}   "
              f"{'current':>10}{'target':>10}  unit")
        print("-" * 78)
        for name in robot.bus.motors:
            c = calib[name]
            raw_mid = (c["range_min"] + c["range_max"]) / 2
            print(f"{name:<15}{c['range_min']:>9}{c['range_max']:>9}{raw_mid:>9.0f}   "
                  f"{start[name]:>10.2f}{target[name]:>10.2f}  {units[name]}")

        if args.dry_run:
            print("\n--dry-run: all six joints read back OK, no motion commanded.")
            return 0

        # Re-assert where the arm already is, so any stale goal left in the servos
        # from a previous session can't yank it somewhere unexpected.
        robot.send_action({f"{k}.pos": v for k, v in start.items()})
        time.sleep(0.1)

        n = max(1, int(args.duration * args.rate))
        period = 1.0 / args.rate
        print(f"\nmoving over {args.duration:.1f}s in {n} steps -- Ctrl+C or cut power to abort")

        for i in range(1, n + 1):
            a = smoothstep(i / n)
            action = {f"{k}.pos": start[k] + (target[k] - start[k]) * a for k in target}
            robot.send_action(action)
            time.sleep(period)

        time.sleep(0.4)  # let the servos settle before reading back
        final = {k.removesuffix(".pos"): v for k, v in robot.get_observation().items()
                 if k.endswith(".pos")}

        print(f"\n{'joint':<15}{'target':>10}{'reached':>10}{'error':>10}")
        print("-" * 45)
        worst = 0.0
        for name in robot.bus.motors:
            err = final[name] - target[name]
            worst = max(worst, abs(err))
            print(f"{name:<15}{target[name]:>10.2f}{final[name]:>10.2f}{err:>10.2f}")
        print(f"\nlargest error: {worst:.2f}")
        if worst > 5.0:
            print("NOTE: >5 off. Likely gravity droop on an unsupported joint, or a")
            print("joint that hit a mechanical limit. Check which one above.")
    finally:
        robot.disconnect()  # disable_torque_on_disconnect=True -> arm goes limp
        print("disconnected -- torque OFF, support the arm if it can fall")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
