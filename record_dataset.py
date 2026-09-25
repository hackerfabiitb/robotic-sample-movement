r"""Record a LeRobot dataset with both cameras, hand-guided or driven by script.

`lerobot-record` refuses to start without a teleoperator::

    A teleoperator is required for recording. Use --teleop.type=... to specify one.

Every arm teleoperator lerobot ships is a *separate leader device* on its own
serial port, and we have one arm. So the stock command cannot run here.

What it can do is take any object implementing the `Teleoperator` interface, and
whatever that object returns each frame is both sent to the arm and written to
the dataset as the action. This script supplies two of them:

`HandGuidedTeleop` (``--source hand``)
    Torque off, arm limp. Reports the follower's present position, so what you
    demonstrate by hand becomes the action. Action and state are then identical
    -- inherent to kinesthetic teaching: with no motor holding a target, the
    position the arm reached *is* the command.

`ScriptedTeleop` (``--source recording:<name>`` or ``--source waypoints``)
    Torque on, arm driving itself through a precomputed joint trajectory. The
    action is the commanded setpoint and the state is where the servo actually
    got to, which is the more useful pair to train on. Collects episodes with
    nobody holding the arm.

Either way `record_loop`, `LeRobotDataset`, the video encoder and the keyboard
controls are lerobot's own code, running unmodified.

Usage::

    # drive the arm through a saved demonstration, 10 episodes, unattended
    .venv\python.exe record_dataset.py --source recording:wafer-station-3 \
        --repo-id local/wafer --task "Move the wafer" --episodes 10 --jitter 0.005

    # run the saved waypoint pick-and-place instead
    .venv\python.exe record_dataset.py --source waypoints --episodes 10

    # pose it yourself
    .venv\python.exe record_dataset.py --source hand --episode-time 30

During each episode:  right arrow = finish early,  left arrow = re-record,
ESC = stop the session.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import arm
import demos
from arm import DEFAULT_ID, DEFAULT_PORT
from kinematics import ARM_JOINTS, SO101Kinematics, pose_matrix
from lerobot.cameras.configs import Cv2Backends
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say

# Camera indices as they enumerate on this machine, confirmed by eye from
# `lerobot-find-cameras opencv` (see README "Recording datasets"): 0 looks down
# over the bench, 1 is bolted to the wrist and sees both jaws.
TOP_CAMERA_INDEX = 0
WRIST_CAMERA_INDEX = 1

# MJPG matters. The default YUY2 is uncompressed, and two uncompressed 1080p
# streams do not fit through one USB controller; with MJPG both cameras hold 30
# fps even at 1920x1080 (measured).
CAMERA_FOURCC = "MJPG"

# The six motors in dataset order. Matches what demos.py records.
RECORD_JOINTS = [*ARM_JOINTS, "gripper"]

# Motion constants, mirroring server.py so a scripted episode moves like the GUI
# does. They are duplicated rather than imported because importing server.py
# would drag in the HTTP server and its module-level state.
T_TRAVEL = 1.5      # seconds per point-to-point move
T_GRIPPER = 0.5     # seconds to open or close the jaw
GRIP_SETTLE = 0.3   # pause after the jaw reaches its target
REPLAY_LOOKAHEAD = 0.10  # read the trajectory this far ahead to cancel servo lag
WAYPOINTS_FILE = Path(__file__).parent / "waypoints.json"

# Peak commanded joint speed for a scripted plan, deg/s. Measured on the saved
# waypoint sequence at Acceleration = 254, mean |action - state| over all joints:
#
#     peak 136 deg/s -> 5.30 deg     peak  68 deg/s -> 2.10 deg
#     peak  91 deg/s -> 2.97 deg     peak  45 deg/s -> 1.38 deg
#
# 60 sits where the curve flattens. Below it the remaining shoulder_lift error is
# static droop under the arm's own weight, not lag, so slowing further buys very
# little and costs episode time.
MAX_JOINT_SPEED = 60.0

# Scripted playback overrides arm.py's Acceleration = 24. That value was tuned to
# stop the arm jittering on slow GUI moves; here the goal is to hit the setpoint,
# and the servo needs the headroom. Measured on the same sequence, mean error:
# accel 24 -> 4.34 deg, 64 -> 4.34, 128 -> 2.98, 254 -> 2.62.
SCRIPTED_ACCELERATION = 254


# --------------------------------------------------------------------------
# Teleoperators
# --------------------------------------------------------------------------

@TeleoperatorConfig.register_subclass("so101_hand_guided")
@dataclass
class HandGuidedTeleopConfig(TeleoperatorConfig):
    pass


@TeleoperatorConfig.register_subclass("so101_scripted")
@dataclass
class ScriptedTeleopConfig(TeleoperatorConfig):
    pass


class _RobotBackedTeleop(Teleoperator):
    """Common base: a teleoperator that borrows the robot's already-open bus.

    The Feetech bus is a single exclusive serial handle, so a teleoperator that
    opened its own connection to COM3 would collide with the robot's. These hold
    a reference instead, and the caller owns connect/disconnect.
    """

    def __init__(self, config: TeleoperatorConfig, robot: SO101Follower):
        super().__init__(config)
        self.robot = robot

    @property
    def action_features(self) -> dict:
        return self.robot.action_features

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        """No-op: the robot owns the bus and is connected by the caller."""

    def calibrate(self) -> None:
        """No-op: the follower's own calibration is the only one involved."""

    def configure(self) -> None:
        """No-op: `configure()` on the robot already set the servos up."""

    def send_feedback(self, feedback: dict) -> None:
        """No-op: there is no leader to give force feedback to."""

    def disconnect(self) -> None:
        """No-op: the caller disconnects the robot, which owns the bus."""


class HandGuidedTeleop(_RobotBackedTeleop):
    """The follower arm itself, back-driven by hand with torque off."""

    config_class = HandGuidedTeleopConfig
    name = "so101_hand_guided"

    def get_action(self) -> dict:
        # Present position, not goal position. With torque off the servos hold
        # no target, so where the arm physically is, is the demonstration.
        return {
            key: value
            for key, value in self.robot.get_observation().items()
            if key.endswith(".pos")
        }


class ScriptedTeleop(_RobotBackedTeleop):
    """Plays a precomputed joint trajectory, one row per recorded frame.

    `record_loop` sends whatever this returns straight to the arm, so returning
    the trajectory *is* driving the robot -- no separate motion thread, and the
    action stored in the dataset is exactly the setpoint the servo was given.
    """

    config_class = ScriptedTeleopConfig
    name = "so101_scripted"

    def __init__(self, config: ScriptedTeleopConfig, robot: SO101Follower,
                 plan: np.ndarray, lookahead_frames: int = 0):
        super().__init__(config, robot)
        self.plan = plan
        self.lookahead_frames = lookahead_frames
        self.index = 0

    def rewind(self) -> None:
        self.index = 0

    @property
    def start_pose(self) -> dict[str, float]:
        return dict(zip(RECORD_JOINTS, self.plan[0]))

    def get_action(self) -> dict:
        # Position-mode servos trail a moving target by a roughly fixed time, so
        # reading the trajectory slightly ahead cancels most of that lag. The
        # shifted setpoint is what gets sent, so it is also what gets recorded.
        i = min(self.index + self.lookahead_frames, len(self.plan) - 1)
        self.index = min(self.index + 1, len(self.plan) - 1)
        return {f"{name}.pos": float(v) for name, v in zip(RECORD_JOINTS, self.plan[i])}


# --------------------------------------------------------------------------
# Trajectory planning
# --------------------------------------------------------------------------

def _segments_to_plan(start: np.ndarray, segments: list[tuple[np.ndarray, float]],
                      fps: int) -> np.ndarray:
    """Expand (target, duration) pairs into one setpoint per frame.

    Eased with the same smoothstep the rest of the repo moves with, so a
    scripted episode accelerates like a GUI move rather than stepping.
    """
    rows: list[np.ndarray] = []
    current = np.asarray(start, dtype=float)
    for target, duration in segments:
        target = np.asarray(target, dtype=float)
        n = max(1, int(round(duration * fps)))
        for i in range(1, n + 1):
            rows.append(current + (target - current) * arm.smoothstep(i / n))
        current = target
    return np.asarray(rows)


def plan_speeds(plan: np.ndarray, fps: int) -> np.ndarray:
    """Peak commanded speed per joint, deg/s."""
    if len(plan) < 2:
        return np.zeros(plan.shape[1])
    return np.abs(np.diff(plan, axis=0)).max(axis=0) * fps


def slow_to_limit(plan: np.ndarray, fps: int, limit: float) -> np.ndarray:
    """Stretch a plan in time until no joint exceeds `MAX_JOINT_SPEED`.

    The shape of the trajectory is preserved exactly; only the clock changes.
    Worth doing automatically because the GUI's own travel time (1.5s per move,
    halved from 3s when the arm was made "2x as fast") is tuned for watching the
    arm work, not for producing action labels the servos can actually hit.
    """
    speeds = plan_speeds(plan, fps)
    ratio = float(speeds.max() / limit) if speeds.max() else 0.0
    if ratio <= 1.0:
        return plan
    n = int(np.ceil(len(plan) * ratio))
    src = np.arange(len(plan))
    dst = np.linspace(0, len(plan) - 1, n)
    stretched = np.stack([np.interp(dst, src, plan[:, c]) for c in range(plan.shape[1])],
                         axis=1)
    print(f"  plan slowed {ratio:.2f}x to stay under {limit:.0f} deg/s "
          f"({len(plan) / fps:.1f}s -> {n / fps:.1f}s)")
    return stretched


def check_plan_speed(plan: np.ndarray, fps: int, limit: float) -> None:
    """Last gate before the arm moves: refuse a trajectory with a jump in it.

    This is what guards the hardware now that `max_relative_target` is off (see
    `build_robot`). Stretching should already have brought a merely-fast plan
    under the limit, so reaching here means something is wrong with its shape --
    most often IK returning a different elbow configuration for one pose, which
    puts a large discontinuity in an otherwise smooth path.
    """
    speeds = plan_speeds(plan, fps)
    if speeds.max() > limit * 1.01:
        worst = RECORD_JOINTS[int(speeds.argmax())]
        raise SystemExit(
            f"plan commands {speeds.max():.0f} deg/s on {worst}, over the "
            f"{limit:.0f} deg/s limit, and stretching did not fix it.\n"
            "A single huge joint swing usually means IK picked a different elbow "
            "configuration for one of the poses."
        )


def plan_from_recording(name: str, fps: int, speed: float) -> np.ndarray:
    """Resample a hand-recorded demonstration onto the dataset's frame clock."""
    data = demos.load_recording(name)
    if not data or not data.get("samples"):
        raise SystemExit(f"recording '{name}' is missing or empty. "
                         f"Available: {[r['name'] for r in demos.list_recordings()]}")
    if data.get("joints", RECORD_JOINTS) != RECORD_JOINTS:
        raise SystemExit(f"recording '{name}' has joints {data.get('joints')}, "
                         f"expected {RECORD_JOINTS}")

    arr = demos.smooth_samples(data["samples"])
    duration = float(arr[-1, 0]) / speed
    n = int(duration * fps)
    return np.asarray([demos.resample(arr, (i / fps) * speed) for i in range(n)])


def _solve(kin: SO101Kinematics, xyz, quat, seed, match_orientation: bool, label: str):
    """IK for one key pose, refusing anything the arm cannot actually reach."""
    if quat is not None and match_orientation:
        q, _, p_err, r_err = kin.ik_pose(pose_matrix(xyz, quat), q_init=seed)
    else:
        q, _, p_err = kin.ik(np.asarray(xyz, dtype=float), q_init=seed)
        r_err = None
    if p_err > 0.005:
        raise SystemExit(f"{label} is unreachable (off by {p_err * 1000:.0f} mm)")
    if r_err is not None and np.degrees(r_err) > 5.0:
        print(f"  note: {label} orientation only reachable to {np.degrees(r_err):.1f} deg")
    return q


def plan_from_waypoints(kin: SO101Kinematics, fps: int, jitter: float,
                        rng: np.random.Generator) -> np.ndarray:
    """Build the GUI's pick-and-place sequence as a joint trajectory.

    Same semantics as `server.run_sequence`: per point, move to it, open,
    retract, move to it again, close, retract. Gripper values are global to the
    sequence; retract moves never touch the jaw, because after closing on an
    object the arm has to lift away still holding it.
    """
    if not WAYPOINTS_FILE.exists():
        raise SystemExit(f"{WAYPOINTS_FILE.name} not found -- capture some points "
                         "in the GUI first (python server.py)")
    saved = json.loads(WAYPOINTS_FILE.read_text())
    waypoints = saved.get("waypoints") or []
    retract = saved.get("retract")
    if not waypoints:
        raise SystemExit("no waypoints saved -- capture some in the GUI first")
    if retract is None:
        raise SystemExit("no retract position saved -- set one in the GUI first")

    grip_open = float(saved.get("gripper_open", 45.0))
    grip_close = float(saved.get("gripper_close", 16.0))
    match_orientation = bool(saved.get("match_orientation", True))

    def jittered(xyz):
        # A dataset of byte-identical episodes teaches a policy nothing about
        # recovering from being slightly off, so shift each point a little.
        if jitter <= 0:
            return list(xyz)
        return list(np.asarray(xyz, dtype=float) + rng.uniform(-jitter, jitter, 3))

    # Every pose is solved seeded from the pose before it, exactly as
    # `server.goto_pose` seeds from wherever the arm currently is. Solving from
    # a fixed seed instead lets IK return a different elbow configuration for
    # retract than for the points, and the arm then has to swing through a huge
    # arc between two branches of the same solution -- measured at 118 deg of
    # shoulder_lift per 1.5s move, which the servos cannot track.
    current_arm = _solve(kin, retract, None, np.zeros(5), match_orientation, "retract")

    grip = grip_open
    start = np.array([*current_arm, grip])
    segments: list[tuple[np.ndarray, float]] = []

    for index, wp in enumerate(waypoints):
        label = wp.get("name") or f"point {index + 1}"
        xyz = jittered(wp["xyz"])
        quat = wp.get("quat")

        # Visit the point twice: once to open the jaw, once to close it. The
        # travel and retract legs carry whatever the jaw is already doing --
        # after closing on an object, retracting must not drop it.
        for target_grip in (grip_open, grip_close):
            point_q = _solve(kin, xyz, quat, current_arm, match_orientation, label)
            segments.append((np.array([*point_q, grip]), T_TRAVEL))        # move to the point
            grip = target_grip
            segments.append((np.array([*point_q, grip]), T_GRIPPER))       # actuate the jaw
            segments.append((np.array([*point_q, grip]), GRIP_SETTLE))     # let it settle

            current_arm = _solve(kin, retract, None, point_q, match_orientation, "retract")
            segments.append((np.array([*current_arm, grip]), T_TRAVEL))    # lift away

    return _segments_to_plan(start, segments, fps)


# --------------------------------------------------------------------------
# Recording
# --------------------------------------------------------------------------

def build_robot(port: str, robot_id: str, width: int, height: int, fps: int,
                scripted: bool) -> SO101Follower:
    cameras = {
        "top": OpenCVCameraConfig(
            index_or_path=TOP_CAMERA_INDEX, fps=fps, width=width, height=height,
            fourcc=CAMERA_FOURCC, backend=Cv2Backends.DSHOW,
        ),
        "wrist": OpenCVCameraConfig(
            index_or_path=WRIST_CAMERA_INDEX, fps=fps, width=width, height=height,
            fourcc=CAMERA_FOURCC, backend=Cv2Backends.DSHOW,
        ),
    }
    # No max_relative_target, deliberately, in either mode.
    #
    # `record_loop` stores the action the teleoperator *requested*, not the one
    # `send_action` actually sent after clamping (lerobot_record.py:331, and
    # their own TODO beside it). So a clamped frame records a command the arm
    # never received -- poison for anything trained on it. Clamping also made
    # tracking much worse rather than safer: capping the goal to present +/- 12
    # deg stops the servo ever being handed a target far enough ahead to catch
    # up, and measured mean error on shoulder_lift went 12.6 deg -> 25.8 deg
    # with the cap on.
    #
    # `check_plan_speed` guards the real risk instead, before the arm moves: it
    # rejects a trajectory with a jump in it, which is what the cap was for.
    return SO101Follower(
        SO101FollowerConfig(port=port, id=robot_id, cameras=cameras,
                            max_relative_target=None)
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default="hand",
                        help="'hand', 'waypoints', or 'recording:<name>'")
    parser.add_argument("--repo-id", default="local/so101_demo",
                        help="dataset id, by convention '<user>/<name>'")
    parser.add_argument("--task", default="Manipulate the object",
                        help="one-line description of the task being demonstrated")
    parser.add_argument("--root", default=None,
                        help="where to write the dataset (default: datasets/<name>)")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--episode-time", type=float, default=30.0,
                        help="seconds per episode (hand source only; a scripted "
                             "episode runs exactly as long as its trajectory)")
    parser.add_argument("--reset-time", type=float, default=10.0,
                        help="seconds between episodes to reset the scene")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="replay speed multiplier for a recording source")
    parser.add_argument("--jitter", type=float, default=0.0,
                        help="metres of random offset per waypoint per episode")
    parser.add_argument("--max-speed", type=float, default=MAX_JOINT_SPEED,
                        help=f"peak commanded joint speed, deg/s (default "
                             f"{MAX_JOINT_SPEED:g}); a plan over this is stretched "
                             "in time. Raise it for shorter episodes at the cost "
                             "of the arm trailing its setpoint")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed for --jitter")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--id", default=DEFAULT_ID)
    parser.add_argument("--push-to-hub", action="store_true",
                        help="upload when done (needs `hf auth login`)")
    parser.add_argument("--resume", action="store_true",
                        help="append episodes to an existing dataset")
    parser.add_argument("--dry-run", action="store_true",
                        help="plan the trajectory and print it, moving nothing")
    args = parser.parse_args()

    init_logging()
    scripted = args.source != "hand"
    rng = np.random.default_rng(args.seed)
    kin = SO101Kinematics() if args.source == "waypoints" else None

    def build_plan() -> np.ndarray:
        """Re-planned per episode, so --jitter varies the trajectory."""
        if args.source == "waypoints":
            plan = plan_from_waypoints(kin, args.fps, args.jitter, rng)
        elif args.source.startswith("recording:"):
            plan = plan_from_recording(args.source.split(":", 1)[1], args.fps, args.speed)
        else:
            raise SystemExit(f"unknown --source '{args.source}' "
                             "(expected 'hand', 'waypoints', or 'recording:<name>')")
        plan = slow_to_limit(plan, args.fps, args.max_speed)
        check_plan_speed(plan, args.fps, args.max_speed)   # backstop after stretching
        return plan

    if args.dry_run and not scripted:
        raise SystemExit("--dry-run only applies to a scripted source; there is no "
                         "trajectory to plan for --source hand")

    if args.dry_run:
        plan = build_plan()
        span = plan.max(axis=0) - plan.min(axis=0)
        print(f"plan: {len(plan)} frames = {len(plan) / args.fps:.1f}s at {args.fps} fps")
        print(f"  start {np.round(plan[0], 1)}")
        print(f"  end   {np.round(plan[-1], 1)}")
        speeds = plan_speeds(plan, args.fps)
        for name, lo, hi, s, v in zip(RECORD_JOINTS, plan.min(axis=0), plan.max(axis=0),
                                      span, speeds):
            print(f"  {name:>14}: {lo:8.1f} .. {hi:8.1f}   "
                  f"(sweeps {s:6.1f} deg, peak {v:5.0f} deg/s)")
        print(f"\n{args.episodes} episode(s) would take "
              f"~{args.episodes * (len(plan) / args.fps + args.reset_time) / 60:.1f} min")
        return 0

    root = Path(args.root) if args.root else Path("datasets") / args.repo_id.split("/")[-1]
    robot = build_robot(args.port, args.id, args.width, args.height, args.fps, scripted)

    plan = build_plan() if scripted else None
    if scripted:
        teleop = ScriptedTeleop(ScriptedTeleopConfig(id=f"{args.id}_scripted"), robot,
                                plan, int(round(REPLAY_LOOKAHEAD * args.fps)))
        episode_time = len(plan) / args.fps
    else:
        teleop = HandGuidedTeleop(HandGuidedTeleopConfig(id=f"{args.id}_hand"), robot)
        episode_time = args.episode_time

    teleop_action_processor, robot_action_processor, robot_observation_processor = (
        make_default_processors()
    )

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )

    listener = None
    dataset = None
    try:
        robot.connect(calibrate=False)
        arm.tune_servos(robot, acceleration=SCRIPTED_ACCELERATION if scripted
                        else arm.SERVO_ACCELERATION)
        if scripted:
            # connect() enables torque, which could otherwise snap the arm toward
            # a stale goal left in the servos from a previous session.
            robot.send_action({f"{k}.pos": v for k, v in arm.read_joints(robot).items()})
            time.sleep(0.1)
            print("torque ON -- the arm will move on its own; keep clear")
        else:
            robot.bus.disable_torque()
            print("torque OFF -- the arm is limp and will sag if you let go")

        if args.resume:
            dataset = LeRobotDataset.resume(
                args.repo_id, root=root,
                image_writer_processes=0,
                image_writer_threads=4 * len(robot.cameras),
            )
        else:
            dataset = LeRobotDataset.create(
                args.repo_id, args.fps, root=root,
                robot_type=robot.name, features=dataset_features, use_videos=True,
                image_writer_processes=0,
                image_writer_threads=4 * len(robot.cameras),
            )

        listener, events = init_keyboard_listener()
        print(f"\nrecording {args.episodes} episode(s) of {episode_time:.1f}s into {root}")
        print("  right arrow = finish episode early   left arrow = re-record   ESC = stop\n")

        with VideoEncodingManager(dataset):
            recorded = 0
            while recorded < args.episodes and not events["stop_recording"]:
                if scripted:
                    # Ease into the trajectory's first pose before the clock
                    # starts, so the episode does not open with a lunge from
                    # wherever the previous one ended.
                    print(f"  moving to the start pose ({teleop.start_pose['shoulder_pan']:.0f}, "
                          f"{teleop.start_pose['shoulder_lift']:.0f}, ...)")
                    arm.ramp_to(robot, teleop.start_pose, duration=2.5, verbose=False)
                    teleop.rewind()

                log_say(f"Recording episode {dataset.num_episodes}", play_sounds=False)
                record_loop(
                    robot=robot, events=events, fps=args.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop, dataset=dataset,
                    control_time_s=episode_time, single_task=args.task,
                    display_data=False,
                )

                # A reset window between episodes, so the scene can be put back
                # without it landing in the data. A scripted run returns to its
                # own start pose instead, which is motion, not a pause.
                if not scripted and not events["stop_recording"] and (
                    recorded < args.episodes - 1 or events["rerecord_episode"]
                ):
                    log_say("Reset the environment", play_sounds=False)
                    print("  -- reset: put the scene back (not being recorded) --")
                    record_loop(
                        robot=robot, events=events, fps=args.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop,
                        control_time_s=args.reset_time, single_task=args.task,
                        display_data=False,
                    )

                if events["rerecord_episode"]:
                    print("  -- re-recording that episode --")
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                dataset.save_episode()
                recorded += 1
                print(f"  saved episode {recorded}/{args.episodes}")

                if scripted and recorded < args.episodes:
                    plan = build_plan()          # re-jitter for the next episode
                    teleop.plan = plan
                    teleop.rewind()
    finally:
        if dataset is not None:
            dataset.finalize()
        if robot.is_connected:
            # Torque off drops the arm from wherever it stopped. For a scripted
            # run that is the trajectory's end pose, which is somewhere it chose,
            # not necessarily somewhere safe to fall from.
            if scripted:
                print("torque OFF -- the arm will sag from its final pose")
            robot.disconnect()
        if listener is not None:
            listener.stop()

    if dataset is not None and dataset.num_episodes > 0:
        print(f"\n{dataset.num_episodes} episode(s), {dataset.num_frames} frames in {root}")
        if args.push_to_hub:
            dataset.push_to_hub()
    else:
        logging.warning("no episodes were saved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
