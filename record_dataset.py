r"""Record a LeRobot dataset by hand-guiding the arm, with both cameras.

`lerobot-record` refuses to start without a teleoperator::

    A teleoperator is required for recording. Use --teleop.type=... to specify one.

Every arm teleoperator lerobot ships is a *separate leader device* on its own
serial port, and we have one arm. So the stock command cannot run here.

What it can do is take any object implementing the `Teleoperator` interface. So
this script supplies one -- `HandGuidedTeleop` -- that reads the follower's own
present position and returns it as the action. Torque is switched off, so the
arm is limp and you pose it by hand; what you demonstrate becomes the action,
and lerobot's own `record_loop` writes the dataset, encodes the videos and
handles the keyboard controls exactly as it would with a leader arm.

That the recorded action equals the observed state is a property of
kinesthetic teaching, not a bug: with no motor holding a target, the position
the arm reached *is* the command. Policies trained on this learn to reproduce
the trajectory you showed.

Usage::

    .venv\python.exe record_dataset.py --repo-id local/wafer --task "Move the wafer" \
        --episodes 5 --episode-time 30

During each episode:  right arrow = finish early,  left arrow = re-record,
ESC = stop the session.  The arm stays limp throughout, including between
episodes, so it will sag if you let go of it mid-air.
"""

from __future__ import annotations

import argparse
import logging
from dataclasses import dataclass
from pathlib import Path

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

from arm import DEFAULT_ID, DEFAULT_PORT

# Camera indices as they enumerate on this machine, confirmed by eye from
# `lerobot-find-cameras opencv` (see README "Cameras"): 0 looks down over the
# bench, 1 is bolted to the wrist and sees both jaws.
TOP_CAMERA_INDEX = 0
WRIST_CAMERA_INDEX = 1

# MJPG matters. The default YUY2 is uncompressed, and two uncompressed 1080p
# streams do not fit through one USB controller; with MJPG both cameras hold 30
# fps even at 1920x1080 (measured).
CAMERA_FOURCC = "MJPG"


@TeleoperatorConfig.register_subclass("so101_hand_guided")
@dataclass
class HandGuidedTeleopConfig(TeleoperatorConfig):
    pass


class HandGuidedTeleop(Teleoperator):
    """A "teleoperator" that is the follower arm itself, back-driven by hand.

    Holds a reference to the already-built robot rather than opening its own
    port: the Feetech bus is a single exclusive serial handle, so a second
    connection to COM3 would collide with the robot's.
    """

    config_class = HandGuidedTeleopConfig
    name = "so101_hand_guided"

    def __init__(self, config: HandGuidedTeleopConfig, robot: SO101Follower):
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

    def get_action(self) -> dict:
        # Present position, not goal position. With torque off the servos hold
        # no target, so where the arm physically is, is the demonstration.
        return {
            key: value
            for key, value in self.robot.get_observation().items()
            if key.endswith(".pos")
        }

    def send_feedback(self, feedback: dict) -> None:
        """No-op: there is no leader to give force feedback to."""

    def disconnect(self) -> None:
        """No-op: the caller disconnects the robot, which owns the bus."""


def build_robot(port: str, robot_id: str, width: int, height: int, fps: int) -> SO101Follower:
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
    # max_relative_target is deliberately left unset. It exists to stop a large
    # commanded jump, but here every action equals the position just read, so
    # there is no jump to clamp -- and enabling it would cost an extra bus read
    # per step for nothing.
    return SO101Follower(SO101FollowerConfig(port=port, id=robot_id, cameras=cameras))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo-id", default="local/so101_demo",
                        help="dataset id, by convention '<user>/<name>'")
    parser.add_argument("--task", default="Manipulate the object",
                        help="one-line description of the task being demonstrated")
    parser.add_argument("--root", default=None,
                        help="where to write the dataset (default: datasets/<name>)")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--episode-time", type=float, default=30.0,
                        help="seconds per episode")
    parser.add_argument("--reset-time", type=float, default=10.0,
                        help="seconds between episodes to reset the scene")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--id", default=DEFAULT_ID)
    parser.add_argument("--push-to-hub", action="store_true",
                        help="upload when done (needs `hf auth login`)")
    parser.add_argument("--resume", action="store_true",
                        help="append episodes to an existing dataset")
    args = parser.parse_args()

    init_logging()
    root = Path(args.root) if args.root else Path("datasets") / args.repo_id.split("/")[-1]

    robot = build_robot(args.port, args.id, args.width, args.height, args.fps)
    teleop = HandGuidedTeleop(HandGuidedTeleopConfig(id=f"{args.id}_hand"), robot)

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
        # connect() enables torque. Drop it immediately: the whole point is that
        # the arm is limp enough to pose by hand. send_action() still writes
        # Goal_Position each step, but a servo with torque off ignores it.
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
        print(f"\nrecording {args.episodes} episode(s) of {args.episode_time:.0f}s into {root}")
        print("  right arrow = finish episode early   left arrow = re-record   ESC = stop\n")

        with VideoEncodingManager(dataset):
            recorded = 0
            while recorded < args.episodes and not events["stop_recording"]:
                log_say(f"Recording episode {dataset.num_episodes}", play_sounds=False)
                record_loop(
                    robot=robot, events=events, fps=args.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop, dataset=dataset,
                    control_time_s=args.episode_time, single_task=args.task,
                    display_data=False,
                )

                # A reset pass between episodes, so you can put the scene back
                # without it landing in the data. Skipped after the last one.
                if not events["stop_recording"] and (
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
    finally:
        if dataset is not None:
            dataset.finalize()
        if robot.is_connected:
            robot.disconnect()
        if listener is not None:
            listener.stop()

    if dataset is not None and dataset.num_episodes > 0:
        print(f"\n{dataset.num_episodes} episode(s), {dataset.num_frames} frames in {root}")
        if args.push_to_hub:
            dataset.push_to_hub()
    else:
        logging.warning("no episodes were saved")


if __name__ == "__main__":
    main()
