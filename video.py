r"""Record both cameras to H.264 files while something else drives the arm.

One thread per camera pulls frames and encodes them; the caller just calls
`start()` and `stop()` around whatever it was going to do anyway.

Timestamps are real elapsed milliseconds rather than an assumed frame rate. A
capture loop sharing a machine with a control loop does not tick evenly, and
writing frames as though it did makes the video drift out of step with what the
arm was doing -- which is the whole point of recording it.

Only one process can hold a DirectShow camera at a time, so while a recording is
running nothing else may open these cameras. `snapshot()` exists for that: it
writes a still from the frame the recorder already has.
"""

from __future__ import annotations

import threading
import time
from fractions import Fraction
from pathlib import Path

import av
import cv2
import numpy as np

from lerobot.cameras.configs import Cv2Backends
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

CAMERAS = {"top": 0, "wrist": 1}
WIDTH, HEIGHT, FPS = 640, 480, 30
CRF = "26"          # keeps a ~90s clip to a few MB, which is committable


class _Channel:
    def __init__(self, name: str, index: int, outdir: Path):
        self.name = name
        self.path = outdir / f"{name}.mp4"
        self.camera = OpenCVCamera(OpenCVCameraConfig(
            index_or_path=index, fps=FPS, width=WIDTH, height=HEIGHT,
            fourcc="MJPG", backend=Cv2Backends.DSHOW, warmup_s=1))
        self.latest: np.ndarray | None = None
        self.frames = 0
        self.t0 = 0.0
        self.elapsed = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def open(self) -> None:
        """Connect the camera and set the encoder up, but do not record yet."""
        self.camera.connect()
        self.container = av.open(str(self.path), mode="w")
        self.stream = self.container.add_stream("libx264", rate=FPS)
        self.stream.width, self.stream.height = WIDTH, HEIGHT
        self.stream.pix_fmt = "yuv420p"
        self.stream.options = {"crf": CRF, "preset": "medium"}

    def begin(self) -> None:
        """Start the capture thread. Opening is separate on purpose: connecting
        a camera takes seconds, so doing it inline would start one recording
        well before the other and leave the two clips offset from each other."""
        self.t0 = time.perf_counter()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        """Emit exactly one frame per 1/FPS of wall clock.

        Sampling on an absolute schedule rather than sleeping a fixed amount
        between grabs is what keeps the file's timeline equal to real time, and
        keeps the two cameras on the *same* timeline. Left to drift, the two
        loops ran at 51 and 32 fps on the same 5 seconds, so the clips played
        at different speeds and neither matched what the arm did.
        """
        period = 1.0 / FPS
        t0 = self.t0
        n = 0
        while not self._stop.is_set():
            target = t0 + n * period
            now = time.perf_counter()
            if now < target:
                time.sleep(target - now)
            elif now - target > period:
                # Fell behind: advance the frame counter to real time rather
                # than emitting a burst of catch-up frames, which would show as
                # the video briefly running fast.
                n = int((now - t0) / period)
            try:
                rgb = self.camera.async_read(timeout_ms=2000)
            except Exception:
                rgb = self.latest
            if rgb is None:
                n += 1
                continue
            with self._lock:
                self.latest = rgb
            frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
            frame.pts = n
            for packet in self.stream.encode(frame):
                self.container.mux(packet)
            self.frames += 1
            n += 1

    def snapshot(self, path: Path) -> None:
        with self._lock:
            rgb = None if self.latest is None else self.latest.copy()
        if rgb is not None:
            cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    def stop(self) -> float:
        self.elapsed = time.perf_counter() - self.t0
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
        for packet in self.stream.encode():     # flush the encoder
            self.container.mux(packet)
        self.container.close()
        self.camera.disconnect()
        return self.path.stat().st_size / 1e6


class Recorder:
    """Both cameras at once. Use as a context manager."""

    def __init__(self, outdir: str | Path, prefix: str = ""):
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.channels = {name: _Channel(f"{prefix}{name}", index, self.outdir)
                         for name, index in CAMERAS.items()}
        self.started = 0.0

    def __enter__(self) -> "Recorder":
        for channel in self.channels.values():
            channel.open()
        for channel in self.channels.values():
            channel.begin()
        self.started = time.perf_counter()
        print(f"recording to {self.outdir}")
        return self

    def snapshot(self, tag: str) -> None:
        for name, channel in self.channels.items():
            channel.snapshot(self.outdir / f"{tag}_{name}.png")

    def __exit__(self, *exc) -> None:
        for name, channel in self.channels.items():
            size = channel.stop()
            print(f"  {channel.path.name}: {channel.frames} frames, "
                  f"{channel.elapsed:.1f}s, "
                  f"{channel.frames / max(channel.elapsed, 1e-6):.1f} fps, "
                  f"{size:.1f} MB")
