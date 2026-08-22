r"""Storage and playback maths for hand-demonstrated trajectories.

A demonstration is recorded in JOINT space, not Cartesian. That is deliberate:
the thing you physically showed the arm *is* a joint trajectory, so replaying
those angles reproduces it exactly. Going via Cartesian would mean solving IK on
every frame, which can pick a different elbow configuration, wander near
singularities, or fail outright on a pose the demo passed through happily.

Each recording is one JSON file under recordings/:

    {"name": "...", "recorded": 1699999999.0, "joints": ["shoulder_pan", ...],
     "samples": [[t, j0, j1, ...], ...]}

`t` is seconds from the start of the recording.
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path

import numpy as np

RECORDINGS_DIR = Path(__file__).parent / "recordings"

# Hand-guided motion is shaky, so the raw capture is smoothed before replay.
# ~0.23s at 30 Hz: long enough to take the tremor out, short enough to keep
# deliberate movement. Raw samples are always what gets stored, so changing this
# re-reads cleanly rather than degrading a recording permanently.
SMOOTH_WINDOW = 7


def safe_name(name: str) -> str:
    """Filesystem-safe slug, so a typed name cannot escape recordings/."""
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "-", (name or "").strip()).strip("-.")
    return slug[:60] or f"demo-{int(time.time())}"


def save_recording(name: str, joint_names: list[str], samples: list[list[float]]) -> str:
    RECORDINGS_DIR.mkdir(exist_ok=True)
    slug = safe_name(name)
    payload = {
        "name": slug,
        "recorded": time.time(),
        "joints": joint_names,
        "samples": [[round(float(v), 4) for v in row] for row in samples],
    }
    (RECORDINGS_DIR / f"{slug}.json").write_text(json.dumps(payload))
    return slug


def load_recording(name: str) -> dict | None:
    path = RECORDINGS_DIR / f"{safe_name(name)}.json"
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def delete_recording(name: str) -> bool:
    path = RECORDINGS_DIR / f"{safe_name(name)}.json"
    if path.exists():
        path.unlink()
        return True
    return False


def list_recordings() -> list[dict]:
    """Summaries only -- the sample arrays are far too big to stream to the UI."""
    if not RECORDINGS_DIR.exists():
        return []
    out = []
    for path in sorted(RECORDINGS_DIR.glob("*.json")):
        try:
            data = json.loads(path.read_text())
            samples = data.get("samples", [])
            out.append({
                "name": data.get("name", path.stem),
                "duration": round(float(samples[-1][0]), 2) if samples else 0.0,
                "samples": len(samples),
                "recorded": data.get("recorded", 0),
            })
        except Exception:
            continue
    out.sort(key=lambda r: r["recorded"], reverse=True)
    return out


def smooth_samples(samples: list[list[float]], window: int = SMOOTH_WINDOW) -> np.ndarray:
    """Moving-average each joint channel, leaving the time column alone.

    Edges use progressively shorter windows ('same' convolution divided by the
    count of real contributors), so the start and end are not dragged toward
    zero -- which on a joint angle would be a real move.
    """
    arr = np.asarray(samples, dtype=float)
    if window < 2 or len(arr) < window:
        return arr
    out = arr.copy()
    kernel = np.ones(window)
    norm = np.convolve(np.ones(len(arr)), kernel, mode="same")
    for c in range(1, arr.shape[1]):
        out[:, c] = np.convolve(arr[:, c], kernel, mode="same") / norm
    return out


def resample(arr: np.ndarray, t: float) -> np.ndarray:
    """Joint vector at time `t`, linearly interpolated between samples.

    Replay steps on its own clock rather than the recorder's, so it can run at a
    different rate or speed than the demo was captured at.
    """
    times = arr[:, 0]
    return np.array([np.interp(t, times, arr[:, c]) for c in range(1, arr.shape[1])])
