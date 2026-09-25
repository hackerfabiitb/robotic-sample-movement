r"""Fit a pixel -> table-coordinate map for the overhead camera.

The arm knows where its own tool is; the camera does not. To turn "the box is at
pixel (790, 337)" into a reachable XYZ, we need the mapping between them.

The bench is flat, so for points lying on it the mapping is a plane-to-plane
homography -- eight numbers, recoverable from four or more correspondences. This
script collects them by driving the tool to known positions just above the table
and finding where it landed in the image.

Locating the gripper in frame is the awkward part, and eyeballing it is both
tedious and imprecise. Instead, at each point the jaw is opened and then closed
and the two frames subtracted: the arm, the bench and the lighting are identical
in both, so the only thing that survives the subtraction is the jaw itself. The
centroid of what is left is the tool position, to within a pixel or two, with no
thresholding of colour or shape and no model of what a gripper looks like.

The result is written to camera_calib.json, and is only valid while the camera
does not move. Re-run it if the camera is bumped.

Usage::

    .venv\python.exe calibrate_camera.py            # run it, writes the json
    .venv\python.exe calibrate_camera.py --check    # residuals of the saved fit
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

import arm
from kinematics import ARM_JOINTS, SO101Kinematics
from lerobot.cameras.configs import Cv2Backends
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

CALIB_FILE = Path(__file__).parent / "camera_calib.json"
RAW_FILE = Path(__file__).parent / "camera_calib_raw.json"

TOP_CAMERA_INDEX = 0
CAM_WIDTH, CAM_HEIGHT = 1280, 720

# Points spanning the working area, in metres in base_link. All verified
# reachable; the spread matters more than the count, since a homography fitted
# from points clustered in one corner extrapolates badly across the rest.
CALIB_POINTS = [
    (0.28, -0.02), (0.28, 0.10), (0.24, 0.12),
    (0.22, -0.06), (0.22, 0.06), (0.22, 0.16),
    (0.16, -0.02), (0.16, 0.10), (0.16, 0.18),
    (0.10, 0.16),
]

# Two sweep heights, metres of tool z. One plane is not enough: a homography
# fitted at a single height describes *that* height only, and an object on the
# bench is ~50mm below where the jaw sweeps, so its pixel would back-project
# with a parallax error of over a centimetre. Points at two heights are not
# coplanar, which is what lets a full projective camera be fitted instead.
SWEEP_Z = (0.005, 0.140)
SAFE_Z = 0.18     # travel height, so the tool crosses the bench over any objects
GRIP_OPEN = 45.0
GRIP_CLOSED = 16.0


def open_camera() -> OpenCVCamera:
    cam = OpenCVCamera(OpenCVCameraConfig(
        index_or_path=TOP_CAMERA_INDEX, fps=30, width=CAM_WIDTH, height=CAM_HEIGHT,
        fourcc="MJPG", backend=Cv2Backends.DSHOW, warmup_s=1))
    cam.connect()
    return cam


def grab(cam: OpenCVCamera, n: int = 6) -> np.ndarray:
    """Average a few frames, to keep sensor noise out of the difference image."""
    frames = [cam.async_read(timeout_ms=2000).astype(np.float32) for _ in range(n)]
    return np.mean(frames, axis=0)


def jaw_world_point(kin: SO101Kinematics, q_deg: np.ndarray) -> np.ndarray:
    """Where the moving jaw sweeps between open and closed, in base_link.

    This is what the difference image actually measures, and it is not the tool
    frame: the jaw sits off to one side of it, and that offset rotates with
    shoulder_pan, so treating the tool point as the observed feature puts a
    pan-dependent error into the fit. Computed from *measured* joint angles, so
    droop and lag drop out too -- the arm does not have to have arrived exactly
    where it was sent.
    """
    a = kin.gripper_geometry(q_deg, GRIP_OPEN)["moving"]
    b = kin.gripper_geometry(q_deg, GRIP_CLOSED)["moving"]
    # Midpoint of the jaw bar in each state, then midway between the two: the
    # centroid of the area the jaw vacates plus the area it moves into.
    mid_open = 0.5 * (a[0] + a[2])
    mid_closed = 0.5 * (b[0] + b[2])
    return 0.5 * (mid_open + mid_closed)


def jaw_centroid(open_frame: np.ndarray, closed_frame: np.ndarray,
                 debug_path: Path | None = None) -> tuple[float, float] | None:
    """Centroid of whatever moved between the two frames.

    Only the jaw moves, so the largest connected difference is it. Returning the
    centroid of the *masked difference* rather than of the contour weights the
    answer toward the strongest edges, which are the jaw tips.
    """
    diff = np.abs(open_frame - closed_frame).mean(axis=2)
    diff = cv2.GaussianBlur(diff, (5, 5), 0)
    if diff.max() < 8.0:                       # nothing moved: jaw stuck or hidden
        return None

    mask = (diff > max(10.0, 0.35 * diff.max())).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if count < 2:
        return None
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = stats[biggest, cv2.CC_STAT_AREA]
    # A jaw swing is a few hundred pixels. Much smaller is noise; much larger
    # means something else moved -- a hand in shot, or the whole forearm shifting
    # as the servo takes up backlash.
    if not (40 <= area <= 4000):
        return None

    blob = (labels == biggest)
    weights = diff * blob
    ys, xs = np.nonzero(blob)
    total = weights[ys, xs].sum()
    u = float((xs * weights[ys, xs]).sum() / total)
    v = float((ys * weights[ys, xs]).sum() / total)

    if debug_path is not None:
        vis = np.dstack([mask * 255] * 3).astype(np.uint8)
        cv2.drawMarker(vis, (int(round(u)), int(round(v))), (0, 0, 255),
                       cv2.MARKER_CROSS, 30, 2)
        cv2.imwrite(str(debug_path), vis)
    return u, v


def goto(robot, kin: SO101Kinematics, xyz, duration: float) -> bool:
    q, _, err = kin.ik(np.asarray(xyz, dtype=float),
                       q_init=arm.arm_vector(arm.read_joints(robot)))
    if err > 0.005:
        print(f"  unreachable {xyz} (off by {err * 1000:.0f} mm)")
        return False
    arm.ramp_to(robot, dict(zip(ARM_JOINTS, q)), duration=duration, verbose=False)
    return True


CYCLES = 3          # open/close repeats per point
AGREE_PX = 25.0     # cycles must land this close to each other to be believed


def measure_point(robot, kin, cam, tag: str, debug_dir: Path | None):
    """One calibration point: several jaw swings, and the joints as measured.

    Repeating matters. A single difference image cannot tell the jaw moving from
    anything else that happened to move in that half second -- on the first run
    one point locked onto activity at the far side of the bench and came out
    400mm wrong. Something transient will not repeat in the same place; the jaw
    will.
    """
    seen = []
    for c in range(CYCLES):
        arm.ramp_to(robot, {"gripper": GRIP_OPEN}, duration=0.6, verbose=False)
        time.sleep(0.4)
        opened = grab(cam)
        arm.ramp_to(robot, {"gripper": GRIP_CLOSED}, duration=0.6, verbose=False)
        time.sleep(0.4)
        closed = grab(cam)
        dbg = (debug_dir / f"jaw_{tag}_{c}.png") if debug_dir else None
        found = jaw_centroid(opened, closed, dbg)
        if found is not None:
            seen.append(found)

    if not seen:
        return None, None
    pts = np.asarray(seen)
    median = np.median(pts, axis=0)
    agree = pts[np.linalg.norm(pts - median, axis=1) <= AGREE_PX]
    if len(agree) < 2:
        return None, None

    # Read the joints while the arm is still standing here, so the world point
    # is where the jaw actually is rather than where it was asked to go.
    q = arm.arm_vector(arm.read_joints(robot))
    return agree.mean(axis=0), q


def collect(debug_dir: Path | None):
    kin = SO101Kinematics()
    cam = open_camera()
    robot = arm.connect(max_step=None)
    pixels, world, joints = [], [], []
    try:
        for z in SWEEP_Z:
            print(f"--- sweeping at tool z = {z:.3f} m ---")
            for i, (x, y) in enumerate(CALIB_POINTS):
                tag = f"z{int(z * 1000):03d}_{i:02d}"
                print(f"[{tag}] ({x:.2f}, {y:.2f})")
                if not goto(robot, kin, (x, y, SAFE_Z), 1.5):
                    continue
                if not goto(robot, kin, (x, y, z), 1.0):
                    continue

                pixel, q = measure_point(robot, kin, cam, tag, debug_dir)
                if pixel is None:
                    print("    jaw not found consistently -- skipped")
                else:
                    xyz = jaw_world_point(kin, q)
                    print(f"    pixel ({pixel[0]:7.1f}, {pixel[1]:7.1f})  "
                          f"jaw at ({xyz[0]:6.3f}, {xyz[1]:6.3f}, {xyz[2]:6.3f})")
                    pixels.append(pixel)
                    world.append(xyz)
                    joints.append(q)
                goto(robot, kin, (x, y, SAFE_Z), 1.0)
    finally:
        robot.disconnect()
        cam.disconnect()
        print("torque off")
    return (np.asarray(pixels, dtype=np.float64),
            np.asarray(world, dtype=np.float64),
            np.asarray(joints, dtype=np.float64))


def _normalise(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Isotropic scaling to mean distance sqrt(dim) -- standard DLT conditioning."""
    centre = pts.mean(axis=0)
    shifted = pts - centre
    scale = np.sqrt(pts.shape[1]) / max(np.linalg.norm(shifted, axis=1).mean(), 1e-12)
    T = np.eye(pts.shape[1] + 1)
    T[:-1, :-1] *= scale
    T[:-1, -1] = -scale * centre
    return np.hstack([pts, np.ones((len(pts), 1))]) @ T.T, T


def fit_projection(world: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    """3x4 camera matrix by DLT, mapping base_link metres to image pixels.

    A projective camera, not a homography: the calibration points sit at two
    different heights, so they are not coplanar and the full 11-DOF projection
    is determined. That is what makes it possible to back-project a pixel onto
    the *table* even though the feature was measured 50mm above it.
    """
    if len(world) < 6:
        raise SystemExit(f"only {len(world)} correspondences; DLT needs at least 6")
    Xn, T3 = _normalise(world)
    xn, T2 = _normalise(pixels)

    rows = []
    for X, x in zip(Xn, xn):
        u, v, w = x
        rows.append(np.concatenate([np.zeros(4), -w * X, v * X]))
        rows.append(np.concatenate([w * X, np.zeros(4), -u * X]))
    _, _, Vt = np.linalg.svd(np.asarray(rows))
    P = Vt[-1].reshape(3, 4)
    return np.linalg.inv(T2) @ P @ T3        # undo the conditioning


def project(P: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    p = P @ np.append(np.asarray(xyz, dtype=float), 1.0)
    return p[:2] / p[2]


def pixel_to_plane(P: np.ndarray, u: float, v: float, z: float = 0.0) -> np.ndarray:
    """Where the ray through pixel (u, v) meets the horizontal plane at height z.

    Each image coordinate gives one linear constraint on the world point, so
    fixing z leaves two equations in x and y.
    """
    A = np.array([
        [P[0, 0] - u * P[2, 0], P[0, 1] - u * P[2, 1]],
        [P[1, 0] - v * P[2, 0], P[1, 1] - v * P[2, 1]],
    ])
    b = np.array([
        u * (P[2, 2] * z + P[2, 3]) - (P[0, 2] * z + P[0, 3]),
        v * (P[2, 2] * z + P[2, 3]) - (P[1, 2] * z + P[1, 3]),
    ])
    xy = np.linalg.solve(A, b)
    return np.array([xy[0], xy[1], z])


def reprojection_errors(P: np.ndarray, world: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    """Pixel error per point -- the natural residual for a projective fit."""
    return np.array([np.linalg.norm(project(P, X) - x) for X, x in zip(world, pixels)])


def fit_robust(world: np.ndarray, pixels: np.ndarray, max_px: float = 5.0,
               iterations: int = 400, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """RANSAC over minimal 6-point samples, then refit on the consensus.

    A plain least-squares fit is not enough. One bad correspondence -- the jaw
    detector locking onto something else that moved -- pulls the whole camera
    model far enough that *every* point then looks like an outlier, so
    fit-then-drop has nothing left to keep. Sampling small subsets finds the
    majority that agree and ignores the rest.
    """
    n = len(world)
    if n < 6:
        raise SystemExit(f"only {n} correspondences; DLT needs at least 6")
    rng = np.random.default_rng(seed)
    best = np.zeros(n, dtype=bool)
    for _ in range(iterations):
        sample = rng.choice(n, 6, replace=False)
        try:
            P = fit_projection(world[sample], pixels[sample])
        except Exception:
            continue
        inliers = reprojection_errors(P, world, pixels) <= max_px
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 6:
        raise SystemExit(
            f"no consensus among {n} correspondences -- at most {int(best.sum())} "
            "points agree on any camera model. Re-run with --debug-dir and look "
            "at the difference masks; something other than the jaw is moving.")
    return fit_projection(world[best], pixels[best]), best


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="print residuals of the saved calibration, moving nothing")
    parser.add_argument("--refit", action="store_true",
                        help="refit from the saved raw correspondences, moving nothing")
    parser.add_argument("--debug-dir", default=None,
                        help="write the jaw-difference masks here for inspection")
    args = parser.parse_args()

    if args.check:
        if not CALIB_FILE.exists():
            raise SystemExit(f"{CALIB_FILE.name} not found -- run without --check first")
        saved = json.loads(CALIB_FILE.read_text())
        P = np.asarray(saved["projection"], dtype=float)
        pixels = np.asarray(saved["pixels"], dtype=float)
        world = np.asarray(saved["world"], dtype=float)
        keep = np.asarray(saved["inliers"], dtype=bool)
        err = reprojection_errors(P, world, pixels)
        print(f"{len(pixels)} correspondences, reprojection error in pixels:")
        for (u, v), X, e, k in zip(pixels, world, err, keep):
            print(f"  ({u:7.1f},{v:7.1f}) -> ({X[0]:6.3f},{X[1]:6.3f},{X[2]:6.3f})"
                  f"   {e:6.2f}{'' if k else '  (dropped)'}")
        print(f"\nkept {int(keep.sum())}: mean {err[keep].mean():.2f} px, "
              f"max {err[keep].max():.2f} px")
        # What the user actually cares about: how well it locates a thing on the
        # bench. Round-trip each kept point through the table-plane back-projection.
        back = np.array([pixel_to_plane(P, u, v, z=X[2])[:2]
                         for (u, v), X in zip(pixels[keep], world[keep])])
        mm = np.linalg.norm(back - world[keep][:, :2], axis=1) * 1000
        print(f"back-projection to each point's own height: mean {mm.mean():.2f} mm, "
              f"max {mm.max():.2f} mm")
        return 0

    debug_dir = Path(args.debug_dir) if args.debug_dir else None
    if debug_dir:
        debug_dir.mkdir(parents=True, exist_ok=True)

    if args.refit:
        if not RAW_FILE.exists():
            raise SystemExit(f"{RAW_FILE.name} not found -- run a collection first")
        raw = json.loads(RAW_FILE.read_text())
        pixels = np.asarray(raw["pixels"], dtype=float)
        world = np.asarray(raw["world"], dtype=float)
        joints = np.asarray(raw["joints"], dtype=float)
        print(f"refitting from {len(pixels)} saved correspondences, nothing moved")
    else:
        pixels, world, joints = collect(debug_dir)
        # Written before fitting, deliberately. Collection costs minutes of robot
        # time; a bug in the fit should never be able to throw it away. --refit
        # reruns the maths against this file with the arm untouched.
        RAW_FILE.write_text(json.dumps({
            "pixels": pixels.tolist(), "world": world.tolist(),
            "joints": joints.tolist(), "collected": time.time(),
        }, indent=2))
        print(f"raw correspondences saved to {RAW_FILE.name}")

    P, keep = fit_robust(world, pixels)
    err = reprojection_errors(P, world, pixels)
    print(f"\nfitted from {int(keep.sum())} of {len(pixels)} points")
    print(f"reprojection: mean {err[keep].mean():.2f} px, max {err[keep].max():.2f} px")
    print(f"heights spanned: {world[keep][:, 2].min():.3f} .. {world[keep][:, 2].max():.3f} m")

    CALIB_FILE.write_text(json.dumps({
        "projection": P.tolist(),
        "pixels": pixels.tolist(),
        "world": world.tolist(),
        "joints": joints.tolist(),
        "inliers": keep.tolist(),
        "sweep_z": list(SWEEP_Z),
        "camera": {"index": TOP_CAMERA_INDEX, "width": CAM_WIDTH, "height": CAM_HEIGHT},
        "fitted": time.time(),
    }, indent=2))
    print(f"wrote {CALIB_FILE.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
