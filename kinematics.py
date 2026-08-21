r"""Forward and inverse kinematics for the SO-101, driven by the official URDF.

LeRobot's own solver (lerobot.model.kinematics.RobotKinematics) needs `placo`,
which has no Windows wheels and fails to build from source here, so this is a
self-contained replacement using numpy only.

Geometry comes from urdf/so101_new_calib.urdf (TheRobotStudio/SO-ARM100). Every
revolute joint in that file has local axis (0,0,1), so each link transform is
just Translate(xyz) @ RPY(rpy) @ RotZ(q).

Frames: base_link is the origin, +Z up. The tool point is `gripper_frame_link`.
Angles are degrees throughout, matching lerobot's `use_degrees=True` convention.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

URDF_PATH = Path(__file__).parent / "urdf" / "so101_new_calib.urdf"
TIP_LINK = "gripper_frame_link"
BASE_LINK = "base_link"

# The five joints that position the tool. `gripper` is not part of the chain --
# it only opens/closes the jaw, so it never affects where the tool point is.
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


@dataclass
class Link:
    name: str
    joint_type: str
    origin: np.ndarray  # 4x4 fixed transform from parent
    axis: np.ndarray


class SO101Kinematics:
    def __init__(self, urdf_path: Path | str = URDF_PATH):
        root = ET.parse(urdf_path).getroot()

        joints = {}
        child_of = {}
        for j in root.findall("joint"):
            name = j.get("name")
            o = j.find("origin")
            xyz = np.fromstring(o.get("xyz", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
            rpy = np.fromstring(o.get("rpy", "0 0 0"), sep=" ") if o is not None else np.zeros(3)
            a = j.find("axis")
            axis = np.fromstring(a.get("xyz", "0 0 1"), sep=" ") if a is not None else np.array([0, 0, 1.0])

            T = np.eye(4)
            T[:3, :3] = rpy_to_matrix(*rpy)
            T[:3, 3] = xyz

            lim = j.find("limit")
            limits = (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None and \
                lim.get("lower") is not None else (-np.inf, np.inf)

            joints[name] = dict(type=j.get("type"), T=T, axis=axis,
                                parent=j.find("parent").get("link"),
                                child=j.find("child").get("link"), limits=limits)
            child_of[j.find("child").get("link")] = name

        # Walk back from the tool to the base to recover the chain order.
        chain, link = [], TIP_LINK
        while link != BASE_LINK:
            jname = child_of[link]
            chain.append(jname)
            link = joints[jname]["parent"]
        chain.reverse()

        self.joints = joints
        self.chain = chain
        self.limits_deg = {
            n: (np.degrees(joints[n]["limits"][0]), np.degrees(joints[n]["limits"][1]))
            for n in ARM_JOINTS
        }

    def fk(self, q_deg: dict[str, float] | np.ndarray) -> np.ndarray:
        """Tool pose as a 4x4 matrix in base_link coordinates."""
        if not isinstance(q_deg, dict):
            q_deg = dict(zip(ARM_JOINTS, np.asarray(q_deg, dtype=float)))

        T = np.eye(4)
        for jname in self.chain:
            j = self.joints[jname]
            T = T @ j["T"]
            if j["type"] == "revolute":
                q = np.radians(q_deg.get(jname, 0.0))
                Rz = np.eye(4)
                c, s = np.cos(q), np.sin(q)
                Rz[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                T = T @ Rz
        return T

    def fk_position(self, q_deg) -> np.ndarray:
        return self.fk(q_deg)[:3, 3]

    def fk_frames(self, q_deg) -> list[tuple[str, np.ndarray]]:
        """Origin of every link along the chain, for drawing a stick figure.

        Returns [(link_name, xyz), ...] starting at the base and ending at the
        tool point. Consecutive entries are the segments to draw.
        """
        if not isinstance(q_deg, dict):
            q_deg = dict(zip(ARM_JOINTS, np.asarray(q_deg, dtype=float)))

        frames = [(BASE_LINK, np.zeros(3))]
        T = np.eye(4)
        for jname in self.chain:
            j = self.joints[jname]
            T = T @ j["T"]
            if j["type"] == "revolute":
                q = np.radians(q_deg.get(jname, 0.0))
                Rz = np.eye(4)
                c, s = np.cos(q), np.sin(q)
                Rz[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                T = T @ Rz
            frames.append((j["child"], T[:3, 3].copy()))
        return frames

    def gripper_geometry(self, q_deg, gripper_norm: float) -> dict[str, list[np.ndarray]]:
        """Segments for drawing both jaws, given the 0-100 gripper value.

        The URDF has no separate link for the fixed jaw -- it is part of
        `gripper_link`'s mesh -- so the static finger is drawn as gripper_link's
        origin out to the tool point. The moving jaw pivots on the `gripper`
        joint; its length comes from that link's centre of mass sitting at
        y = -0.030 in its own frame, i.e. a bar of roughly twice that.

        lerobot's 0-100 maps linearly onto the URDF's joint limits. Checked
        numerically: at the lower limit the jaw tip sits ~25mm from the tool
        point (closed) and at the upper limit ~120mm (open), which matches
        0 = closed / 100 = open.
        """
        if not isinstance(q_deg, dict):
            q_deg = dict(zip(ARM_JOINTS, np.asarray(q_deg, dtype=float)))

        T = np.eye(4)
        T_tip_local = np.eye(4)
        for jname in self.chain:
            j = self.joints[jname]
            if j["child"] == TIP_LINK:
                T_tip_local = j["T"]
                break
            T = T @ j["T"]
            if j["type"] == "revolute":
                a = np.radians(q_deg.get(jname, 0.0))
                c, s = np.cos(a), np.sin(a)
                Rz = np.eye(4)
                Rz[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                T = T @ Rz

        origin = T[:3, 3].copy()
        tcp = (T @ T_tip_local)[:3, 3]

        gj = self.joints["gripper"]
        lo, hi = gj["limits"]
        angle = lo + (float(np.clip(gripper_norm, 0.0, 100.0)) / 100.0) * (hi - lo)
        c, s = np.cos(angle), np.sin(angle)
        Rz = np.eye(4)
        Rz[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        T_jaw = T @ gj["T"] @ Rz

        pivot = T_jaw[:3, 3].copy()
        tip = (T_jaw @ np.array([0.0, -0.058, 0.019, 1.0]))[:3]
        knuckle = (T_jaw @ np.array([0.0, 0.0, 0.019, 1.0]))[:3]

        return {"fixed": [origin, tcp], "moving": [pivot, knuckle, tip]}

    def jacobian(self, q_deg: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """Numeric position Jacobian, 3x5, in metres per RADIAN.

        Radians matter: in deg the entries are ~0.005 and any sane damping term
        would dominate J@J.T and stall the solver.
        """
        J = np.zeros((3, len(ARM_JOINTS)))
        base = self.fk_position(q_deg)
        for i in range(len(ARM_JOINTS)):
            dq = np.asarray(q_deg, dtype=float).copy()
            dq[i] += np.degrees(eps)
            J[:, i] = (self.fk_position(dq) - base) / eps
        return J

    def ik(
        self,
        target_xyz,
        q_init=None,
        restarts: int = 12,
        seed: int = 0,
    ) -> tuple[np.ndarray, bool, float]:
        """IK with random restarts. Returns (q_deg, converged, err_m).

        A single damped-least-squares descent can settle into a local minimum,
        so on failure we retry from random configurations and keep the best.
        """
        q, ok, err = self._ik_once(target_xyz, q_init)
        if ok:
            return q, ok, err

        lo = np.array([self.limits_deg[n][0] for n in ARM_JOINTS])
        hi = np.array([self.limits_deg[n][1] for n in ARM_JOINTS])
        rng = np.random.default_rng(seed)
        best = (q, ok, err)
        for _ in range(restarts):
            q_try, ok_try, err_try = self._ik_once(target_xyz, rng.uniform(lo * 0.7, hi * 0.7))
            if err_try < best[2]:
                best = (q_try, ok_try, err_try)
            if ok_try:
                break
        return best

    def _ik_once(
        self,
        target_xyz,
        q_init=None,
        max_iters: int = 200,
        tol: float = 1e-4,
        damping: float = 0.02,
        rest_weight: float = 0.01,
    ) -> tuple[np.ndarray, bool, float]:
        """Damped-least-squares IK for tool position. Returns (q_deg, converged, err_m).

        The arm has 5 joints but position is only 3 constraints, so solutions form
        a 2-parameter family. The nullspace term gently pulls toward `q_init` so
        successive calls stay near each other instead of wandering.
        """
        target = np.asarray(target_xyz, dtype=float)
        q = np.zeros(len(ARM_JOINTS)) if q_init is None else np.asarray(q_init, dtype=float).copy()
        rest_rad = np.radians(q.copy())

        lo = np.array([self.limits_deg[n][0] for n in ARM_JOINTS])
        hi = np.array([self.limits_deg[n][1] for n in ARM_JOINTS])

        err = np.inf
        for _ in range(max_iters):
            e = target - self.fk_position(q)
            err = float(np.linalg.norm(e))
            if err < tol:
                return q, True, err

            J = self.jacobian(q)                      # m/rad
            JJt = J @ J.T + (damping ** 2) * np.eye(3)
            dq = J.T @ np.linalg.solve(JJt, e)        # rad
            null = (np.eye(len(q)) - J.T @ np.linalg.solve(JJt, J)) @ (rest_rad - np.radians(q))
            dq = dq + rest_weight * null

            step = np.linalg.norm(dq)
            if step > 0.15:                           # cap ~8.6 deg/iter
                dq *= 0.15 / step
            q = np.clip(q + np.degrees(dq), lo, hi)

        return q, False, err


if __name__ == "__main__":
    import sys

    kin = SO101Kinematics()
    print("chain:", " -> ".join(kin.chain))
    print()
    for name in ARM_JOINTS:
        lo, hi = kin.limits_deg[name]
        print(f"  {name:<15} limits {lo:>7.1f} .. {hi:>7.1f} deg")

    print("\nzero pose (all joints 0):")
    T = kin.fk(np.zeros(5))
    print(f"  tool xyz = {np.round(T[:3, 3], 4)} m")
    print(f"  tool R   =\n{np.round(T[:3, :3], 3)}")

    if len(sys.argv) > 1:
        q = np.array([float(v) for v in sys.argv[1:6]])
        print(f"\nFK at {q}: {np.round(kin.fk_position(q), 4)} m")
