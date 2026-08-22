r"""Live web GUI for the SO-101: monitor it, record waypoints, and play them back.

    .venv\python.exe server.py

Then open http://localhost:8000

Two modes, and the arm is only ever stiff in one of them:

  monitor  torque OFF. Back-drive the arm by hand, watch the readout follow,
           and hit "capture" to record wherever it currently is.
  running  torque ON. The arm plays the recorded waypoints back.

Only one thread ever touches the robot. A serial port is exclusive, so the
poller owns the connection and the HTTP handlers hand it commands through a
queue rather than opening the port themselves.

Forward kinematics stays in Python: the browser is sent the already-computed 3D
position of every joint origin plus both gripper jaws, and simply draws them.
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

import arm
import demos
from kinematics import ARM_JOINTS, SO101Kinematics, pose_matrix, quat_from_matrix

WEB_DIR = Path(__file__).parent / "web"
WAYPOINT_FILE = Path(__file__).parent / "waypoints.json"

# Motion tuning for playback. Halved from the original 3.0/1.0 for 2x speed;
# measured roughness at 1.5s is the same as at 3.0s, so this costs no smoothness.
T_TRAVEL = 1.5
T_GRIPPER = 0.5
GRIP_SETTLE = 0.3
RAMP_RATE = 30.0
MAX_STEP = 12.0

# Seconds to read the demo trajectory ahead of the clock during replay, to
# offset servo lag. Tuned by measurement -- see README "Teach by demonstration".
REPLAY_LOOKAHEAD = 0.10

_state: dict = {
    "connected": False,
    "error": None,
    "mode": "starting",
    "seq": 0,
    "waypoints": [],
    "retract": None,
    # One open value and one close value for the whole sequence, not per point.
    "gripper_open": 90.0,
    "gripper_close": 5.0,
    # Waypoints record orientation as well as position; this decides whether
    # playback tries to reproduce it.
    "match_orientation": True,
    # Hand-demonstrated trajectories: record by back-driving, then replay.
    "recording": False,
    "record_name": "",
    "record_samples": 0,
    "record_duration": 0.0,
    "recordings": [],
    "replaying": None,
    "replay_progress": 0.0,
    "log": [],
    "running_index": None,
}
_state_lock = threading.Lock()
_commands: queue.Queue = queue.Queue()
_stop = threading.Event()
_abort = threading.Event()

# Written only by the poller thread while a demonstration is being captured.
_rec: dict = {"active": False, "t0": 0.0, "samples": [], "name": ""}


# Everything the arm has, gripper included: the demo is replayed verbatim.
RECORD_JOINTS = ARM_JOINTS + ["gripper"]


def log(msg: str) -> None:
    with _state_lock:
        _state["log"] = (_state["log"] + [msg])[-40:]
        _state["seq"] += 1
    print(msg)


def load_saved() -> dict:
    """Read the saved setup, tolerating both older file layouts.

    Points used to carry their own gripper value; that is now a pair of values
    for the whole sequence, so any per-point `gripper` key is dropped on load.
    """
    out = {"waypoints": [], "retract": None, "gripper_open": 90.0, "gripper_close": 5.0,
           "match_orientation": True}
    if not WAYPOINT_FILE.exists():
        return out
    try:
        data = json.loads(WAYPOINT_FILE.read_text())
        if isinstance(data, list):  # oldest layout: a bare list of points
            data = {"waypoints": data}
        out["waypoints"] = [
            {"name": w.get("name") or f"p{i + 1}", "xyz": w["xyz"], "quat": w.get("quat")}
            for i, w in enumerate(data.get("waypoints", []))
        ]
        out["retract"] = data.get("retract")
        out["gripper_open"] = float(data.get("gripper_open", 90.0))
        out["gripper_close"] = float(data.get("gripper_close", 5.0))
        out["match_orientation"] = bool(data.get("match_orientation", True))
    except Exception:
        pass
    return out


def persist() -> None:
    with _state_lock:
        data = {
            "retract": _state.get("retract"),
            "gripper_open": _state.get("gripper_open"),
            "gripper_close": _state.get("gripper_close"),
            "match_orientation": _state.get("match_orientation"),
            "waypoints": list(_state["waypoints"]),
        }
    WAYPOINT_FILE.write_text(json.dumps(data, indent=2))


def connect_failure_hint(port: str, e: Exception) -> str:
    """Explain a failed connect, distinguishing the two causes that look alike.

    Port contention and a flaky bus both surface as an exception out of connect,
    but the fixes are opposite: kill a process, versus just try again. Blaming
    contention for a comms glitch sends you hunting a process that isn't there.
    """
    msg = str(e)
    contention = "could not connect on port" in msg.lower() or "access is denied" in msg.lower()

    if contention:
        return "\n".join([
            f"could not OPEN {port}: {e}",
            "",
            f"{port} exists but is already held by another process -- most often an",
            "earlier server.py still running in another terminal. A serial port is",
            "exclusive on Windows, so the second opener just gets refused.",
            "",
            "Find and stop it with:",
            '  Get-CimInstance Win32_Process -Filter "Name LIKE \'%python%\'" |',
            "      Select-Object ProcessId, CommandLine",
            "  Stop-Process -Id <pid> -Force",
            "",
            "Note that `pkill` from Git Bash does NOT reliably kill these.",
        ])

    return "\n".join([
        f"{port} opened, but talking to the servos failed: {e}",
        "",
        "This is a bus comms glitch, not port contention -- retrying usually",
        "clears it. It shows up most often right after another process was",
        "killed mid-transaction, so give it a few seconds first.",
        "",
        "If it persists, check the 12V supply and the 3-pin daisy-chain: a",
        "marginal connection produces exactly this.",
    ])


def connect_with_retry(port: str, robot_id: str, attempts: int = 3):
    """Connect, retrying transient bus errors but failing fast on contention.

    Killing a process mid-transaction can leave the servo bus briefly unable to
    complete a handshake. That clears on its own, so it is worth a retry -- but
    a held port never will be, so there is no point waiting on it.
    """
    last = None
    for attempt in range(1, attempts + 1):
        try:
            return arm.connect(port, robot_id, max_step=MAX_STEP)
        except Exception as e:
            last = e
            if attempt < attempts:
                # Both causes are usually transient when they happen at startup:
                # a previous server may still be releasing the port, and a bus
                # glitch clears on its own. Only give up after several tries.
                busy = "could not connect on port" in str(e).lower()
                why = "port still held" if busy else "bus glitch"
                log(f"connect attempt {attempt}/{attempts} failed ({why}); retrying in 3s")
                time.sleep(3.0)
    raise RuntimeError(connect_failure_hint(port, last)) from last


def publish(robot, kin: SO101Kinematics, mode: str, running_index=None) -> dict[str, float]:
    """Read the arm once and push joints, tool pose and skeleton to the UI."""
    joints = arm.read_joints(robot)
    q = arm.arm_vector(joints)
    frames = kin.fk_frames(q)
    jaws = kin.gripper_geometry(q, joints.get("gripper", 0.0))
    quat = quat_from_matrix(kin.fk(q)[:3, :3])

    with _state_lock:
        _state.update(
            connected=True,
            joints={k: round(v, 3) for k, v in joints.items()},
            xyz=[round(float(v), 5) for v in frames[-1][1]],
            quat=[round(v, 6) for v in quat],
            skeleton=[[round(float(c), 5) for c in p] for _, p in frames],
            jaw_fixed=[[round(float(c), 5) for c in p] for p in jaws["fixed"]],
            jaw_moving=[[round(float(c), 5) for c in p] for p in jaws["moving"]],
            mode=mode,
            running_index=running_index,
            seq=_state["seq"] + 1,
            t=time.time(),
        )
    return joints


def ramp(robot, kin, target: dict[str, float], duration: float, index=None) -> None:
    """Interpolate to `target`, publishing state as it goes so the UI animates."""
    start = arm.read_joints(robot)
    goal = dict(start)
    goal.update({k: v for k, v in target.items() if k in start})

    n = max(1, int(duration * RAMP_RATE))
    period = 1.0 / RAMP_RATE
    for i in range(1, n + 1):
        if _abort.is_set():
            return
        a = arm.smoothstep(i / n)
        robot.send_action({f"{k}.pos": start[k] + (v - start[k]) * a for k, v in goal.items()})
        publish(robot, kin, "running", index)
        time.sleep(period)
    time.sleep(0.3)


def solve(kin: SO101Kinematics, xyz, quat, q_seed, match_orientation: bool):
    """Pose IK when the waypoint carries an orientation, position-only otherwise.

    Returns (q_deg, pos_err_m, rot_err_rad_or_None).
    """
    if quat is not None and match_orientation:
        q, _, p_err, r_err = kin.ik_pose(pose_matrix(xyz, quat), q_init=q_seed)
        return q, p_err, r_err
    q, _, p_err = kin.ik(np.asarray(xyz, dtype=float), q_init=q_seed)
    return q, p_err, None


def goto_pose(robot, kin: SO101Kinematics, xyz, quat, duration: float,
              match_orientation: bool, index=None) -> bool:
    """Solve from where the arm is now and interpolate there. False if unreachable."""
    q_seed = arm.arm_vector(arm.read_joints(robot))
    q, p_err, r_err = solve(kin, xyz, quat, q_seed, match_orientation)
    if p_err > 0.005:
        log(f"ABORT: unreachable from here (off by {p_err * 1000:.0f} mm)")
        return False
    if r_err is not None and np.degrees(r_err) > 5.0:
        log(f"    note: orientation off by {np.degrees(r_err):.1f} deg (5 joints "
            "cannot always hit a full pose)")
    ramp(robot, kin, dict(zip(ARM_JOINTS, q)), duration, index)
    return True


def run_sequence(robot, kin: SO101Kinematics, waypoints: list[dict],
                 retract: list[float] | None, grip_open: float, grip_close: float,
                 cycles: int, match_orientation: bool = True) -> None:
    """Visit each point twice -- once to open, once to close -- retracting between.

    Per point:  move to p, open, retract, move to p, close, retract.

    The gripper values are one open and one close for the whole sequence, so the
    points list carries positions only. Retract moves never touch the gripper:
    after closing on an object the arm has to lift away still holding it.
    """
    if not waypoints:
        log("nothing to run -- capture some points first")
        return
    if retract is None:
        log("ABORT: no retract position set. Back-drive the arm somewhere clear "
            "and press 'set retract here' first.")
        return

    # Check everything is solvable before making the arm stiff.
    checks = [("retract", retract, None)] + [
        (w.get("name") or f"point {i + 1}", w["xyz"], w.get("quat"))
        for i, w in enumerate(waypoints)]
    for label, xyz, quat in checks:
        _, p_err, r_err = solve(kin, xyz, quat, np.zeros(5), match_orientation)
        if p_err > 0.005:
            log(f"ABORT: {label} is unreachable (off by {p_err * 1000:.0f} mm)")
            return
        if r_err is not None and np.degrees(r_err) > 5.0:
            log(f"note: {label} orientation only reachable to {np.degrees(r_err):.1f} deg")

    log(f"torque ON -- {len(waypoints)} points x{cycles}, "
        f"open={grip_open:.0f} close={grip_close:.0f}")
    robot.bus.enable_torque()
    # Assert the present pose before anything else, so enabling torque cannot
    # snap the arm toward a stale goal left in the servos.
    robot.send_action({f"{k}.pos": v for k, v in arm.read_joints(robot).items()})
    time.sleep(0.1)

    def leg(label: str, xyz, quat, grip: float, grip_label: str, index: int) -> bool:
        """move to the point -> actuate the gripper -> retract."""
        if _abort.is_set():
            log("aborted")
            return False
        log(f"  -> {label}")
        if not goto_pose(robot, kin, xyz, quat, T_TRAVEL, match_orientation, index):
            return False

        log(f"     {grip_label} ({grip:.0f})")
        ramp(robot, kin, {"gripper": float(grip)}, T_GRIPPER, index)
        time.sleep(GRIP_SETTLE)

        if _abort.is_set():
            log("aborted")
            return False
        log("     -> retract")
        # The retract point has no recorded orientation -- it is just somewhere
        # clear to wait, so let IK pick whatever wrist pose suits.
        return goto_pose(robot, kin, retract, None, T_TRAVEL, match_orientation, index)

    try:
        log("-> retract (start)")
        if not goto_pose(robot, kin, retract, None, T_TRAVEL, match_orientation):
            return

        for c in range(1, cycles + 1):
            for i, wp in enumerate(waypoints):
                label = wp.get("name") or f"point {i + 1}"
                log(f"cycle {c}/{cycles}: {label}")
                quat = wp.get("quat")
                if not leg(label, wp["xyz"], quat, grip_open, "open", i):
                    return
                if not leg(label, wp["xyz"], quat, grip_close, "close", i):
                    return
        log("sequence complete")
    finally:
        robot.bus.disable_torque()
        log("torque OFF -- back-drivable again")


def replay_demo(robot, kin: SO101Kinematics, name: str, speed: float, cycles: int,
                lookahead: float = REPLAY_LOOKAHEAD) -> None:
    """Replay a hand-demonstrated trajectory with torque on.

    Joint angles go back verbatim, so the arm reproduces exactly what was shown
    -- no IK, so no chance of picking a different elbow configuration or failing
    on a pose the demonstration passed through happily.
    """
    data = demos.load_recording(name)
    if not data or not data.get("samples"):
        log(f"ABORT: recording '{name}' is missing or empty")
        return

    joint_names = data.get("joints", RECORD_JOINTS)
    # Position-mode servos trail a moving target by a roughly fixed time, so
    # reading the trajectory slightly ahead cancels most of that lag.
    lookahead = float(lookahead)
    arr = demos.smooth_samples(data["samples"])
    duration = float(arr[-1, 0])
    speed = max(0.1, min(4.0, float(speed)))

    log(f"torque ON -- replaying '{name}' ({duration:.1f}s at {speed:g}x) x{cycles}")
    robot.bus.enable_torque()
    robot.send_action({f"{k}.pos": v for k, v in arm.read_joints(robot).items()})
    time.sleep(0.1)

    try:
        # Ease into the demo's first pose before starting the clock; the arm is
        # wherever you left it, which may be nowhere near where the demo began.
        first = demos.resample(arr, 0.0)
        log("  -> moving to the start of the demonstration")
        ramp(robot, kin, dict(zip(joint_names, first)), 2.0)

        period = 1.0 / RAMP_RATE
        for c in range(1, cycles + 1):
            log(f"  cycle {c}/{cycles}")
            t0 = time.perf_counter()
            while True:
                if _abort.is_set():
                    log("aborted")
                    return
                elapsed = (time.perf_counter() - t0) * speed
                if elapsed > duration:
                    break
                target = demos.resample(arr, min(elapsed + lookahead, duration))
                robot.send_action({f"{n}.pos": float(v) for n, v in zip(joint_names, target)})
                with _state_lock:
                    _state["replay_progress"] = round(elapsed / duration, 3)
                publish(robot, kin, "running")
                time.sleep(period)
        log("replay complete")
    finally:
        with _state_lock:
            _state["replaying"] = None
            _state["replay_progress"] = 0.0
        robot.bus.disable_torque()
        log("torque OFF -- back-drivable again")


def poll_arm(port: str, robot_id: str, rate: float) -> None:
    """Own the robot: monitor continuously, and execute queued commands."""
    kin = SO101Kinematics()
    robot = None
    try:
        # Two very different failures, and telling them apart is the whole
        # diagnosis: the port won't OPEN (something else holds it) versus the
        # port opens fine but nothing ANSWERS (the servos have no power).
        try:
            missing = arm.preflight(port, timeout=0.0)
        except Exception as e:
            raise RuntimeError(connect_failure_hint(port, e)) from e

        if missing:
            with _state_lock:
                _state["error"] = (
                    f"motor ids {missing} not responding on {port} -- "
                    "check the 12V supply, USB powers the board but not the servos"
                )
            return

        robot = connect_with_retry(port, robot_id)

        robot.bus.disable_torque()
        saved = load_saved()
        with _state_lock:
            _state.update(saved)
            _state["recordings"] = demos.list_recordings()
            _state["error"] = None
        log("connected -- torque OFF, arm is back-drivable")

        period = 1.0 / rate
        # The Feetech bus drops the odd status packet. A single bad read used to
        # kill this thread and take the whole server with it, so transient
        # failures are absorbed and only a sustained run of them is fatal.
        MAX_CONSECUTIVE_FAILURES = 25
        failures = 0

        while not _stop.is_set():
            t0 = time.perf_counter()

            try:
                try:
                    cmd = _commands.get_nowait()
                except queue.Empty:
                    cmd = None

                if cmd is not None:
                    handle_command(cmd, robot, kin)

                joints = publish(robot, kin, "recording" if _rec["active"] else "monitor")

                if _rec["active"]:
                    _rec["samples"].append(
                        [time.perf_counter() - _rec["t0"]] + [joints[n] for n in RECORD_JOINTS])
                    with _state_lock:
                        _state["record_samples"] = len(_rec["samples"])
                        _state["record_duration"] = round(_rec["samples"][-1][0], 2)

                if failures:
                    log(f"bus recovered after {failures} failed cycle(s)")
                    failures = 0
            except Exception as e:
                failures += 1
                if failures == 1:
                    log(f"bus glitch ({type(e).__name__}) -- retrying")
                if failures >= MAX_CONSECUTIVE_FAILURES:
                    raise RuntimeError(
                        f"bus unusable: {failures} consecutive failures, last was {e}. "
                        "Check the 12V supply and the 3-pin daisy-chain."
                    ) from e
                time.sleep(0.1)
                continue
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except Exception as e:
        with _state_lock:
            _state["error"] = f"{type(e).__name__}: {e}"
            _state["connected"] = False
    finally:
        if _rec["active"] and len(_rec["samples"]) >= 5:
            demos.save_recording(_rec["name"], RECORD_JOINTS, _rec["samples"])
            print("saved in-progress recording before shutdown")
        _rec["active"] = False
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass
        with _state_lock:
            _state["connected"] = False
            _state["mode"] = "stopped"


def handle_command(cmd: dict, robot, kin: SO101Kinematics) -> None:
    action = cmd.get("action")

    if action == "capture":
        joints = arm.read_joints(robot)
        T = kin.fk(arm.arm_vector(joints))
        with _state_lock:
            wps = list(_state["waypoints"])
        wps.append({
            "name": cmd.get("name") or f"p{len(wps) + 1}",
            "xyz": [round(float(v), 5) for v in T[:3, 3]],
            # Captured straight off the arm, so the full 6-DOF pose is by
            # construction reachable -- which a hand-typed one would not be.
            "quat": [round(v, 6) for v in quat_from_matrix(T[:3, :3])],
        })
        with _state_lock:
            _state["waypoints"] = wps
        persist()
        log(f"captured {wps[-1]['name']} at {np.round(T[:3, 3], 4).tolist()} with orientation")

    elif action == "set_retract":
        joints = arm.read_joints(robot)
        xyz = [round(float(v), 5) for v in kin.fk_position(arm.arm_vector(joints))]
        with _state_lock:
            _state["retract"] = xyz
        persist()
        log(f"retract position set to {xyz}")

    elif action == "clear_retract":
        with _state_lock:
            _state["retract"] = None
        persist()
        log("retract position cleared")

    elif action == "delete":
        i = int(cmd.get("index", -1))
        with _state_lock:
            wps = list(_state["waypoints"])
        if 0 <= i < len(wps):
            removed = wps.pop(i)
            with _state_lock:
                _state["waypoints"] = wps
            persist()
            log(f"deleted {removed['name']}")

    elif action == "clear":
        with _state_lock:
            _state["waypoints"] = []
        persist()
        log("cleared all waypoints")

    elif action in ("set_gripper_open", "set_gripper_close"):
        key = "gripper_open" if action.endswith("open") else "gripper_close"
        value = float(np.clip(float(cmd.get("value", 50.0)), 0.0, 100.0))
        with _state_lock:
            _state[key] = value
        persist()
        log(f"{key.replace('_', ' ')} set to {value:.0f}")

    elif action == "record_start":
        if _rec["active"]:
            log("already recording")
        else:
            robot.bus.disable_torque()  # must be back-drivable to demonstrate
            _rec.update(active=True, t0=time.perf_counter(), samples=[],
                        name=cmd.get("name") or "")
            with _state_lock:
                _state.update(recording=True, record_name=_rec["name"],
                              record_samples=0, record_duration=0.0)
            log("RECORDING -- move the arm through the task, then press stop")

    elif action == "record_stop":
        if not _rec["active"]:
            log("not recording")
        else:
            _rec["active"] = False
            samples = _rec["samples"]
            with _state_lock:
                _state.update(recording=False, record_samples=0, record_duration=0.0)
            if len(samples) < 5:
                log("discarded -- too short to be a demonstration")
            else:
                slug = demos.save_recording(_rec["name"], RECORD_JOINTS, samples)
                with _state_lock:
                    _state["recordings"] = demos.list_recordings()
                log(f"saved '{slug}': {len(samples)} samples over {samples[-1][0]:.1f}s")

    elif action == "delete_recording":
        if demos.delete_recording(cmd.get("name", "")):
            with _state_lock:
                _state["recordings"] = demos.list_recordings()
            log(f"deleted recording '{cmd.get('name')}'")

    elif action == "replay":
        _abort.clear()
        name = cmd.get("name", "")
        with _state_lock:
            _state["replaying"] = name
        try:
            replay_demo(robot, kin, name, float(cmd.get("speed", 1.0)),
                        int(cmd.get("cycles", 1)),
                        float(cmd.get("lookahead", REPLAY_LOOKAHEAD)))
        except Exception as e:
            log(f"replay failed: {type(e).__name__}: {e}")
            try:
                robot.bus.disable_torque()
            except Exception:
                pass
        finally:
            with _state_lock:
                _state["replaying"] = None

    elif action == "set_match_orientation":
        with _state_lock:
            _state["match_orientation"] = bool(cmd.get("value", True))
            v = _state["match_orientation"]
        persist()
        log(f"match orientation: {'on' if v else 'off'}")

    elif action == "grip_preview":
        # Drive the jaw to one of the two values so you can eyeball it against a
        # real object. Only the gripper moves; the arm stays limp.
        key = "gripper_open" if cmd.get("which") == "open" else "gripper_close"
        with _state_lock:
            value = float(_state[key])
        log(f"previewing {key.replace('_', ' ')} ({value:.0f})")
        robot.bus.enable_torque(["gripper"])
        try:
            ramp(robot, kin, {"gripper": value}, T_GRIPPER)
            time.sleep(GRIP_SETTLE)
        finally:
            robot.bus.disable_torque(["gripper"])

    elif action == "run":
        _abort.clear()
        with _state_lock:
            wps = list(_state["waypoints"])
            retract = _state.get("retract")
            g_open = float(_state["gripper_open"])
            g_close = float(_state["gripper_close"])
            match_o = bool(_state.get("match_orientation", True))
        try:
            run_sequence(robot, kin, wps, retract, g_open, g_close,
                         int(cmd.get("cycles", 1)), match_o)
        except Exception as e:
            log(f"run failed: {type(e).__name__}: {e}")
            try:
                robot.bus.disable_torque()
            except Exception:
                pass
        finally:
            with _state_lock:
                _state["running_index"] = None


class QuietServer(ThreadingHTTPServer):
    """ThreadingHTTPServer that does not shout when a browser walks away.

    Refreshing the page or closing the tab tears down an in-flight SSE response,
    which surfaces as a connection reset well outside the handler's own try block.
    That is normal, not an error worth a traceback.
    """

    daemon_threads = True

    def handle_error(self, request, client_address):
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):  # keep the console free for real output
        pass

    def _send(self, code, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        if self.path.split("?")[0] != "/api":
            self._send(404, b"not found", "text/plain")
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            cmd = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            self._send(400, b'{"ok":false}', "application/json")
            return

        # "abort" must not queue behind a running sequence -- it has to land now.
        if cmd.get("action") == "abort":
            _abort.set()
        else:
            _commands.put(cmd)
        self._send(200, b'{"ok":true}', "application/json")

    def do_GET(self) -> None:
        path = self.path.split("?")[0]
        if path == "/":
            self._send(200, (WEB_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
        elif path == "/vendor/three.module.js":
            self._send(200, (WEB_DIR / "vendor" / "three.module.js").read_bytes(),
                       "text/javascript; charset=utf-8")
        elif path == "/state":
            with _state_lock:
                body = json.dumps(_state).encode()
            self._send(200, body, "application/json")
        elif path == "/events":
            self._stream_events()
        else:
            self._send(404, b"not found", "text/plain")

    def _stream_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        last = -1
        try:
            while not _stop.is_set():
                with _state_lock:
                    seq = _state.get("seq", 0)
                    payload = json.dumps(_state) if seq != last else None
                if payload is not None:
                    last = seq
                    self.wfile.write(f"data: {payload}\n\n".encode())
                    self.wfile.flush()
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # browser navigated away or refreshed


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default=arm.DEFAULT_PORT, help="serial port")
    p.add_argument("--id", default=arm.DEFAULT_ID)
    p.add_argument("--http-port", type=int, default=8000)
    p.add_argument("--rate", type=float, default=30.0, help="arm polling Hz")
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args()

    poller = threading.Thread(target=poll_arm, args=(args.port, args.id, args.rate), daemon=True)
    poller.start()

    httpd = QuietServer(("127.0.0.1", args.http_port), Handler)
    url = f"http://localhost:{args.http_port}"
    print(f"serving {url}")
    print("Ctrl+C to stop")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        _stop.set()
        _abort.set()
        httpd.shutdown()
        poller.join(timeout=5.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
