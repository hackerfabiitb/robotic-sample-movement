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
from kinematics import ARM_JOINTS, SO101Kinematics

WEB_DIR = Path(__file__).parent / "web"
WAYPOINT_FILE = Path(__file__).parent / "waypoints.json"

# Motion tuning for playback.
T_TRAVEL = 3.0
T_GRIPPER = 1.0
GRIP_SETTLE = 0.4
RAMP_RATE = 30.0
MAX_STEP = 12.0

_state: dict = {
    "connected": False,
    "error": None,
    "mode": "starting",
    "seq": 0,
    "waypoints": [],
    "log": [],
    "running_index": None,
}
_state_lock = threading.Lock()
_commands: queue.Queue = queue.Queue()
_stop = threading.Event()
_abort = threading.Event()


def log(msg: str) -> None:
    with _state_lock:
        _state["log"] = (_state["log"] + [msg])[-40:]
        _state["seq"] += 1
    print(msg)


def load_waypoints() -> list[dict]:
    if WAYPOINT_FILE.exists():
        try:
            return json.loads(WAYPOINT_FILE.read_text())
        except Exception:
            pass
    return []


def save_waypoints(wps: list[dict]) -> None:
    WAYPOINT_FILE.write_text(json.dumps(wps, indent=2))


def port_busy_hint(port: str, e: Exception) -> str:
    """Explain a failure to OPEN the serial port, which is nearly always contention.

    A serial port is exclusive on Windows, so a second opener is simply refused.
    lerobot reports this as 'Could not connect on port ... try lerobot-find-port',
    which sends you looking for the wrong problem when the port is perfectly fine
    and merely taken.
    """
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
    ])


def publish(robot, kin: SO101Kinematics, mode: str, running_index=None) -> dict[str, float]:
    """Read the arm once and push joints, tool pose and skeleton to the UI."""
    joints = arm.read_joints(robot)
    q = arm.arm_vector(joints)
    frames = kin.fk_frames(q)
    jaws = kin.gripper_geometry(q, joints.get("gripper", 0.0))

    with _state_lock:
        _state.update(
            connected=True,
            joints={k: round(v, 3) for k, v in joints.items()},
            xyz=[round(float(v), 5) for v in frames[-1][1]],
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


def run_sequence(robot, kin: SO101Kinematics, waypoints: list[dict], cycles: int) -> None:
    """Play the recorded waypoints back with torque on."""
    if not waypoints:
        log("nothing to run -- capture some waypoints first")
        return

    # Check every target is solvable before making the arm stiff.
    for i, wp in enumerate(waypoints):
        _, _, err = kin.ik(np.array(wp["xyz"]), q_init=np.zeros(5))
        if err > 0.005:
            log(f"ABORT: waypoint {i + 1} is unreachable (off by {err * 1000:.0f} mm)")
            return

    log(f"torque ON -- running {len(waypoints)} waypoints x{cycles}")
    robot.bus.enable_torque()
    # Assert the present pose before anything else, so enabling torque cannot
    # snap the arm toward a stale goal left in the servos.
    robot.send_action({f"{k}.pos": v for k, v in arm.read_joints(robot).items()})
    time.sleep(0.1)

    try:
        for c in range(1, cycles + 1):
            for i, wp in enumerate(waypoints):
                if _abort.is_set():
                    log("aborted")
                    return
                joints = arm.read_joints(robot)
                q, _, err = kin.ik(np.array(wp["xyz"]), q_init=arm.arm_vector(joints))
                if err > 0.005:
                    log(f"ABORT: waypoint {i + 1} unreachable from here")
                    return
                label = wp.get("name") or f"waypoint {i + 1}"
                log(f"cycle {c}/{cycles}: -> {label}")
                ramp(robot, kin, dict(zip(ARM_JOINTS, q)), T_TRAVEL, i)

                if wp.get("gripper") is not None:
                    log(f"    gripper -> {wp['gripper']:.0f}")
                    ramp(robot, kin, {"gripper": float(wp["gripper"])}, T_GRIPPER, i)
                    time.sleep(GRIP_SETTLE)
        log("sequence complete")
    finally:
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
            raise RuntimeError(port_busy_hint(port, e)) from e

        if missing:
            with _state_lock:
                _state["error"] = (
                    f"motor ids {missing} not responding on {port} -- "
                    "check the 12V supply, USB powers the board but not the servos"
                )
            return

        try:
            robot = arm.connect(port, robot_id, max_step=MAX_STEP)
        except Exception as e:
            raise RuntimeError(port_busy_hint(port, e)) from e

        robot.bus.disable_torque()
        with _state_lock:
            _state["waypoints"] = load_waypoints()
            _state["error"] = None
        log("connected -- torque OFF, arm is back-drivable")

        period = 1.0 / rate
        while not _stop.is_set():
            t0 = time.perf_counter()

            try:
                cmd = _commands.get_nowait()
            except queue.Empty:
                cmd = None

            if cmd is not None:
                handle_command(cmd, robot, kin)

            publish(robot, kin, "monitor")
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except Exception as e:
        with _state_lock:
            _state["error"] = f"{type(e).__name__}: {e}"
            _state["connected"] = False
    finally:
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
        xyz = kin.fk_position(arm.arm_vector(joints))
        with _state_lock:
            wps = list(_state["waypoints"])
        wps.append({
            "name": cmd.get("name") or f"p{len(wps) + 1}",
            "xyz": [round(float(v), 5) for v in xyz],
            "gripper": round(float(joints.get("gripper", 0.0)), 1),
        })
        save_waypoints(wps)
        with _state_lock:
            _state["waypoints"] = wps
        log(f"captured {wps[-1]['name']} at {np.round(xyz, 4).tolist()} grip {wps[-1]['gripper']:.0f}")

    elif action == "delete":
        i = int(cmd.get("index", -1))
        with _state_lock:
            wps = list(_state["waypoints"])
        if 0 <= i < len(wps):
            removed = wps.pop(i)
            save_waypoints(wps)
            with _state_lock:
                _state["waypoints"] = wps
            log(f"deleted {removed['name']}")

    elif action == "clear":
        save_waypoints([])
        with _state_lock:
            _state["waypoints"] = []
        log("cleared all waypoints")

    elif action == "set_gripper":
        with _state_lock:
            wps = list(_state["waypoints"])
        i = int(cmd.get("index", -1))
        if 0 <= i < len(wps):
            wps[i]["gripper"] = float(cmd.get("value", 50.0))
            save_waypoints(wps)
            with _state_lock:
                _state["waypoints"] = wps
            log(f"{wps[i]['name']} gripper set to {wps[i]['gripper']:.0f}")

    elif action == "run":
        _abort.clear()
        with _state_lock:
            wps = list(_state["waypoints"])
        try:
            run_sequence(robot, kin, wps, int(cmd.get("cycles", 1)))
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
