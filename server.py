r"""Live web GUI for the SO-101: stream joint state and pose to a browser.

    .venv\python.exe server.py

Then open http://localhost:8000

Torque is DISABLED while this runs, so you can back-drive the arm by hand and
watch the readout follow. Nothing here ever commands a position -- it is a
read-only monitor, which is what makes it safe to leave running.

Transport is Server-Sent Events over the standard library's http.server. The
data only flows one way, so SSE avoids pulling in a websocket dependency, and
forward kinematics stays in Python -- the browser just draws the points it is
given rather than duplicating the URDF maths in JavaScript.
"""

from __future__ import annotations

import argparse
import json
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

# Shared between the poller thread and the request handlers.
_state: dict = {"connected": False, "error": None, "seq": 0}
_state_lock = threading.Lock()
_stop = threading.Event()


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


def poll_arm(port: str, robot_id: str, rate: float) -> None:
    """Read the arm forever, publishing joint angles, tool pose and skeleton."""
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
            robot = arm.connect(port, robot_id)
        except Exception as e:
            raise RuntimeError(port_busy_hint(port, e)) from e
        # Read-only monitor: let go of the joints so they can be moved by hand.
        robot.bus.disable_torque()

        with _state_lock:
            _state["connected"] = True
            _state["error"] = None

        period = 1.0 / rate
        while not _stop.is_set():
            t0 = time.perf_counter()
            joints = arm.read_joints(robot)
            q = arm.arm_vector(joints)
            frames = kin.fk_frames(q)
            tip = frames[-1][1]

            with _state_lock:
                _state.update(
                    joints={k: round(v, 3) for k, v in joints.items()},
                    xyz=[round(float(v), 5) for v in tip],
                    skeleton=[[round(float(c), 5) for c in p] for _, p in frames],
                    links=[n for n, _ in frames],
                    seq=_state["seq"] + 1,
                    t=time.time(),
                )
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except Exception as e:  # surface it in the UI rather than dying silently
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
    print("torque is OFF -- move the arm by hand and watch it follow")
    print("Ctrl+C to stop")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        _stop.set()
        httpd.shutdown()
        poller.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
