# Working agreements for this repo

## Every prompt

- **Commit after every prompt**, once the work is done and verified. Small,
  self-contained commits with a message explaining *why*, not just what.
- **Update [README.md](README.md) when the change warrants it** — new commands,
  changed defaults, new measured numbers, new failure modes. The README is the
  entry point for someone returning to this repo after months; keep it true.
- Do not commit `waypoints.json` or `recordings/` (gitignored — user data).

## Hardware is real

This repo drives a physical arm. Before anything that moves it:

- The **12V barrel jack** must be connected. USB powers the controller board's
  logic but not the servos; the COM port appearing says nothing about whether
  the motors have power.
- Prefer `--dry-run` / read-only paths first where a script offers one.
- A run ends with torque off, so the arm **drops** from wherever it stopped.
- **Keep `wrist_roll` at about -99 deg** (`jog.SAFE_ROLL`). The wrist camera is
  bolted to one side of the wrist, and that roll is what keeps it on the *upper*
  side. Position-only IK treats roll as free and will return a solution ~99 deg
  away, which rolls the camera underneath, where it strikes the arm — this has
  already happened once. Use `jog.ik_fixed_roll`, not `kinematics.ik`, for
  anything that moves the arm near the bench.
- Two Cartesian points a short distance apart can solve to completely different
  arm configurations (wrist_flex +63 deg vs -90 deg was measured). Seed IK from
  the current pose, refuse large joint jumps, and split long travels into
  sub-moves.

## Connecting drops the arm

`SO101Follower.connect()` calls `configure()`, which does its register writes
inside `with self.bus.torque_disabled()`. The arm sags several centimetres every
time a process connects, and **anything in the gripper is dropped**. So:

- A pick-and-place must run start to finish in **one** process.
- To attach to an arm that is already holding something, open the bus directly
  (`robot.bus.connect()`) and skip `configure()` — see `pick_box.attach`.
- `arm.tune_servos` drops torque for the same reason. Read the pose *before*
  calling it and re-assert it afterwards, or the arm ratchets downward on every
  command.

## Environment gotchas

- `.venv` is a **conda prefix env**, not `python -m venv`. Use
  `.\.venv\python.exe` directly, or dot-source `activate.ps1`.
- **Never `conda install` into `.venv`.** It replaces pip's numpy with an
  MKL-backed build that collides with torch's MKL, and `np.linalg.solve` then
  hard-crashes the interpreter with no traceback.
- Kill stray processes with PowerShell `Stop-Process`. `pkill` from Git Bash
  does **not** reliably kill them, and a leftover process holds COM3, which is
  exclusive — the next connect then fails confusingly.

## Verify against the hardware

Claims about the arm's behaviour should be measured, not assumed. Several
plausible hypotheses in this repo turned out wrong when tested (raising the
command rate does not reduce jitter; placo installs on Windows but cannot
import). If a number ends up in the README, it came from a run.
