# SO-101 Robot Arm

Control environment for an [SO-101](https://huggingface.co/docs/lerobot/so101) arm
(6x Feetech STS3215 servos) driven by [LeRobot](https://github.com/huggingface/lerobot)
on Windows.

## Prerequisites

- **conda** (Anaconda/Miniconda). Assumed at `%USERPROFILE%\anaconda3`; adjust
  [activate.ps1](activate.ps1) if yours lives elsewhere.
- The arm's **USB cable** *and* its **12V power supply**. Both are required —
  USB powers the controller board's logic, but the servos run off the barrel
  jack. With USB alone the COM port appears and no motor ever answers.
- On a Waveshare controller board, both jumpers set to the **B (USB)** channel.

## Environment setup

The environment lives in `.venv/` inside this directory and is gitignored.

LeRobot targets **Python 3.12**. Conda's base interpreter here is 3.14, which is
ahead of what the torch wheels support, so create the env with an explicit 3.12:

```powershell
conda create -y -p .\.venv python=3.12
.\.venv\python.exe -m pip install -r requirements.txt
```

That takes a few minutes — it pulls torch and its dependencies.

> `.venv` is a conda environment at a path prefix, not a `python -m venv` venv,
> so it has no `Scripts\activate.ps1` of its own. Use the wrapper below.

## Activating

```powershell
. .\activate.ps1
```

Dot-sourced, so it modifies the current shell. This puts `python` and all the
`lerobot-*` commands on PATH. Verify:

```powershell
python --version              # -> Python 3.12.13
lerobot-find-port --help
```

Without activating, you can always call the interpreter directly:
`.\.venv\python.exe script.py`

## Hardware bring-up

### 1. Find the serial port

```powershell
lerobot-find-port
```

It lists ports, asks you to unplug the arm, and reports which one vanished. If
only one port is present that step is moot — on this machine the adapter is
**COM3** (`USB-Enhanced-SERIAL CH343`). Substitute your own port below.

### 2. Check whether the motors are already configured

```powershell
python scan_bus.py COM3
```

Do this **before** anything else, especially on a second-hand arm. Motor IDs are
stored in each servo's own EEPROM, so an arm configured on another PC arrives
already done and you can skip straight to calibration.

[scan_bus.py](scan_bus.py) pings each ID individually rather than broadcasting.
A broadcast makes every motor reply at once, so several factory-default servos
(all ID 1) collide into garbage that looks identical to an empty bus.

How to read the result:

| Output | Meaning |
| --- | --- |
| IDs 1–6 at `1000000` | Already configured — skip to step 4. |
| Only ID 1 | Factory default, or just one motor connected. Do step 3. |
| IDs 1–6 at another baud rate | Repurposed from another robot. Do step 3. |
| A subset of 1–6 | Partial — the rest are unconfigured or unplugged. |
| Nothing | See Troubleshooting. |

`python scan_bus.py COM3 --all-ids` sweeps every ID at every baud rate (slower).

### 3. Set motor IDs and baud rates

Only needed if step 2 says so. This writes to EEPROM, so it's a one-time job.

```powershell
lerobot-setup-motors --robot.type=so101_follower --robot.port=COM3
```

The script prompts for one motor at a time, **in reverse joint order**:
`gripper` → `wrist_roll` → `wrist_flex` → `elbow_flex` → `shoulder_lift` → `shoulder_pan`.

**Exactly one motor may be connected to the board at each prompt.** Every servo
ships with ID 1, and the bus is half-duplex — two motors sharing an address both
drive the line and the replies collide. Isolation is the only way to guarantee
you are reprogramming the motor you mean.

Do **not** try to build the chain up incrementally (configure one, re-link it,
do the next). LeRobot picks whichever motor answers first and only validates the
*model* number, not that a single motor is present — so a partially-linked chain
can silently overwrite an already-configured motor's ID. Keep all six physically
separate until the script finishes, then wire the chain up.

If the arm is already assembled, you'll have to unplug the 3-pin links between
motors to isolate each one.

### 4. Calibrate

```powershell
lerobot-calibrate --robot.type=so101_follower --robot.port=COM3 --robot.id=my_so101
```

Center every joint, press Enter, then move each joint through its full range.

Calibration is **not** stored in the motors — it's a JSON file on this PC under
`HF_LEROBOT_CALIBRATION/robots/so101_follower/<id>.json`. A second-hand arm's
calibration stayed on its previous owner's machine, so run this even when step 2
reports the motors are already configured.

## Motor map

Identical for leader and follower:

| ID | Joint |
| --- | --- |
| 1 | `shoulder_pan` |
| 2 | `shoulder_lift` |
| 3 | `elbow_flex` |
| 4 | `wrist_flex` |
| 5 | `wrist_roll` |
| 6 | `gripper` |

## Leader or follower?

Because the ID map is identical, `setup-motors` behaves the same either way —
the choice only starts to matter at calibration, where `--robot.id` and
`--teleop.id` write to different paths.

The physical tell is **gearing**: a follower uses 6x 1/345 servos, while a
leader mixes ratios (1/191, 1/345, 1/191, 1/147, 1/147, 1/147) so it can be
back-driven by hand. Check the labels on the servos.

With a single arm, use **follower** — a leader is only an input device and
can't actuate anything on its own. A follower can be driven by keyboard,
gamepad, or phone teleop, and can run trained policies.

## Cartesian control (IK)

```powershell
python move_ee.py --show                      # current joints + tool position
python move_ee.py --xyz 0.25 0.0 0.20         # move the tool point there
python move_ee.py --gripper open              # open | close | half | 0-100
python move_ee.py --xyz 0.25 0.0 0.20 --gripper close
python move_ee.py --xyz 0.25 0.0 0.20 --dry-run   # solve and print, don't move
```

Coordinates are metres in `base_link`: **+X forward** out of the base, **+Z up**,
origin at the base plate. `--show` first is the easy way to get your bearings.

### Why not lerobot's solver

`lerobot.model.kinematics.RobotKinematics` needs [`placo`](https://pypi.org/project/placo/),
which does not work on Windows. This was checked properly, not assumed:

| Route | Result |
| --- | --- |
| `pip install placo` | No Windows wheels exist at all -- PyPI publishes only `macosx_*` and `manylinux_*`. Pip falls back to source and fails building `eiquadprog`. |
| conda-forge | Native `win-64` builds **do** exist, and install cleanly with all of pinocchio/eigenpy/eiquadprog as prebuilt binaries. |
| ...but importing it | `ImportError: DLL load failed while importing placo: The specified procedure could not be found.` |
| Versions 0.9.17, 0.9.18, 0.9.19, 0.9.20 | All four fail identically. |
| In a pristine env (only python + placo) | Still fails -- so it is the conda-forge Windows build, not a local conflict. |

If you ever want the real placo, WSL is the route: the `manylinux` wheels install
with plain `pip`. You would then need to get COM3 across the WSL boundary with
`usbipd`, which is its own project.

> [!WARNING]
> Do not `conda install placo` into `.venv`. It pulls conda's MKL-backed numpy
> over the pip one, and MKL then collides with the copy torch ships: every
> `np.linalg.solve` **hard-crashes the interpreter** with no traceback. If you
> have already done it, recover with
> `.venv\python.exe -m pip install --force-reinstall --no-deps numpy==2.2.6`.

[kinematics.py](kinematics.py) instead uses a numpy-only solver driving the official URDF
([urdf/so101_new_calib.urdf](urdf/so101_new_calib.urdf), from TheRobotStudio/SO-ARM100).

Every revolute joint in that URDF has local axis `(0,0,1)`, so each link transform
is `Translate(xyz) @ RPY(rpy) @ RotZ(q)` and forward kinematics is a plain product
of 4x4s. IK is damped least squares on tool position with random restarts.

Validated over 300 random reachable targets: **291/300 converge, max error 0.34 mm**
(the rest just hit the iteration cap, still sub-millimetre). Reproduce with the
round-trip check in the git history, or by sampling `fk_position` and re-solving.

The arm has 5 positioning joints but position is only 3 constraints, so solutions
form a 2-parameter family. `ik()` seeds from the current pose and applies a
nullspace pull toward it, so successive calls stay near each other instead of
picking wildly different elbow configurations.

### Frame conventions worth knowing

`shoulder_pan` carries `rpy="3.14159 0 -3.14159"` in the URDF, a 180-degree flip.
**Positive pan therefore moves the tool toward -Y.** FK and IK agree with each
other, so the maths is consistent -- just don't expect pan sign to match a naive
right-handed reading of the base frame.

`gripper` is not part of the kinematic chain. It only opens and closes the jaw,
so it never affects where the tool point is, and is commanded separately.

### How far to trust the model

The URDF describes an ideal SO-101. Whether it matches *your* arm depends on
lerobot's calibrated zero lining up with the URDF's zero pose. The evidence is
good for the joints that dominate positioning -- comparing your calibrated sweep
against the URDF limits:

| Joint | Your span | URDF span | |
| --- | --- | --- | --- |
| `shoulder_pan` | 210.5 deg | 220.0 deg | close |
| `shoulder_lift` | 202.5 deg | 200.0 deg | close |
| `elbow_flex` | 193.8 deg | 193.7 deg | near exact |
| `wrist_flex` | 206.5 deg | 190.0 deg | over-swept ~16 deg |
| `wrist_roll` | 360.0 deg | 320.0 deg | **swept full turn** |

The three joints that set tool position agree closely, which is why Cartesian
targets land where they should. Two caveats:

- **`wrist_flex`** was swept ~16 deg wider than the URDF allows, so its zero may
  be off by up to ~8 deg. That tilts the tool slightly; it barely moves the tool point.
- **`wrist_roll`** was swept a full 360 deg, so its midpoint -- and therefore its
  zero -- is essentially arbitrary. Re-calibrate it against a physical reference
  before relying on tool *orientation*.

Position accuracy is unaffected by both. Measured tracking error moving to
`(0.25, 0, 0.20)` was **4 mm**, which is servo droop under gravity rather than
model error.

### Safety behaviour

Shared in [arm.py](arm.py) so every script inherits it:

- On connect, the present pose is immediately re-asserted as the goal, so a stale
  `Goal_Position` in the servos can't yank the arm.
- All motion is interpolated with smoothstep easing, never stepped.
- `max_relative_target` (default 12) caps how far a goal may lead the measurement.
- Targets below `z = 0.02 m` are refused unless you pass `--allow-low`.
- IK residual over 5 mm is treated as unreachable and refuses to move.
- Joint-space interpolation is deliberate: a straight-line Cartesian path can
  cross a singularity, so the tool traces an arc instead.
- Every motion commands all six motors, holding the ones it isn't moving. That
  is partly a lerobot requirement (a dict-valued `max_relative_target` must have
  exactly the same keys as the action) and partly just safer.
- The gripper is exempt from the relative-step cap. It is geared slowly and lags
  its goal, so a cap tight enough to restrain the arm throttles the jaw on every
  step and floods the log with clamp warnings.

## Pick and place

[pick_place.py](pick_place.py) shuttles an object around a series of positions.
Everything is configured in the CONFIG block at the top -- no CLI arguments:

```powershell
python pick_place.py
```

Edit `POSITIONS` to your waypoints. For each leg the arm picks up at one position
and sets down at the next, looping for `CYCLES` passes. Set `DRY_RUN = True` to
solve and print every waypoint without moving.

Picks and places go **via a hover point** `HOVER_DZ` above the target rather than
driving straight in -- approaching from directly above stops the gripper sweeping
the object sideways, and lifting before travelling stops it dragging.

Every waypoint (and its hover) is IK-checked before the arm is even connected, so
an unreachable target costs nothing.

**Setting Z.** `POSITIONS` z-values are the height the gripper closes at, which
depends on your object and how high your table sits relative to the base plate.
To measure it: back-drive the arm so the gripper sits where you want it, then run
`python move_ee.py --show` and copy the reported Z. The shipped default of
`0.04` is deliberately on the high side -- it will miss a short object rather
than drive the gripper into the table.

## Live web GUI

```powershell
python server.py
```

Opens <http://localhost:8000>. Two modes, and the arm is stiff in only one:

| Mode | Torque | What you do |
| --- | --- | --- |
| **monitor** (default) | **off** | Back-drive the arm by hand; the readout and 3D view follow. Hit **capture** to record where it is. |
| **running** | **on** | The arm plays the recorded waypoints back. A red banner shows across the view the whole time. |

It always returns to monitor with torque off when a run ends, aborts, or fails.

### Recording and playing a sequence

The GUI holds three things: a **list of points**, a **pair of gripper values**
(one open, one close, for the whole sequence), and a **retract position**.

1. Move the gripper by hand to where you want a point.
2. **+ capture here** records the tool XYZ **and its orientation**.
3. Repeat for every point, deleting any you don't want.
4. Set the **open** and **close** gripper values. **try** drives just the jaw so
   you can check a value against a real object, leaving the arm limp.
5. Back-drive the arm somewhere clear and press **set retract here**.
6. Set **cycles**, then **run (torque ON)**. **stop** aborts mid-motion.

Each point is visited **twice** -- once to open, once to close -- retracting
after each:

```
-> retract (start)
cycle 1/1: p1
  -> p1
     open (85)
     -> retract
  -> p1
     close (12)
     -> retract
cycle 1/1: p2
  -> p2
     open (85)
     -> retract
  -> p2
     close (12)
     -> retract
```

**Retract moves never touch the gripper.** After closing on an object the arm
has to lift away still holding it, so retracting only repositions the arm.

The retract point is set separately from the list, and **playback refuses to
start without one** rather than silently running a different shape. Every point
*and* the retract are IK-checked before torque is enabled, so an unreachable
target aborts while the arm is still limp.

### Orientation

Capture records the full 6-DOF pose, stored as a quaternion, and playback
reproduces it. The **match captured orientation** checkbox turns this off and
falls back to position-only.

The arm has 5 joints, so an arbitrary 6-DOF pose is over-constrained and in
general unreachable. Poses *captured from the arm itself* always are, which is
exactly how waypoints are made -- so this works in practice while a hand-typed
orientation might not. Solving 300 captured-style poses: **300/300 converged,
max 0.10 mm and 0.52 deg**; a real pose captured off the arm reproduced to
0.050 mm and 0.069 deg.

Where a pose cannot be hit exactly, position wins -- orientation rows are
weighted down in the solver, and playback logs how many degrees it gave up.

The retract point deliberately has **no** orientation: it is just somewhere clear
to wait, so IK is free to pick whatever wrist pose suits.

Each waypoint draws a small axis triad in the 3D view, so a recorded orientation
is visible rather than merely implied by a dot.

State persists to `waypoints.json`:

```json
{
  "retract": [0.25, 0.0, 0.22],
  "gripper_open": 85.0,
  "gripper_close": 12.0,
  "match_orientation": true,
  "waypoints": [
    {"name": "p1", "xyz": [0.22, -0.1, 0.1], "quat": [0.49, 0.21, 0.84, -0.1]}
  ]
}
```

Both older layouts still load: a bare list of points, and the version where each
point carried its own `gripper` value (that key is dropped, since the open/close
pair now covers it).

> [!NOTE]
> A run ends by disabling torque so you can back-drive again. The arm is at the
> retract point when that happens, so it will **drop** from there. Put the
> retract somewhere a fall is harmless, or catch it.

### Gripper rendering

Both jaws are drawn: the **fixed** jaw in blue and the **moving** jaw in amber.

The URDF has no separate link for the fixed jaw -- it is part of `gripper_link`'s
mesh -- so it is drawn as that link's origin out to the tool point. The moving
jaw pivots on the real `gripper` joint; its length comes from that link's centre
of mass sitting at `y = -0.030` in its own frame, i.e. a bar of about twice that.

lerobot's 0-100 gripper value maps linearly onto the URDF's joint limits. Checked
numerically, the jaw tip travels 25mm from the tool point at 0 to 120mm at 100 --
confirming 0 = closed, 100 = open, which matches the arm's behaviour.

### How it fits together

No new dependencies -- standard library plus a vendored copy of three.js.

- **Transport** is Server-Sent Events for the live stream (Python to browser) and
  plain `POST /api` for commands the other way. One-way streaming plus occasional
  commands does not justify a websocket library.
- **One thread owns the robot.** A serial port is exclusive, so the poller holds
  the connection and HTTP handlers hand it commands through a `queue`. Playback
  runs *inside* that thread, publishing state as it interpolates, which is why
  the 3D view animates during a run.
- **`abort` bypasses the queue** and sets an `Event` directly -- queued behind a
  running sequence, a stop button would be useless.
- **Forward kinematics stays in Python.** The browser receives already-computed
  3D points for the skeleton and both jaws, and just draws them. Nothing about
  the URDF is reimplemented in JavaScript.
- **three.js is vendored** at [web/vendor/](web/vendor/) rather than pulled from
  a CDN, so the GUI works at a bench with no internet. It is ~1.3 MB, the bulk of
  this repo. Orbit control is a ~20-line pointer handler rather than another
  vendored file.
- The camera is set `up = (0,0,1)`: the URDF is Z-up, three.js defaults to Y-up.
- The waypoint list only re-renders when it actually changes -- rebuilding it at
  30 Hz would wipe out whatever you were typing in a gripper field.

Measured: the arm reads at **~650 Hz** over serial, so the 30 Hz default stream
has plenty of headroom. `--rate` raises it.

```powershell
python server.py --http-port 8080 --rate 60 --no-browser
```

## Jitter

The arm is visibly rough during motion. Measured, rather than guessed at, by
sweeping one joint and taking the RMS of the high-frequency part of its velocity
on a fixed 100 Hz analysis grid (resampling first, so encoder quantisation at
short intervals cannot masquerade as roughness).

**Raising the command rate does not help.** This was the obvious hypothesis --
30 Hz goals to a position-mode servo ought to stair-step -- and it is wrong:

| Command rate | Goal step | Roughness (deg/s) |
| --- | --- | --- |
| 30 Hz | 0.67 deg | 2.2 |
| 60 Hz | 0.33 deg | 2.5 |
| 120 Hz | 0.17 deg | 2.4 |
| 200 Hz | 0.10 deg | 3.0 |

Flat, and slightly worse at the top. So going to 650 Hz would gain nothing. For
reference the bus could take it -- 569 Hz for commands, 307 Hz for a full
read-and-command loop -- the rate simply is not the bottleneck.

**At rest the arm is rock steady**: position noise measured 0.000 deg holding a
fixed goal. The servo is not hunting, so this is not a control-loop oscillation.

**Servo acceleration is the one real software lever.** lerobot writes
`Acceleration = Maximum_Acceleration = 254`, the maximum, so every goal is chased
flat out ([feetech.py:209-217](.venv/Lib/site-packages/lerobot/motors/feetech/feetech.py#L209-L217)):

| Acceleration | Roughness (deg/s) | Lag (deg) |
| --- | --- | --- |
| 254 (lerobot default) | 3.2 | 1.23 |
| 96 | 2.6 | 1.05 |
| 24 | 2.4 | 0.88 |
| 12 | 2.2 | 0.75 |

Backing it off is ~25% smoother *and* tracks better, because the servo follows
the ramp instead of overshooting each goal.

Gains barely matter: P = 16 (lerobot's value) beat 32 on both roughness and hold;
D = 0 was marginally better than lerobot's 32.

[arm.py](arm.py) now applies `Acceleration = 24`, `D = 0` on connect, in
`tune_servos()`. It has to run *after* `robot.connect()`, since lerobot's own
`configure()` resets acceleration to maximum every time.

**What is left is mechanical, and your instinct about backlash is right.** Rate
does nothing, gains do nothing, it holds perfectly still at rest, and the best
tuning still leaves ~2.2 deg/s. That signature -- rough only while moving, silent
when stopped -- is stick-slip and lash in the 1/345 plastic gearboxes, which no
amount of software will remove. Software can only stop making it worse.

One thing that genuinely helps in practice: **approach every point from the same
direction**. Backlash is hysteresis, so consistent approach makes the error
repeatable even though it does not make it smaller. The retract-then-approach
playback shape already does this.

## Speed

Motion is 2x faster: `T_TRAVEL` 3.0s -> 1.5s and `T_GRIPPER` 1.0s -> 0.5s, in
both [server.py](server.py) and [pick_place.py](pick_place.py); `move_ee.py`
defaults to `--duration 2.0`. Measured roughness at 1.5s is the same as at 3.0s
(2.3 vs 2.4 deg/s), so the speed costs no smoothness.

## Teach by demonstration

Show the arm a task by hand, then have it repeat it.

1. Type a name, press **&#9679; record**. Torque is released.
2. Move the arm through the whole task by hand, gripper included.
3. Press **&#9632; stop**. The trajectory is saved under `recordings/`.
4. Press **play** on it. Set **speed** (0.1-4x) and **cycles** first.

This is separate from the waypoint system: waypoints are discrete poses you
choose, a demonstration is a continuous trajectory you perform.

### Recorded in joint space, deliberately

What you physically showed the arm *is* a joint trajectory, so replaying those
angles reproduces it exactly. Going via Cartesian would mean solving IK on every
frame, which can pick a different elbow configuration, wander near a
singularity, or fail outright on a pose the demo passed through happily.

All six joints are captured, gripper included, at the poller's 30 Hz.

Hand-guided motion is shaky, so [demos.py](demos.py) moving-averages each channel
before replay (`SMOOTH_WINDOW = 7`, ~0.23s). The **raw** samples are what gets
stored, so changing the window re-reads cleanly rather than degrading a
recording permanently.

### Lookahead

Position-mode servos trail a moving target by a roughly fixed time, so replay
reads the trajectory slightly ahead to cancel it. Measured on a 6s sweep, mean
error against the commanded trajectory across all six joints:

| Lookahead | Mean lag | Worst |
| --- | --- | --- |
| 0.00s | 2.09 deg | 9.29 |
| 0.05s | 1.52 deg | 7.19 |
| 0.10s | 1.01 deg | 5.57 |
| 0.15s | 0.70 deg | 5.31 |

`REPLAY_LOOKAHEAD` defaults to **0.10s** -- a measured 2x improvement, staying
short of the largest value tested since a hand demo has sharper reversals than
the smooth sweep this was tuned on. The gripper lags most (it is the slowest
geared), which is why it dominates the worst-case column.

Replay always eases into the demo's first pose over 2s before starting the
clock, since the arm may be nowhere near where the demonstration began.

### Robustness

The Feetech bus drops the occasional status packet. Two fixes came out of
hitting this repeatedly:

- **The poll loop survives it.** A single failed read used to kill the poller
  thread and leave the server permanently dead while still serving HTTP. Now
  transient failures are absorbed and logged, and only 25 consecutive failures
  are fatal.
- **Connect retries.** Startup retries 3 times, 3s apart, which covers both a
  previous server still releasing the port and a bus glitch after a hard kill.
- **A failed connect releases the port.** `robot.connect()` opens the serial
  port before the writes that follow it, so a glitch during those writes used to
  leave the handle open. The next attempt then collided with it and reported the
  port as held by another process -- which it was: the previous attempt. Seen in
  the wild as `attempt 1 failed (bus glitch)` followed by `attempt 2 failed (port
  still held)`, a cascade entirely of its own making. `arm.connect()` now closes
  the port on every failure path.

The connect error message also distinguishes the two causes now, because they
look identical but have opposite fixes -- kill a process, versus just wait. The
old message blamed contention for everything and would send you hunting a
process that was not there.

`recordings/` is gitignored: demonstrations are yours, not part of the project.

## Troubleshooting

**Nothing responds to `scan_bus.py`, but the COM port exists.** The port comes
from the USB-serial chip on the controller board, which is USB-powered — its
presence says nothing about the servos. Check the 12V supply first, then the
3-pin cable, then the Waveshare B-channel jumpers.

**`lerobot-find-port` crashes with an EOF error.** It's interactive and needs a
real terminal; it can't run piped or from a script.

**Only ID 1 shows up with the whole chain connected.** Expected for unconfigured
motors — they're all colliding on address 1. Proceed with step 3.

**Gripper won't hold an object.** The follower caps gripper torque at 50% and
`Overload_Torque` at 25% to avoid burning the servo out
([so_follower.py:168-171](.venv/Lib/site-packages/lerobot/robots/so_follower/so_follower.py#L168-L171)).
An `[RxPacketError] Overload error!` on id 6 means it stalled -- back the
target off rather than raising the limits.

**`Could not connect on port 'COM3'` / `ConnectionError` on startup.** The port
exists but is already held by another process -- usually an earlier `server.py`
still running in another terminal. Serial ports are exclusive on Windows, so the
second opener is simply refused. lerobot's message tells you to run
`lerobot-find-port`, which sends you after the wrong problem. Find the holder:

```powershell
Get-CimInstance Win32_Process -Filter "Name LIKE '%python%'" |
    Select-Object ProcessId, CommandLine
Stop-Process -Id <pid> -Force
```

Note that `pkill` from Git Bash does **not** reliably kill these -- use
`Stop-Process`. `server.py` now detects this case and prints the above.

**Torch/CUDA.** On Windows pip installs the default wheel. LeRobot falls back to
`pyav` for video decoding, so a separate ffmpeg install isn't required.

## Files

| File | Purpose |
| --- | --- |
| [requirements.txt](requirements.txt) | Pinned dependency set |
| [activate.ps1](activate.ps1) | Dot-source to activate `.venv` |
| [scan_bus.py](scan_bus.py) | Report which motor IDs/baud rates are live |
| [move_middle.py](move_middle.py) | Move every joint to its calibrated midpoint |
| [kinematics.py](kinematics.py) | URDF-driven FK and IK, numpy only |
| [arm.py](arm.py) | Shared connect / read / interpolated-move helpers |
| [move_ee.py](move_ee.py) | Cartesian tool moves and gripper control |
| [pick_place.py](pick_place.py) | Shuttle an object between configured positions |
| [server.py](server.py) | Live web GUI server: monitor, waypoints, demonstrations |
| [demos.py](demos.py) | Storage and playback maths for demonstrations |
| [web/index.html](web/index.html) | three.js front end for the monitor |
