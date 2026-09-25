# SO-101 Robot Arm

Control software for an [SO-101](https://huggingface.co/docs/lerobot/so101) arm
(6x Feetech STS3215 servos) on Windows: a browser GUI with live 3D view,
Cartesian control with our own inverse kinematics, waypoint sequences, and
teach-by-demonstration.

[**Demo: moving wafer using its holder between beakers**](https://www.youtube.com/shorts/06ecVcR53Uo?feature=share)
— by Aryamman on Aug 22 2026

<a href="https://youtu.be/WDfGkfzMThQ?t=53"><img src="parallel_gripper.png" width="420" alt="A parallel gripper lifting a thin plate off a fixture"></a>

[**Parallel Gripper TODO**](https://youtu.be/WDfGkfzMThQ?t=53)

---

## Quickstart

Plug in **both** the USB cable and the **12V barrel jack** — USB powers the
controller board's logic but not the servos, and the arm is silent without it.

```powershell
cd C:\hf\automation\so101
. .\activate.ps1
python server.py
```

That opens <http://localhost:8000> automatically. Torque starts **off**, so the
arm is limp and safe to move by hand.

Prefer not to activate the environment? Call the interpreter directly:

```powershell
.venv\python.exe server.py
```

Useful flags:

```powershell
python server.py --http-port 8080 --rate 60 --no-browser
python server.py --port COM4 --id arm0        # different serial port / calibration
```

> [!NOTE]
> There is **no npm or build step**. The front end is a single static
> `web/index.html` served by `server.py`, and three.js is vendored at
> `web/vendor/`. "Run the app" means "run the Python server".

If it will not connect, jump to [Troubleshooting](#troubleshooting) — the two
usual causes are the 12V supply being off and a previous server still holding
the serial port.

---

## What this is

A single arm, no leader, so there is no teleoperation. Everything is driven
either from the browser or from small command-line scripts.

Defaults throughout: serial port **COM3**, robot id **arm0** (which selects
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/arm0.json`).

Built on [LeRobot](https://github.com/huggingface/lerobot) 0.6.1 for servo I/O
and calibration, but the kinematics, motion planning and GUI are all local —
see [Why not lerobot's solver](#why-not-lerobots-solver).

## Repo layout

| File | Purpose |
| --- | --- |
| [server.py](server.py) | Web GUI server: monitor, waypoint sequences, demonstrations |
| [web/index.html](web/index.html) | The whole front end — three.js view plus controls |
| [kinematics.py](kinematics.py) | URDF-driven forward and inverse kinematics, numpy only |
| [arm.py](arm.py) | Connect, read, interpolated moves, servo tuning |
| [demos.py](demos.py) | Storage and playback maths for recorded demonstrations |
| [move_ee.py](move_ee.py) | CLI: move the tool to an XYZ, open/close the gripper |
| [pick_place.py](pick_place.py) | CLI: shuttle an object between hard-coded positions |
| [move_middle.py](move_middle.py) | CLI: send every joint to its calibrated midpoint |
| [scan_bus.py](scan_bus.py) | CLI: report which motor IDs and baud rates are live |
| [record_dataset.py](record_dataset.py) | CLI: record a LeRobot dataset, hand-guided or self-driven |
| [urdf/](urdf/) | Official SO-101 URDF from TheRobotStudio/SO-ARM100 |
| [requirements.txt](requirements.txt) | Pinned dependency set |
| [activate.ps1](activate.ps1) | Dot-source to activate `.venv` |

Not tracked (gitignored): `.venv/`, `waypoints.json`, `recordings/`,
`datasets/`, `outputs/`. Those are your data, not project content.

---

## The web GUI

![The SO-101 monitor GUI](ui_screenshot.png)

*The UI as of 24 September 2026: monitor mode with three saved demonstrations,
three captured waypoints, and the 3D view showing the arm skeleton, waypoint
markers with orientation triads, and the blue retract cube.*

Two modes, and the arm is stiff in only one of them:

| Mode | Torque | What happens |
| --- | --- | --- |
| **monitor** (default) | **off** | Back-drive by hand; readout and 3D view follow |
| **running** | **on** | Plays back a sequence or a demonstration; red banner across the view |

It always returns to monitor with torque off when a run ends, aborts, or fails.

The left panel shows live tool position and orientation, all six joint angles
with range bars, and a log. The 3D view draws the arm skeleton, **both gripper
jaws**, a drop line to the table, a trail of recent tool positions, and markers
for your waypoints. Drag to orbit, scroll to zoom.

### Waypoint sequences

Three things: a **list of points**, a **pair of gripper values** (one open, one
close, for the whole sequence), and a **retract position**.

1. Move the gripper by hand to where you want a point.
2. **+ capture here** records the tool XYZ *and* its orientation.
3. Set the **open** and **close** gripper values. **try** drives just the jaw so
   you can check a value against a real object, leaving the arm limp.
4. Back-drive somewhere clear and press **set retract here**.
5. Set **cycles**, then **run (torque ON)**. **stop** aborts mid-motion.

Each point is visited **twice** — once to open, once to close — retracting after
each:

```
-> retract (start)
cycle 1/1: p1
  -> p1 ... open (85) ... -> retract
  -> p1 ... close (12) .. -> retract
cycle 1/1: p2
  -> p2 ... open (85) ... -> retract
  -> p2 ... close (12) .. -> retract
```

Retract moves never touch the gripper: after closing on an object the arm has to
lift away still holding it.

Playback **refuses to start** without a retract position, and every point plus
the retract is IK-checked before torque is enabled — so an unreachable target
aborts while the arm is still limp.

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

### Teach by demonstration

Show the arm a task by hand, then have it repeat it. Separate from waypoints:
waypoints are discrete poses you choose, a demonstration is a continuous
trajectory you perform.

1. Type a name, press **● record**. Torque is released.
2. Move the arm through the whole task by hand, gripper included.
3. Press **■ stop**. Saved under `recordings/`.
4. Press **play**. Set **speed** (0.1–4x) and **cycles** first.

Recordings are captured in **joint space**, deliberately. What you physically
showed the arm *is* a joint trajectory, so replaying those angles reproduces it
exactly. Going via Cartesian would mean solving IK every frame, which can pick a
different elbow configuration, wander near a singularity, or fail outright on a
pose the demonstration passed through happily.

All six joints are captured at 30 Hz. Raw samples are stored; the moving-average
smoothing (`demos.SMOOTH_WINDOW`, ~0.23s) is applied at replay, so changing the
window cannot degrade an existing recording.

Replay eases into the demo's first pose over 2s before starting the clock, since
the arm may be nowhere near where the demonstration began.

---

## Command-line tools

```powershell
python scan_bus.py COM3                     # which motor ids/baud rates answer
python move_middle.py --dry-run             # read every joint, command nothing
python move_middle.py                       # all joints to calibrated midpoint
python move_ee.py --show                    # current joints + tool position
python move_ee.py --xyz 0.25 0.0 0.20       # move the tool there
python move_ee.py --gripper open            # open | close | half | 0-100
python pick_place.py                        # hard-coded shuttle (config at top of file)
```

Coordinates are metres in `base_link`: **+X forward** out of the base, **+Z up**,
origin at the base plate. `move_ee.py --show` is the quick way to get bearings.

---

## Recording datasets

Two Lenovo FHD webcams are mounted on this rig: one overhead looking down at the
bench, one on the wrist. `lerobot-find-cameras opencv` enumerates them as

| Index | View | Observation key |
| --- | --- | --- |
| 0 | overhead, whole bench | `observation.images.top` |
| 1 | wrist, sees both jaws | `observation.images.wrist` |

Measured on this machine, DSHOW backend, **MJPG**: each camera holds 30 fps on
its own and **both together hold 30 fps even at 1920x1080**. The default YUY2 is
uncompressed and does not fit two streams through one USB controller — set
`fourcc="MJPG"` or expect stalls. A full observation (6 joints + 2 frames) reads
at **659 Hz**, so 30 fps recording has 20x headroom.

### Why not `lerobot-record`

The stock command refuses to start:

```
A teleoperator is required for recording. Use --teleop.type=... to specify one.
```

Every arm teleoperator lerobot ships is a *separate leader device on its own
serial port*, and this rig has one arm. There is no built-in "read the follower
itself" teleoperator.

[record_dataset.py](record_dataset.py) supplies one — in fact two, chosen with
`--source`. Everything else is lerobot's own code: `record_loop`,
`LeRobotDataset`, the video encoder and the keyboard controls run unmodified.

During any episode: **right arrow** finishes it early, **left arrow** re-records
it, **ESC** stops the session. The dataset lands in `datasets/<name>/`
(gitignored) in standard LeRobot layout — a parquet of states and actions plus
one AV1 MP4 per camera — and loads straight back with `LeRobotDataset`. Add
`--push-to-hub` to upload, `--resume` to append.

### `--source hand` — pose it yourself

Torque **off**, arm limp, present position reported as the action.

```powershell
python record_dataset.py --source hand --episode-time 30 --episodes 5
```

Action and state come out identical, which is inherent to kinesthetic teaching:
with no motor holding a target, the position the arm reached *is* the command.
Between episodes there is a reset window (default 10s) that is not recorded.
**The arm is limp for the whole session** and sags if you let go of it mid-air.

### `--source waypoints` / `recording:<name>` — the arm drives itself

Torque **on**, arm playing a precomputed joint trajectory, nobody holding it.
`waypoints` runs the saved GUI pick-and-place; `recording:<name>` replays one of
the saved [demonstrations](#teach-by-demonstration).

```powershell
python record_dataset.py --source waypoints --repo-id local/wafer `
    --task "Move the wafer" --episodes 10 --jitter 0.005

python record_dataset.py --source recording:wafer-station-3 --episodes 10
```

`--dry-run` plans the trajectory and prints its per-joint range and peak speed
without moving anything. Always worth running first. `--jitter` offsets each
waypoint by a few mm per episode, so a run produces varied data rather than ten
identical passes. Episode length is derived from the trajectory, not guessed, so
`--episode-time` is ignored here.

Here `action` is the commanded setpoint and `observation.state` is where the
servo actually got to — the more useful pair to train on.

Note that none of the three saved `wafer-station` recordings move the gripper —
it sits at 0.7–1.7 throughout, clamped on the holder for the whole demonstration.
Replaying them gives arm motion but no grasping. Use `--source waypoints`, whose
jaw sweeps 45 → 16, for data that includes opening and closing.

### Making the arm actually follow the plan

Three things had to change before a scripted episode tracked its own setpoints.
Measured on the saved waypoint sequence, mean |action − state| per joint:

| | shoulder_lift mean | max |
| --- | --- | --- |
| First working version | 25.8° | 84.5° |
| After all three fixes | **2.9°** | **9.0°** |

**Seed IK from the previous pose.** Solving `retract` from a fixed seed let IK
return a *different elbow configuration* than the waypoints, so the arm swung
between two branches of the same solution — 118° of shoulder_lift per 1.5s move.
`server.goto_pose` never hits this because it seeds from wherever the arm is.

**Turn `max_relative_target` off.** Counter-intuitively the safety cap made
tracking twice as bad (12.6° → 25.8° mean): clamping the goal to present ±12°
stops the servo ever being handed a target far enough ahead to catch up. Worse,
`record_loop` stores the action the teleoperator *requested*, not the clamped one
`send_action` sent ([lerobot_record.py:331](.venv/Lib/site-packages/lerobot/scripts/lerobot_record.py),
with their own TODO beside it) — so a clamped frame records a command the arm
never received. `check_plan_speed` guards the real risk instead, before the arm
moves.

**Raise `Acceleration` to 254 for playback.** arm.py's 24 is tuned to stop the
arm jittering on slow GUI moves; here the goal is to hit the setpoint. Mean
error by acceleration: 24 → 4.34°, 64 → 4.34°, 128 → 2.98°, 254 → 2.62°.

A plan faster than `--max-speed` (default 60 deg/s) is stretched in time
automatically. That number is where the curve flattens:

| peak commanded speed | 136 | 91 | 68 | 45 deg/s |
| --- | --- | --- | --- | --- |
| mean error, all joints | 5.30° | 2.97° | 2.10° | 1.38° |

What is left below that is **static droop** — shoulder_lift carrying the arm's
weight against a proportional band of P=16 — not lag, so slowing further buys
almost nothing and costs episode time. Raise `--max-speed` for shorter episodes
if you are willing to trade tracking for throughput.

**The arm moves on its own with nobody holding it.** Run `--dry-run` first,
check the workspace is clear, and note that the session ends with torque off, so
the arm sags from wherever the trajectory left it.

---

## How it works

### Kinematics

[kinematics.py](kinematics.py) drives the official URDF
([urdf/so101_new_calib.urdf](urdf/)). Every revolute joint in that file has local
axis `(0,0,1)`, so each link transform is `Translate(xyz) @ RPY(rpy) @ RotZ(q)`
and forward kinematics is a plain product of 4x4s.

Two solvers, both damped least squares with random restarts:

- **Position IK** (`ik`) — 3 constraints on 5 joints. Validated over 300 random
  reachable targets: 291/300 converge, **max error 0.34 mm** (the rest hit the
  iteration cap, still sub-millimetre).
- **Pose IK** (`ik_pose`) — position *and* orientation, 6x5 Jacobian. Five joints
  cannot hit an arbitrary 6-DOF pose, but poses *captured from the arm* always
  can, which is exactly how waypoints are made. 300/300 converge, **max 0.10 mm
  and 0.52 deg**. Position is weighted above orientation so it wins where a pose
  is not exactly attainable.

The arm has 5 positioning joints but position is only 3 constraints, so solutions
form a 2-parameter family. Both solvers seed from the current pose and apply a
nullspace pull toward it, so successive calls stay near each other instead of
picking wildly different elbow configurations.

### Why not lerobot's solver

`lerobot.model.kinematics.RobotKinematics` needs
[`placo`](https://pypi.org/project/placo/), which does not work on Windows. This
was checked properly, not assumed:

| Route | Result |
| --- | --- |
| `pip install placo` | No Windows wheels exist — PyPI has only `macosx_*` and `manylinux_*`. Source build fails on `eiquadprog`. |
| conda-forge | Native `win-64` builds **do** exist and install cleanly with pinocchio/eigenpy/eiquadprog. |
| ...but importing it | `ImportError: DLL load failed while importing placo: The specified procedure could not be found.` |
| Versions 0.9.17–0.9.20 | All four fail identically. |
| In a pristine env (only python + placo) | Still fails — so it is the conda-forge Windows build, not a local conflict. |

If you ever want real placo, WSL is the route: the manylinux wheels install with
plain pip. You would then need `usbipd` to get COM3 across the WSL boundary.

> [!WARNING]
> Do not `conda install placo` into `.venv`. It pulls conda's MKL-backed numpy
> over the pip one, and MKL then collides with the copy torch ships: every
> `np.linalg.solve` **hard-crashes the interpreter** with no traceback. Recover
> with `.venv\python.exe -m pip install --force-reinstall --no-deps numpy==2.2.6`.

### Frame conventions

`shoulder_pan` carries `rpy="3.14159 0 -3.14159"` in the URDF, a 180-degree flip.
**Positive pan moves the tool toward −Y.** FK and IK agree with each other, so
the maths is consistent — just do not expect pan sign to match a naive
right-handed reading of the base frame.

`gripper` is not part of the kinematic chain. It only opens and closes the jaw,
so it never affects where the tool point is. lerobot's 0–100 maps linearly onto
the URDF joint limits; measured, the jaw tip travels 25 mm from the tool point at
0 to 120 mm at 100, confirming **0 = closed, 100 = open**.

### Server architecture

No dependencies beyond the standard library plus vendored three.js.

- **Transport** is Server-Sent Events for the live stream (Python → browser) and
  plain `POST /api` for commands the other way.
- **One thread owns the robot.** A serial port is exclusive, so the poller holds
  the connection and HTTP handlers hand it commands through a `queue`. Playback
  runs *inside* that thread, publishing state as it interpolates, which is why
  the 3D view animates during a run.
- **`abort` bypasses the queue** and sets an `Event` directly — queued behind a
  running sequence, a stop button would be useless.
- **Forward kinematics stays in Python.** The browser receives already-computed
  3D points for the skeleton and both jaws and just draws them. Nothing about the
  URDF is reimplemented in JavaScript.
- The camera is set `up = (0,0,1)`: the URDF is Z-up, three.js defaults to Y-up.

---

## Measured behaviour

### Jitter

The arm is visibly rough during motion. Measured by sweeping one joint and taking
the RMS of the high-frequency part of its velocity on a fixed 100 Hz analysis
grid (resampling first, so encoder quantisation at short intervals cannot
masquerade as roughness).

**Raising the command rate does not help.** This was the obvious hypothesis — 30
Hz goals to a position-mode servo ought to stair-step — and it is wrong:

| Command rate | Goal step | Roughness (deg/s) |
| --- | --- | --- |
| 30 Hz | 0.67 deg | 2.2 |
| 60 Hz | 0.33 deg | 2.5 |
| 120 Hz | 0.17 deg | 2.4 |
| 200 Hz | 0.10 deg | 3.0 |

Flat, slightly worse at the top. The bus could take it — 569 Hz for commands, 307
Hz for a full read-and-command loop — the rate simply is not the bottleneck.
**At rest the arm is rock steady**: 0.000 deg position noise holding a fixed goal,
so this is not control-loop hunting either.

**Servo acceleration is the one real software lever.** lerobot writes
`Acceleration = Maximum_Acceleration = 254`, the maximum, so every goal is chased
flat out:

| Acceleration | Roughness (deg/s) | Lag (deg) |
| --- | --- | --- |
| 254 (lerobot default) | 3.2 | 1.23 |
| 96 | 2.6 | 1.05 |
| 24 | 2.4 | 0.88 |
| 12 | 2.2 | 0.75 |

Backing it off is ~25% smoother *and* tracks better. `arm.tune_servos()` applies
`Acceleration = 24`, `D = 0` on connect — it must run *after* `robot.connect()`,
since lerobot's own `configure()` resets acceleration to maximum every time.

**What is left is mechanical.** Rate does nothing, gains do nothing, it holds
perfectly still at rest, and the best tuning still leaves ~2.2 deg/s. That
signature — rough only while moving, silent when stopped — is stick-slip and lash
in the 1/345 plastic gearboxes, which no amount of software will remove.

One thing that genuinely helps: **approach every point from the same direction**.
Backlash is hysteresis, so consistent approach makes the error repeatable even
though it does not make it smaller. The retract-then-approach shape already does
this.

### Replay lookahead

Position-mode servos trail a moving target by a roughly fixed time, so demo
replay reads the trajectory slightly ahead to cancel it. Measured on a 6s sweep,
mean error across all six joints:

| Lookahead | Mean lag | Worst |
| --- | --- | --- |
| 0.00s | 2.09 deg | 9.29 |
| 0.05s | 1.52 deg | 7.19 |
| 0.10s | 1.01 deg | 5.57 |
| 0.15s | 0.70 deg | 5.31 |

`server.REPLAY_LOOKAHEAD` defaults to **0.10s** — a measured 2x improvement,
staying short of the best value tested since a hand demonstration has sharper
reversals than the smooth sweep this was tuned on.

### How far to trust the model

The URDF describes an ideal SO-101. Whether it matches *this* arm depends on the
calibrated zero lining up with the URDF's zero pose. Comparing the calibrated
sweep against the URDF limits:

| Joint | Calibrated span | URDF span | |
| --- | --- | --- | --- |
| `shoulder_pan` | 210.5 deg | 220.0 deg | close |
| `shoulder_lift` | 202.5 deg | 200.0 deg | close |
| `elbow_flex` | 193.8 deg | 193.7 deg | near exact |
| `wrist_flex` | 206.5 deg | 190.0 deg | over-swept ~16 deg |
| `wrist_roll` | 360.0 deg | 320.0 deg | **swept a full turn** |

The three joints that set tool position agree closely, which is why Cartesian
targets land where they should — measured tracking error moving to
`(0.25, 0, 0.20)` was **4 mm**, mostly gravity droop rather than model error.

Two caveats:

- **`wrist_flex`** was swept ~16 deg wider than the URDF allows, so its zero may
  be off by up to ~8 deg.
- **`wrist_roll`** was swept a full 360 deg, so its midpoint — and therefore its
  zero — is essentially arbitrary.

Neither affects tool *position*. Both affect tool *orientation*, so re-calibrate
the wrists against a physical reference before relying on orientation matching.

---

## Safety behaviour

Shared in [arm.py](arm.py) so every entry point inherits it:

- On connect, the present pose is immediately re-asserted as the goal, so a stale
  `Goal_Position` left in the servos cannot yank the arm.
- All motion is interpolated with smoothstep easing, never stepped.
- `max_relative_target` (default 12 deg) caps how far a goal may lead the measured
  position. The **gripper is exempt** — it is geared slowly and a cap tight enough
  for the arm throttles the jaw on every step.
- Every motion commands all six motors, holding the ones it is not moving.
- Targets below `z = 0.02 m` are refused by `move_ee.py` unless `--allow-low`.
- IK residual over 5 mm is treated as unreachable and refuses to move.
- Joint-space interpolation is deliberate: a straight-line Cartesian path can
  cross a singularity, so the tool traces an arc instead.

> [!NOTE]
> A run ends by disabling torque so you can back-drive again. The arm is usually
> up at the retract point when that happens, so it will **drop** from there. Put
> the retract somewhere a fall is harmless, or catch it.

---

## Setting up from scratch

Already done on this machine — this section is for a rebuild or a second arm.

### Environment

LeRobot targets **Python 3.12**. Conda's base interpreter here is 3.14, ahead of
what the torch wheels support, so create the env with an explicit 3.12:

```powershell
conda create -y -p .\.venv python=3.12
.\.venv\python.exe -m pip install -r requirements.txt
```

`.venv` is a conda environment at a path prefix, not a `python -m venv` venv, so
it has no `Scripts\activate.ps1` of its own — use [activate.ps1](activate.ps1),
which assumes conda at `%USERPROFILE%\anaconda3`.

Verified working set: Python 3.12.13, lerobot 0.6.1, feetech-servo-sdk 1.0.0,
pyserial 3.5, torch 2.11.0, numpy 2.2.6.

### Hardware bring-up

1. **Find the port** — `lerobot-find-port` lists ports and asks you to unplug the
   arm. With one adapter present it is moot; this machine uses COM3
   (`USB-Enhanced-SERIAL CH343`).

2. **Check whether the motors are already configured** — `python scan_bus.py COM3`.
   Motor IDs live in each servo's EEPROM, so a second-hand arm may already be
   done. IDs 1–6 at 1000000 baud means skip to calibration.

3. **Set motor IDs and baud rates** (only if needed):

   ```powershell
   lerobot-setup-motors --robot.type=so101_follower --robot.port=COM3
   ```

   Prompts for one motor at a time in reverse joint order, `gripper` first.
   **Exactly one motor may be connected at each prompt** — every servo ships with
   ID 1 and the bus is half-duplex, so two motors sharing an address collide. Do
   not build the chain up incrementally: lerobot picks whichever motor answers
   first and only validates the *model* number, so a partially-linked chain can
   silently overwrite an already-configured motor's ID.

4. **Calibrate**:

   ```powershell
   lerobot-calibrate --robot.type=so101_follower --robot.port=COM3 --robot.id=arm0
   ```

   Centre every joint, press Enter, then move each joint through its full range.
   Calibration is **not** stored in the motors — it is a JSON file on this PC at
   `HF_LEROBOT_CALIBRATION/robots/so_follower/<id>.json`.

### Motor map

Identical for leader and follower: `1 shoulder_pan`, `2 shoulder_lift`,
`3 elbow_flex`, `4 wrist_flex`, `5 wrist_roll`, `6 gripper`.

---

## Troubleshooting

**Nothing responds, but the COM port exists.** The port comes from the USB-serial
chip on the controller board, which is USB-powered — its presence says nothing
about the servos. Check the **12V supply** first, then the 3-pin cable, then the
Waveshare B-channel jumpers.

**`Could not connect on port 'COM3'` / `ConnectionError`.** The port is already
held by another process — usually an earlier `server.py` still running. Serial
ports are exclusive on Windows. lerobot's message tells you to run
`lerobot-find-port`, which sends you after the wrong problem. Find the holder:

```powershell
Get-CimInstance Win32_Process -Filter "Name LIKE '%python%'" |
    Select-Object ProcessId, CommandLine
Stop-Process -Id <pid> -Force
```

`pkill` from Git Bash does **not** reliably kill these — use `Stop-Process`.

**The GUI loads but the arm reads as not connected — and the port is free.** A
`server.py` whose startup retries were all used up keeps serving HTTP with no
robot behind it. The browser still finds a page at <http://localhost:8000>, so it
looks like the arm is undetected, but the serial port was released and the entry
above sends you hunting for a holder that no longer exists. Confirm the motors
are fine, then kill the dead server by the port it still owns:

```powershell
.\.venv\python.exe scan_bus.py          # all six ids should answer
Get-NetTCPConnection -LocalPort 8000 -State Listen | Select-Object OwningProcess
Stop-Process -Id <pid> -Force
```

Restarting `server.py` then connects on the first attempt. Look for the startup
line `connected -- torque OFF, arm is back-drivable`; without it, the server is
in this dead state no matter what the page shows.

**`connect attempt 1/3 failed (bus glitch)`.** The Feetech bus drops the odd
status packet, most often right after a process was killed mid-transaction.
Startup retries 3 times, 3s apart, and the poll loop absorbs transient failures
during monitoring — only 25 consecutive failures are fatal. If it happens on
*every* attempt rather than occasionally, suspect the 12V supply or a marginal
3-pin connection.

**`ImportError: DLL load failed while importing _imaging`.** Pillow was installed
by **conda**, not pip, and the conda build does not load in this env — the same
class of breakage as the numpy/MKL one above. Nothing in the repo imported it
until the dataset work pulled in torchvision, so it sat there unnoticed. Check
the installer and replace it with pip's build:

```powershell
type .venv\Lib\site-packages\pillow-*.dist-info\INSTALLER   # "conda" = broken
.\.venv\python.exe -m pip install --force-reinstall --no-deps pillow
```

Leftovers from the abandoned placo attempt (`casadi`, `meshcat`, `pyngrok`) came
in the same way. They are inert, but if an import fails oddly, check `INSTALLER`
first.

**`'torchcodec' is installed but cannot be loaded ... Falling back to 'pyav'`.**
Harmless. `libtorchcodec_core4.dll` will not load against this torch build, and
pyav decodes the dataset videos correctly — only slower. Recording does not use
it at all; it appears when you *read* a dataset back.

**Gripper will not hold an object.** The follower caps gripper torque at 50% and
`Overload_Torque` at 25% to avoid burning the servo out. An
`[RxPacketError] Overload error!` on id 6 means it stalled — back the target off
rather than raising the limits.

**Only ID 1 shows up with the whole chain connected.** Expected for unconfigured
motors — they are all colliding on address 1. Run `lerobot-setup-motors`.

**`lerobot-find-port` crashes with an EOF error.** It is interactive and needs a
real terminal; it cannot run piped or from a script.

---

## Camera-guided pick and place

Locating something the arm has never been told about means turning a camera
pixel into a reachable position. [calibrate_camera.py](calibrate_camera.py)
fits that mapping, and [pick_box.py](pick_box.py) uses it.

A full run, both cameras, recorded live:
[**both views stacked**](media/pick_and_place_stacked.mp4) — 50 s, picking a
transparent box off the bench and setting it on the red mat, overhead on top and
wrist below. The separate
[overhead](media/pick_and_place_top.mp4) and
[wrist](media/pick_and_place_wrist.mp4) clips are there too.

```powershell
python calibrate_camera.py                  # ~8 min of arm time, writes camera_calib.json
python calibrate_camera.py --check          # residuals of the saved fit, moves nothing
python calibrate_camera.py --refit          # refit the maths from saved raw data
python probe.py                             # where is the jaw, in pixels and in metres
python pick_box.py --dry-run                # plan the pick, move nothing
python pick_box.py --approach-only          # stop with the jaws around the object
python pick_box.py
python pick_box.py --record media           # ...and film it from both cameras
```

`jog.py` steps the arm to one Cartesian target and photographs both cameras —
the tool for working a new task out by hand.

### Finding the gripper without eyeballing it

The calibration needs to know where the tool is in the image. Rather than
locate a gripper by colour or shape, it **opens and closes the jaw and subtracts
the two frames**: the arm, the bench and the lighting are identical in both, so
the only thing that survives is the jaw. Three cycles per point, median-filtered
— a single difference cannot tell the jaw from anything else that moved in that
half second, and on the first run one point locked onto activity at the far side
of the bench and came out 400mm wrong.

### Why a projective camera and not a homography

A homography maps pixels to one plane. Fitted to the jaw sweeping at gripper
height it describes *that* height, and an object lying on the bench is ~50mm
below it — over a centimetre of parallax error. Sweeping at two heights makes
the points non-coplanar, which determines a full 3x4 projection, and a pixel can
then be back-projected onto the table specifically. Measured: **1.27 px
reprojection, 1.18 mm back-projection**, against 3.6 mm for the single-plane fit.

The world point is the midpoint of the jaw bar, chosen by leave-one-out
cross-validation over four candidates. The tool point had the *lowest* in-sample
residual and the worst held-out error (22 mm) — it was overfitting five points.

### Aim the grip centre, not the tool point

`fk_position` returns the tip of the **fixed** jaw, which is one side of the
gripper. Driving it to an object leaves the object against one jaw with the
other 35mm away. `pick_box.solve_for_grip` iterates the tool-point command until
the *midpoint between the jaw tips* lands on the target instead.

An object also does not end up exactly at the grip centre — it settles toward
the fixed jaw as the other closes. Measured 28mm for the box here, so the place
target is offset by the same amount.

### Recording it

[video.py](video.py) films both cameras to H.264 while something else drives the
arm. Two details it gets wrong if written naively:

- **Sample on an absolute wall-clock schedule**, not a fixed sleep between
  grabs. Left to drift the two loops ran at 51 and 32 fps over the same five
  seconds, so the clips played at different speeds and neither matched what the
  arm did. If a frame is missed the counter advances to real time rather than
  emitting a burst of catch-up frames.
- **Open both cameras before starting either thread.** Connecting a camera takes
  seconds, so opening and starting each in turn left the first clip running
  three seconds ahead of the second.

A recording owns both cameras while it runs -- DirectShow will not hand the same
device to a second process -- so stills come from `Recorder.snapshot()` rather
than from `jog.py`. 640x480 at CRF 26 keeps a 50 s run to ~3.5 MB per camera.

The two clips are written independently and end a fraction of a second apart,
because `stop()` closes them in turn. To put them in one frame, trim both to the
shorter and stack. They start together, so trimming from the front keeps them in
step:

```powershell
winget install --id Gyan.FFmpeg -e        # ffmpeg is not otherwise needed

ffmpeg -t 49.766667 -i media/pick_and_place_top.mp4 `
       -t 49.766667 -i media/pick_and_place_wrist.mp4 `
       -filter_complex "[0:v][1:v]vstack=inputs=2[v]" -map "[v]" `
       -c:v libx264 -crf 26 -preset medium -pix_fmt yuv420p `
       media/pick_and_place_stacked.mp4
```

`ffprobe -show_entries format=duration` on each input gives the length to trim
to. The result is 640x960 and 5.5 MB.

### Measured behaviour

- The arm settles **~10-15mm low** under gravity. Each move re-measures and
  re-aims, under-relaxed at 0.6 gain — at full gain it oscillates, because the
  servos do not respond 1:1 once backlash is taken up.
- Accuracy falls off to the far left of the workspace: placements landed within
  2mm near the middle of the bench and ~10mm at full left reach.
