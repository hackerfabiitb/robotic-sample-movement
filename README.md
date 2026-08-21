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

Opens <http://localhost:8000> with a live readout of the tool position and joint
angles, plus a three.js stick figure of the arm. **Torque is disabled while it
runs**, so you can back-drive the arm by hand and watch the display follow.

It is strictly read-only -- nothing in it ever commands a position, which is what
makes it safe to leave running while you work.

```
python server.py --http-port 8080 --rate 60 --no-browser
```

### How it fits together

No new dependencies -- it is standard library plus a vendored copy of three.js.

- **Transport is Server-Sent Events** over `http.server`. Data only flows one
  way (Python to browser), so SSE does the job without a websocket library.
- **Forward kinematics stays in Python.** The server sends the browser the
  already-computed 3D position of every joint origin via
  `SO101Kinematics.fk_frames()`, and the page just draws lines between them.
  Nothing about the URDF is reimplemented in JavaScript.
- **three.js is vendored** at [web/vendor/three.module.js](web/vendor/) rather
  than pulled from a CDN, so the GUI works at a bench with no internet. It is
  ~1.3 MB, which is the bulk of this repo.
- Orbit control is a ~20-line pointer handler rather than another vendored file.
- The camera is set `up = (0,0,1)`: the URDF is Z-up and three.js defaults to Y-up.

Measured: the arm reads at **~650 Hz** over the serial bus, so the 30 Hz default
stream rate has enormous headroom. `--rate` raises it if you want.

### Using it to set pick-and-place waypoints

This is the easy way to fill in `POSITIONS` in [pick_place.py](pick_place.py):
start the server, physically move the gripper to where you want a waypoint, and
read X/Y/Z straight off the panel.

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
| [server.py](server.py) | Live web GUI server (read-only, torque off) |
| [web/index.html](web/index.html) | three.js front end for the monitor |
