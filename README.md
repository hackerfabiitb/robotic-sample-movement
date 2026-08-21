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

## Troubleshooting

**Nothing responds to `scan_bus.py`, but the COM port exists.** The port comes
from the USB-serial chip on the controller board, which is USB-powered — its
presence says nothing about the servos. Check the 12V supply first, then the
3-pin cable, then the Waveshare B-channel jumpers.

**`lerobot-find-port` crashes with an EOF error.** It's interactive and needs a
real terminal; it can't run piped or from a script.

**Only ID 1 shows up with the whole chain connected.** Expected for unconfigured
motors — they're all colliding on address 1. Proceed with step 3.

**Torch/CUDA.** On Windows pip installs the default wheel. LeRobot falls back to
`pyav` for video decoding, so a separate ffmpeg install isn't required.

## Files

| File | Purpose |
| --- | --- |
| [requirements.txt](requirements.txt) | Pinned dependency set |
| [activate.ps1](activate.ps1) | Dot-source to activate `.venv` |
| [scan_bus.py](scan_bus.py) | Report which motor IDs/baud rates are live |
