# Inference on MCXN236

The [OnePolicy](https://github.com/Xygarde69/OnePolicy) quadruped locomotion
policy running on an **NXP FRDM-MCXN236** board (Arm Cortex-M33 @ 150 MHz,
1 MB flash), walking a real-time simulation of **ANYmal C**. Tilt the board
to steer the robot.

This repo is standalone: the firmware, the policy, the simulator and the
ANYmal C model are all here.

```
            PC: test/run.py (MuJoCo, real time)                     FRDM-MCXN236: firmware/app
 ┌──────────────────────────────────────────────┐  USB-C (J10)  ┌──────────────────────────────────┐
 │ ANYmal C (test/assets/anymal_c)              │               │ accelerometer tilt ─► command    │
 │   ─► 51 observations ────────────────────────┼─────────────► │ normalize ─► MLP 51-512-256-128- │
 │ 12 joint targets ─► DC-motor PD ─► physics   │  921600 baud  │ 12 (ELU), weights in flash       │
 │   ◄── actions + tilt command ────────────────┼◄───────────── │ (firmware/policy/OnePolicy.onnx) │
 └──────────────────────────────────────────────┘  every 20 ms  └──────────────────────────────────┘
```

Every 20 ms the PC sends the robot's observations to the board. The board runs
the policy and replies with the 12 joint actions plus the forward/sideways
velocity command it reads from its tilt. Measured: 11.3 ms per inference on the
board, 15.2 ms round trip, and the board's actions match ONNX Runtime on the PC
to about 1e-6.

## Layout

| Folder | What |
|---|---|
| **`firmware/`** | Everything for the board |
| `firmware/app/` | The firmware: serial protocol (`main.c`), the network (`policy_net.c`), tilt command (`tilt.c`), settings (`Kconfig`) |
| `firmware/policy/` | The policy: `OnePolicy.onnx`, `export.py`, and the generated `policy_model.[ch]` (the weights as C arrays) |
| `firmware/accel/` | Accelerometer demo (prints X/Y/Z in mg) |
| `firmware/docs/` | MCU datasheet and board user manual |
| **`test/`** | Watch the policy run in real time |
| `test/run.py` | The simulator: viewer, control loop, board link |
| `test/assets/anymal_c/` | ANYmal C: scene `anymal_c.xml`, meshes and textures in `assets/`, controller `robot.json` |
| `test/anymal.py`, `board.py`, `policy.py`, `tilt.py` | Robot and observations, serial link, PC policy (ONNX), tilt checker |

## Run it from scratch

Tested on Arch Linux. You need git, [uv](https://docs.astral.sh/uv/), and the
board connected by its **MCU-Link USB-C port (J10)**. That one cable does
flashing and the serial link. No GPU needed.

### 1. Clone and install

```bash
git clone git@github.com:Xygarde69/Inference_on_MCX236.git Inference
cd Inference
sudo pacman -S --needed arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-gdb cmake ninja
uv sync                                    # Python: west, pyocd, mujoco, onnxruntime, ...
```

### 2. Fetch the NXP SDK (once, ~2 GB)

West and pyOCD commands run from inside `firmware/`. Outside it, uv can't
find the west workspace.

```bash
cd firmware
uv run west init -l manifest               # make firmware/ the west workspace (manifest/west.yml)
uv run west update --narrow -o=--depth=1   # MCUXpresso SDK into firmware/mcuxsdk
uv run pyocd pack install MCXN236VDF       # flash algorithm for pyOCD
uv run pyocd list                          # should show "MCU-LINK FRDM-MCXN236"
```

If opening the serial port later fails with "Permission denied", add yourself
to the group that owns it (`ls -l /dev/ttyACM*`; `uucp` on Arch, `dialout` on
Debian/Ubuntu): `sudo usermod -aG uucp $USER`, then log in again.

### 3. Build and flash the firmware

```bash
cd firmware                                # if not there already
source env.sh                              # ARMGCC_DIR=/usr
uv run west build -p always app -b frdmmcxn236 -d build --config release
uv run pyocd flash build/policy.elf
```

The policy's weights are already in `firmware/policy/`. To flash a different
policy, put its ONNX there (an rsl_rl actor exported the way OnePolicy does) and
export it before building:

```bash
uv run policy/export.py policy/MyPolicy.onnx
```

### 4. Check the tilt directions

```bash
cd ..                                      # back to the repo root
uv run test/tilt.py
```

Hold the board still when it starts: that pose means "stand still". Then dip
each edge and watch `forward` and `left`. If one comes out reversed or they are
swapped, see [Tilt settings](#tilt-settings).

### 5. Walk ANYmal C

```bash
uv run test/run.py
```

A MuJoCo window opens with ANYmal C. Hold the board in its neutral pose while
it starts, then tilt it forward/back to walk forward/back and left/right to
walk sideways. **Left / Right arrow** keys turn. Every 5 s the terminal prints
the command, how fast the robot actually walked, the board's inference time and
round trip, and the difference from the PC's ONNX policy. The robot is reset if
it falls.

Other ways to run it:

```bash
uv run test/run.py --policy pc                       # no board: ONNX on the PC, Up/Down arrows walk
uv run test/run.py --no-tilt --command 0.5 0 0       # policy on the board, fixed command, keys steer
uv run test/run.py --headless --duration 20          # no window, just the stats
uv run test/run.py --help
```

Commands are in Go2 units (m/s, m/s, rad/s), as OnePolicy uses them. The
simulator scales them to ANYmal C's size (×1.22 for speed). The ranges are
forward −0.5 … 1.0, sideways ±0.5, yaw ±0.5.

## Tilt settings

Set in `firmware/app/Kconfig`, then rebuild and reflash:

| Setting | Default | |
|---|---|---|
| `TILT_FORWARD_AXIS` | 0 | Accelerometer axis pointing forward: 0 = X, 1 = Y (the other is left) |
| `TILT_FORWARD_SIGN` / `TILT_LEFT_SIGN` | 1 | Set −1 to reverse a direction |
| `TILT_DEADZONE_DEG` | 5 | Tilt below this does nothing |
| `TILT_FULL_DEG` | 25 | Tilt for full speed |
| `TILT_MAX_FORWARD_MMS` / `_BACKWARD_` / `_LATERAL_` | 1000 / 500 / 500 | Full-tilt speeds in Go2 mm/s |
| `TILT_SMOOTHING_PCT` | 20 | Low-pass: % of the way to the new command each 20 ms step |

The tilt is measured as the gravity direction relative to the pose at start,
so the board doesn't have to lie flat. The board's manual doesn't show which
way the accelerometer axes point: the defaults assume +X forward and +Y left.

## How the simulator matches OnePolicy

`test/anymal.py` rebuilds, in plain MuJoCo, the environment OnePolicy trained
and plays in, so the policy sees exactly what it expects:

- **Robot:** `anymal_c.xml` is ANYmal C rewritten into OnePolicy's convention (joint names and order, axes), with the
  contact softness, armature and motor limits scaled to its size, plus OnePolicy's physics options (5 ms steps,
  implicitfast).
- **Observations (51):** IMU angular velocity, gravity in the base frame, velocity command, gait clock,
  joint positions and velocities, knee direction, and the last action.
- **Action:** joint targets = default pose + 0.25 × action. They are tracked by PD-controlled DC motors with a
  torque-speed limit, updated every 5 ms physics step (4 per policy step).

Checked against OnePolicy's own simulation from the same states: the
observations match to 1e-6, a 20 ms step lands within 7e-6 rad, and at a 0.5
m/s command it walks 0.49–0.50 m/s, as in OnePolicy.

`test/tools/extract_anymal_c.py` generated `test/assets/anymal_c` from
OnePolicy. It is only needed to regenerate the assets (it needs a OnePolicy
checkout); nothing else uses OnePolicy.

## Serial protocol

Little-endian frames. Every frame ends with the XOR of its command and payload bytes.

| Request (`0xA5 cmd payload xor`) | Reply (`0x5A cmd payload xor`) |
|---|---|
| `I` | u32 weights CRC32, u16 n_obs, u16 n_act, u32 core Hz, u8 accelerometer ok |
| `P` + 51 × f32 observations | 12 × f32 actions, f32 forward, f32 left (Go2 m/s), u32 inference cycles |
| `C` | f32 forward, f32 left (tilt command, no inference) |
| `Z` | u8 ok: the current pose becomes "stand still" |
| (bad frame) | `E` + u8 code: 1 checksum, 2 unknown command, 3 UART error |

## Troubleshooting

- **`Failed to spawn: west`** or **`not in a west workspace`**: run west and pyOCD from inside `firmware/`.
- **`No reply from the board`**: the firmware isn't flashed, or a terminal (miniterm, screen) has the port open.
- **`Board actions differ from ... OnePolicy.onnx`**: the board holds a different policy. Build and flash again.
- **`/dev/ttyACM0` missing**: the board can come back as `ttyACM1` after flashing. The scripts find it by its
  `/dev/serial/by-id` name automatically; `--port` overrides.
- **The robot walks the wrong way when tilted**: flip `TILT_*_SIGN` or `TILT_FORWARD_AXIS`.

## Limits

- This OnePolicy checkpoint walks forward and turns well. Its sideways and backward walking is weak (0.3 m/s
  commanded gives about 0.06 sideways and 0.09 backward), the same as in OnePolicy itself.
- Only the flat-ground policy fits. Its float32 weights take 752 kB of the 1 MB flash; the rough-terrain
  `OnePolicy_r` (~1.1 MB) would need smaller fp16/int8 weights.
- Flat ground only, nominal robot (no randomised friction or mass).

ANYmal C model: © ANYbotics AG, BSD-3-Clause (`test/assets/anymal_c/LICENSE`), via MuJoCo Menagerie and OnePolicy.
Board and SDK notes for development are in [CLAUDE.md](CLAUDE.md).
