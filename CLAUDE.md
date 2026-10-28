# CLAUDE.md

Standalone repo: OnePolicy's quadruped policy running on the **NXP FRDM-MCXN236** (MCU: MCXN236VDFT, Arm Cortex-M33 @ 150 MHz,
1 MB flash, 352 KB SRAM, 184VFBGA), driving a real-time MuJoCo simulation of ANYmal C on the PC. Nothing outside this repo
is needed. `test/tools/extract_anymal_c.py` is the only file that touches OnePolicy, and only to regenerate the assets.

The firmware is built on the **MCUXpresso SDK v26.09.00** with arm-none-eabi-gcc, CMake and Ninja, and flashed with
**pyOCD** through the board's on-board MCU-Link (CMSIS-DAP).

- MCU datasheet: `firmware/docs/MCXN23x.pdf` (Rev. 3, 21 Jan 2025).
- Board user manual: `firmware/docs/UM12041-FRDM-MCXN236.pdf` (UM12041 Rev. 3.0): header pinouts, on-board peripherals, jumper/resistor options.

Cite a table or section from these when a hardware decision depends on it.

## Layout

```
firmware/                 everything that runs on the board; the west workspace (.west/) lives here
  app/                    the policy firmware: serial loop (main.c), MLP (policy_net.c), tilt command (tilt.c)
    Kconfig               BOARD_DEBUG_UART_BAUDRATE (921600) and the TILT_* mapping
    board/                app.h (UART, I2C pins), hardware_init.c (clocks, I2C bus release), pin_mux.[ch]
  policy/                 OnePolicy.onnx, export.py, and the generated policy_model.[ch] (weights as const arrays)
  accel/                  accelerometer demo: print X/Y/Z in mg (pinmux and bus recovery from demo_apps/bubble)
  manifest/west.yml       pulls only the SDK repos this board needs (pinned to v26.09.00)
  mcuxsdk/                SDK checkout (git-ignored; `uv run west update` from firmware/)
  docs/                   MCU datasheet + board user manual
  env.sh                  ARMGCC_DIR for the SDK's CMake toolchain file
  pyocd.yaml              pins the pyOCD target (mcxn236vdf); pyOCD reads it from the cwd, so run it from firmware/
test/                     the PC side: real-time ANYmal C simulation with the policy on the board
  run.py                  viewer + control loop (--policy board|pc, --headless)
  anymal.py               plain-MuJoCo ANYmal C: OnePolicy's 51 observations and DC-motor PD action
  board.py                serial protocol to firmware/app (termios, no pyserial)
  policy.py               ONNX Runtime policy: runs without the board, and checks the board's actions
  tilt.py                 live tilt command, to check the axes
  assets/anymal_c/        anymal_c.xml (scene), assets/ (meshes, textures), robot.json (controller), LICENSE
  tools/extract_anymal_c.py   regenerates assets/anymal_c from a OnePolicy checkout (needs OnePolicy's venv)
pyproject.toml, uv.lock   one uv project for both: west, pyocd, SDK build deps, mujoco 3.5.0, onnx, onnxruntime
```

Don't edit anything under `firmware/mcuxsdk/`. Copy what you need into the app and change it there.

## Python: uv only

Manage every Python dependency through uv: `uv add <pkg>`, `uv sync`, and `uv run <tool>`.
Never use pip, `uv pip`, or `.venv/bin/...` directly. `uv run` works from any subdirectory (it finds the root project),
but `west` and `pyocd` must run from `firmware/` (west topdir, `pyocd.yaml`).

## Setup (fresh clone)

```bash
sudo pacman -S --needed arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-gdb cmake ninja
uv sync
cd firmware
uv run west init -l manifest               # .west/ is git-ignored: recreate the workspace (topdir = firmware/)
uv run west update --narrow -o=--depth=1   # fetch the SDK (~2 GB) into firmware/mcuxsdk
uv run pyocd pack install MCXN236VDF       # CMSIS pack (flash algorithm) for pyOCD
source env.sh                              # ARMGCC_DIR=/usr
```

## Build / flash / run

```bash
cd firmware && source env.sh
uv run policy/export.py                     # only after changing policy/OnePolicy.onnx
uv run west build -p always app -b frdmmcxn236 -d build --config release
uv run pyocd flash build/policy.elf
cd .. && uv run test/run.py                 # or --policy pc without a board, --headless for logs only
```

Other firmware commands (from `firmware/`):

```bash
uv run west build -p always accel -b frdmmcxn236 -d build_accel --config debug
uv run pyocd gdbserver       # then: arm-none-eabi-gdb build/policy.elf -ex "target remote :3333"
```

Connect only the **MCU-Link USB-C port (J10)**. It provides SWD flashing and debugging plus the serial port, so no
external probe is needed. `uv run pyocd list` should show "MCU-LINK FRDM-MCXN236". The board reports itself as
`mcxn236vdft`, which pyOCD doesn't know, so `pyocd.yaml` overrides it to `mcxn236vdf`. The SDK's west runners for this
board only support LinkServer and J-Link, so `west flash` isn't wired up. Call pyOCD directly.

## SDK conventions (new MCUXpresso build system)

- Kconfig selects which drivers are built, e.g. `CONFIG_MCUX_COMPONENT_driver.fxls8974cf=y` in `prj.conf`. On this chip `driver.i2c` doesn't apply (the I2C here is LPI2C), and the sensor component already pulls LPI2C in.
- `CMakeLists.txt` uses `mcux_add_source` / `mcux_add_include`, not raw `target_sources`. Relative paths like `../policy/...` work.
- `board.c` and `clock_config.c` come from `mcuxsdk/examples/_boards/frdmmcxn236/` (boot clock: PLL 150 MHz core). Each app owns `pin_mux.[ch]`, `hardware_init.c` and `app.h` in its `board/`; `prj.conf` turns off the SDK's own copies (`pinmux_customize_folder=y`, `use_hw_app=n`).
- `mcux_config.h` (generated from Kconfig) is force-included in every file with `-include`. Don't `#include` it. A Kconfig symbol with "No prefix in generated macro" in its help becomes a bare macro (that is how `BOARD_DEBUG_UART_BAUDRATE` overrides `board.h`).
- Builds use `-Werror`.
- `BOARD_InitHardware()` attaches peripheral clocks, then calls `BOARD_InitPins()`, `BOARD_InitBootClocks()` and `BOARD_InitDebugConsole()`.
- `BOARD_Accel_I2C_Init/Send/Receive` in the SDK's `board.c` exist only when `SDK_I2C_BASED_COMPONENT_USED=1`, which the apps define with `mcux_add_macro`.
- Reference examples: `mcuxsdk/examples/_boards/frdmmcxn236/demo_apps/bubble` (accelerometer), `mcuxsdk/examples/issdk_examples/sensors/fxls8974cf/`, `driver_examples/ctimer/simple_pwm` and `.../pwm/` (PWM).

## On-board sensors (user manual §2.5–2.8)

| Sensor | Part | MCU connection |
|---|---|---|
| 3-axis accelerometer | NXP FXLS8974CFR3 (U10) | LPI2C2 / FC2: P4_0 = SDA, P4_1 = SCL. INT1 = P0_21, INT2 = P0_24, WAKE = P0_23 (not connected by default) |
| Ambient light | Everlight ALS-PT19 (Q2) | P0_25 = ADC0_B17 |
| Digital microphone | SPK0641HT4H-1 | PDM0, clock on P0_16 |

## Accelerometer (FXLS8974CF), verified on hardware

- 7-bit I2C address **0x18** (SA0 = 0). The manual's "0x30/0x31" are the 8-bit write/read forms. WHO_AM_I (0x13) = **0x86**.
- LPI2C2 is clocked from FRO 12M (`kFRO12M_to_FLEXCOMM2`), which matches `BOARD_ACCEL_I2C_CLOCK_FREQ` in `board.h`. The bus runs at 100 kHz.
- `FXLS_Init()` checks WHO_AM_I and sets the sensor active. It leaves FSR at the default **±2 g**.
- Output is 12-bit signed, read from OUT_X_LSB (0x04), 6 bytes for X/Y/Z. **mg = raw × range_g × 1000 / 2048** (about 0.98 mg/LSB at ±2 g). Range comes from SENS_CONFIG1[2:1]: 00/01/10/11 = ±2/4/8/16 g.
- Lying flat, the board reads about X ≈ 0, Y ≈ 0, **Z ≈ +1.08 g**. The extra ~80 mg is part offset/sensitivity tolerance. `tilt.c` normalizes the vector, so it cancels out there.
- `hardware_init.c` runs an I2C bus release (9 SCL clocks + STOP as GPIO) before muxing the pins to LPI2C. This recovers the sensor if a reset happened mid-transfer and it is holding SDA low.
- P0_21, P0_23 and P0_24 are shared with the FlexIO/LCD header J8 and the Arduino headers, so check for conflicts before using the INT pins.
- The manual doesn't give the sensor's axis directions. The tilt defaults assume +X = forward, +Y = left; not yet confirmed by tilting.

## Policy on the board, verified on hardware

Hardware-in-the-loop: every 20 ms `test/run.py` sends ANYmal C's 51 observations over the MCU-Link VCOM (LPUART4, P1_8/P1_9,
921600 baud). The board reads its accelerometer, runs the 51-512-256-128-12 ELU MLP, and replies with 12 actions, the tilt
command (forward, left in Go2 m/s) and the DWT cycle count. The command enters the next observation (one step of latency).

- One inference takes **11.3 ms** (1.69 M cycles, about 8.8 cycles/MAC: flash-bandwidth bound). Round trip with the
  accelerometer read: **15.2 ms** of the 20 ms step. `policy_net.c` is always built with `-O3`.
- Weights: float32 `const` arrays in flash, 752 kB of 1 MB (image ~790 kB). Only the flat policy fits; OnePolicy_r
  (238 inputs, ~1.1 MB) would need fp16/int8 weights.
- Board actions match ONNX Runtime to ~1e-6. `run.py` checks every step and stops above 1e-3 (the board holds another
  policy). The board's `I` reply carries the weights' CRC32 (`POLICY_MODEL_CRC`; OnePolicy.onnx = 0x129AEBBF).
- debug_console_lite polls (no RX interrupt), so the protocol reads/writes LPUART4 directly next to PRINTF (boot banner
  only). Close any terminal on the port while `run.py` runs.
- The VCOM's /dev/ttyACM<n> can change after flashing. `test/board.py` finds it by `/dev/serial/by-id/*MCU-LINK_FRDM-MCXN236*-if02`.

## Simulation (`test/anymal.py`), checked against OnePolicy

- Rebuilds OnePolicy's flat play env for `anymal_c_xknee` in plain MuJoCo 3.5.0 (CPU). Observations, in order: IMU gyro (3),
  projected gravity (3), command in robot units (3), gait phase sin/cos with period 0.6 × √s_L, zeroed when |Go2 command| < 0.1
  (2), joint pos − default (12), joint vel (12), knee direction = sign(default calf) (4), last action (12).
- Action: target = default + 0.25 × action, then each of the 4 × 5 ms substeps applies PD torque (kp, kd from robot.json),
  clipped by the DC motor's torque-speed line (saturation, velocity limit 18.85 rad/s, effort limit). Ctrl order is grouped
  hips/thighs/calves; robot.json maps each joint to its ctrl index.
- Commands are Go2 units: forward/left × √s_L, yaw / √s_L (s_L = 1.4786 for ANYmal C).
- `mj_step` leaves `xmat` and `sensordata` at the last substep's start, so `step()` ends with `mj_forward` (mjlab re-senses).
- Against a OnePolicy rollout from the same states: observations match to 1e-6, one 20 ms step to 7e-6 rad. Closed loop it
  walks 0.49-0.50 Go2 m/s for a 0.5 command, like OnePolicy. Sideways (0.3 -> 0.06) and backward (-0.3 -> -0.09) are weak in
  OnePolicy itself; that is the checkpoint, not the sim.
- Startup randomisation (friction, COM, encoder bias) is left out: the assets are the nominal robot.
