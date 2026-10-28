# CLAUDE.md

Accelerometer readout firmware for the **NXP FRDM-MCXN236** board (MCU: MCXN236VDFT, Arm Cortex-M33 @ 150 MHz, 1 MB flash, 352 KB SRAM, 184VFBGA).
It is built on the **MCUXpresso SDK v26.09.00** with arm-none-eabi-gcc, CMake and Ninja, and flashed with **pyOCD** through the board's on-board MCU-Link (CMSIS-DAP).

- MCU datasheet: `docs/MCXN23x.pdf` (Rev. 3, 21 Jan 2025).
- Board user manual: `docs/UM12041-FRDM-MCXN236.pdf` (UM12041 Rev. 3.0). It has the header pinouts, on-board peripherals, and jumper/resistor options.

Cite a table or section from these when a hardware decision depends on it.

## Layout

```
manifest/west.yml   West manifest; pulls only the SDK repos this board needs (pinned to v26.09.00)
mcuxsdk/            MCUXpresso SDK checkout (git-ignored; regenerate with `uv run west update`)
accel/              The application (freestanding SDK project; pinmux and bus recovery taken from demo_apps/bubble)
  main.c            Init the FXLS8974CF, then print X/Y/Z in mg forever
  Kconfig           Tunable: ACCEL_SAMPLE_PERIOD_MS
  board/            app.h (I2C address/pins), hardware_init.c (clocks + I2C bus release), pin_mux.[ch]
docs/               MCU datasheet + board user manual
pyproject.toml      Python tooling (west, pyocd, SDK build-script deps), managed by uv
env.sh              Sets ARMGCC_DIR for the SDK's CMake toolchain file
pyocd.yaml          Pins the pyOCD target (mcxn236vdf) so `-t` isn't needed
```

Don't edit anything under `mcuxsdk/`. Copy what you need into `accel/` and change it there.

## Python: uv only

Manage every Python dependency through uv: `uv add <pkg>`, `uv sync`, and `uv run <tool>`.
Never use pip, `uv pip`, or `.venv/bin/...` directly.

## Setup (fresh clone)

```bash
sudo pacman -S --needed arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-gdb cmake ninja
uv sync                                    # west, pyocd, SDK Python deps
uv run west update --narrow -o=--depth=1   # fetch the SDK (~2 GB)
uv run pyocd pack install MCXN236VDF       # CMSIS pack (flash algorithm) for pyOCD
source env.sh                              # ARMGCC_DIR=/usr
```

## Build / flash / debug

Run these from the repo root after `source env.sh`:

```bash
uv run west build -p always accel -b frdmmcxn236 -d build --config debug   # or --config release
uv run pyocd flash build/*.elf
uv run pyocd gdbserver       # then: arm-none-eabi-gdb build/*.elf -ex "target remote :3333"
picocom -b 115200 /dev/ttyACM0             # debug console (LP_FLEXCOMM4 UART over the MCU-Link USB, J10)
```

Connect only the **MCU-Link USB-C port (J10)**. It provides SWD flashing and debugging plus the serial console, so no external probe is needed.
`uv run pyocd list` should show "MCU-LINK FRDM-MCXN236". The board reports itself as `mcxn236vdft`, which pyOCD doesn't know, so `pyocd.yaml` overrides it to `mcxn236vdf`.
The SDK's west runners for this board only support LinkServer and J-Link, so `west flash` isn't wired up. Call pyOCD directly as shown above.

## SDK conventions (new MCUXpresso build system)

- Kconfig selects which drivers are built. Enable one in `accel/prj.conf`, e.g. `CONFIG_MCUX_COMPONENT_driver.fxls8974cf=y`. On this chip `driver.i2c` doesn't apply (the I2C here is LPI2C), and the sensor component already pulls LPI2C in.
- The project's `CMakeLists.txt` uses `mcux_add_source` / `mcux_add_include`, not raw `target_sources`.
- `board.c` and `clock_config.c` come from `mcuxsdk/examples/_boards/frdmmcxn236/` (boot clock: PLL 150 MHz core). The app owns `pin_mux.[ch]`, `hardware_init.c` and `app.h` in `accel/board/`; `prj.conf` turns off the SDK's own copies (`pinmux_customize_folder=y`, `use_hw_app=n`).
- `mcux_config.h` (generated from Kconfig) is force-included in every file with `-include`. Don't `#include` it.
- Builds use `-Werror`.
- `BOARD_InitHardware()` attaches peripheral clocks, then calls `BOARD_InitPins()`, `BOARD_InitBootClocks()` and `BOARD_InitDebugConsole()`.
- `BOARD_Accel_I2C_Init/Send/Receive` in the SDK's `board.c` exist only when `SDK_I2C_BASED_COMPONENT_USED=1`, which `accel/CMakeLists.txt` defines with `mcux_add_macro`.
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
- Lying flat, the board reads about X ≈ 0, Y ≈ 0, **Z ≈ +1.08 g**. The extra ~80 mg is part offset/sensitivity tolerance. Correct it with the OFF_X/Y/Z registers (0x22–0x24) if accuracy matters.
- `hardware_init.c` runs an I2C bus release (9 SCL clocks + STOP as GPIO) before muxing the pins to LPI2C. This recovers the sensor if a reset happened mid-transfer and it is holding SDA low.
- P0_21, P0_23 and P0_24 are shared with the FlexIO/LCD header J8 and the Arduino headers, so check for conflicts before using the INT pins.
