# RF Module Controller

A small hand-held box that sets up an EFR32 RF module running the
[rc_module_trx](https://github.com/2B-Tobe/rc_module_trx) firmware — role
(transmitter / receiver), protocol, single or dual link — and binds a
receiver, all without jumpers. It talks to the module's `cfg` text
command over the module's J3 serial pins and shows the module's answer on
a 128×64 display.

Hardware: an STM32F103C8 (a "Blue Pill" class board), a 128×64 UC1701
SPI display (e.g. JLX12864), four menu keys and one extra key, two 3-pin
leads to the module. Pin assignment in `Core/Inc/board.h`.

| Function | Pins |
|---|---|
| Display SPI2 | SCK PB13, MOSI PB15, CD/A0 PB14, RST PB12, backlight PB2 |
| Keys (to GND) | INC PB4, DEC PA10, BACK PB11, ENTER PB3, AUX PB8 |
| To the module | PA9 (USART1 TX) → module J3 RX, 115200 8N1 |
| From the module | PA8 (software receiver, default) ← module J3 TX; alternatives PA10 / PA3, see `Core/Inc/serial.h` |

## Build

**STM32CubeIDE** (the normal way): the folder is a complete CubeIDE
project — File → Import → Existing Projects into Workspace → Select archive
file, then Build and Run As. Step by step in `docs/CUBEIDE.md`.

**Command line:** `sh build.sh` with `arm-none-eabi-gcc` on the path →
`build/module_controller.hex`. Uses the same sources, the Arm CMSIS and ST
device files in `third_party/` and the linker script in the root.

## Use

`docs/MANUAL.md`.

## Licence

MIT (see `LICENSE`). u8glib (2-clause BSD), Arm CMSIS (Apache-2.0) and the
ST device files (Apache-2.0) keep their own licences, kept next to them.

## Versions

- 1.3 — a receiver's output layout: row 4 reads **Out** for a receiver (8 PWM / 5+SBUS / 5+PPM / 5+S+P:
  SBUS on servo connector 6, PPM on connector 7), sent as the number after Sub (`cfg rx 6 2 1`) and read back
  from the module. Needs module firmware with receiver output layouts; older firmware ignores it.
- 1.2 — the setup is sent as the module's Prot / Sub numbers (`cfg tx 15 2 dual`), the same as a
  transmitter's serial frames, so the box can set every mode the frames can, in both roles; the FrSky D16
  versions (v1 / v2, EU-LBT / FCC, FCC 8 channels) are in the list. Row 1 is built from the stored numbers.
- 1.1 — complete STM32CubeIDE project (import the zip as it is); own linker
  script; `syscalls.c` stubs. The firmware itself is unchanged from 1.0.
- 1.0 — first clean version: role / protocol / option / Send / Bind menu,
  BACK reads the module, BACK held clears, AUX = bind, software receiver on
  PA8 with automatic polarity.
