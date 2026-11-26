# Seeed Studio XIAO MG24 as a development board

**Status (26 Nov 2026): builds; not yet run on a XIAO.** The pin and board facts below are from Seeed's wiki,
Zephyr's board files and Silicon Labs' Arduino core for this board, not from a board on the bench.

The XIAO MG24 carries the same chip family as the module (EFR32MG24, 1536 KB flash, 256 KB RAM, 39 MHz crystal,
+19.5 dBm), with a USB programmer / serial bridge on the board. The firmware has a build for it: **2.4 GHz only**,
every 2.4 GHz mode of the module, log and command line over the USB cable.

## What is different from the module

| | Module | XIAO MG24 |
|---|---|---|
| Log and command line (`cfg ...`) | PA05 / PA06, FTDI lead | the USB cable (115200 8N1), listens in every mode |
| Signal input: MULTI frames from a radio or an encoder, or PPM | J3 RX (PA06) | **D7** (PC07) |
| PPM input, telemetry to the encoder | PB01 | **D16** (PB01), the same pin |
| 459 MHz radio (dual) | SX1278 | none: Dual is ignored |
| TX panel (display, buttons) | optional | none |
| Receiver outputs | 8 PWM, SBUS, PPM | **SBUS on D8, PPM on D3**; no PWM servo outputs |
| Bind plug on servo connector 8 | yes | no (use the BIND pad or `cfg bind`) |
| Antenna | the module's | ceramic on the board, or U.FL (`XIAO_ANTENNA_UFL` in `board_config.h`) |
| Yellow LED on the board | – | follows LED_B |

## Pads

| Pad | Pin | Use |
|---|---|---|
| D0, D1, D2, D3 | PC00–PC03 | protocol jumpers to GND, as J1 on the module; D3 is also PPM out in receiver roles |
| D4, D5 | PC04, PC05 | LED_A, LED_B (optional LEDs to GND through a resistor) |
| D6 | PC06 | free |
| D7 | PC07 | **signal input**: MULTI serial (100000 8E2) or PPM, 3.3 V |
| D8 | PA03 | SBUS out (receiver roles) |
| D9 | PA04 | receiver battery sense (divider, as on the module) |
| D10 | PA05 | free |
| D13 (back) | PB02 | RANGE, to GND |
| D14 (back) | PB03 | F/S, to GND |
| D15 (back) | PB00 | BIND, to GND |
| D16 (back) | PB01 | PPM in / telemetry out to the encoder |
| D17 (back) | PA00 | free |
| D18 (back) | PD02 | buzzer (RLINK transmitter) |

All pads are 3.3 V only. D8, D9 and D10 are shared with the XIAO's own flash memory chip; the firmware keeps that
chip switched off.

## Building for the XIAO

1. Open `firmware\board_config.h`.
2. Change the line `#define BOARD             BOARD_MODULE` to `#define BOARD             BOARD_XIAO_MG24`.
3. For the U.FL antenna connector set `XIAO_ANTENNA_UFL` to `1` (default `0` = the antenna on the board). Save.
4. `cd C:\dev\rc-rf-module\firmware`
5. `.\build.ps1` — the first lines must say `Board: Seeed XIAO MG24`.

**A XIAO build must not be flashed onto the module** (and the other way round): the pins differ. Set the line back
to `BOARD_MODULE` and run `.\build.ps1` again before the next module build.

## Flashing the XIAO

The XIAO's own USB port is the programmer. No ST-Link.

1. Unplug the ST-Link, so that pyOCD sees only the XIAO.
2. Plug the XIAO into USB.
3. Once only: `pyocd pack install EFR32MG24B220F1536IM48`
4. `pyocd flash -t efr32mg24b220f1536im48 -O adi.v5.max_invalid_ap_count=0 cmake_gcc\build\base\rc_module_trx.hex`
5. Open PuTTY on the XIAO's COM port (Device Manager shows it), 115200. Press the XIAO's reset button.
6. The log must start with `Simple TRX`, then a line `BOARD Seeed XIAO MG24 ...`.
7. Type `cfg` and Enter: the stored setup is shown. `cfg tx duplex`, `cfg rx 0 1` and so on work as on the module.

This replaces whatever was on the XIAO (the Arduino bootloader included). To use it with the Arduino IDE again,
write the bootloader back from there (Tools → Burn Bootloader).

## First test

1. Flash two XIAOs, or one XIAO and one module.
2. `cfg tx duplex` on one, `cfg rx duplex` on the other (or any other protocol both have).
3. `cfg bind` on both, about a metre apart.
4. The logs must show the link as on the module (`JQ tx ...` / `JQ rx link ...`).

## Not known yet

- Whether the firmware built for the module's chip (EFR32MG24**A420**) runs unchanged on the XIAO's
  (EFR32MG24**B220**). Same family and memory; expected to work, not tried.
- The crystal trim: 95 is the value Silicon Labs and Zephyr use for this board. A frequency offset shows first on
  the narrow modes (FrSky, S-FHSS); Jeti, RLINK and nRF24 tolerate more.
- Range with the ceramic antenna.
