# RC RF Module

A 2.4 GHz multi-protocol RC transmitter module and receiver on one board, built around the EFR32MG24
(RF-BM-MG24B2I module), with an optional second radio — an SX1278 (RA-01) — that carries the same channels
on 459 MHz beside the 2.4 GHz link. One firmware serves both roles and both board variants.

| | |
|---|---|
| **Transmitter module** | takes the channels from a transmitter's encoder — PPM, or Multiprotocol serial frames — and sends them on nRF24, RLINK, FrSky D8, FrSky D16 (ACCST v1/v2, EU-LBT or FCC) or DSMX, with the 459 MHz link beside it when the RA-01 is fitted |
| **Receiver** | 8 servo outputs, PPM and SBUS; with the RA-01 fitted it listens on both bands and takes the servos from 459 MHz when 2.4 GHz goes quiet |
| **Setup** | protocol jumpers, or a setup stored in the module: written over its serial pins with the hand-held **Controller** (this repository) or a terminal |
| **Links of its own** | RLINK (two-way 2.4 GHz with telemetry, adaptive channels and power, alarms), nRF24 (the original 11 ms link), UK 459 MHz (10-byte hopping link) |

## Two board variants, one firmware

- **2.4 GHz only** — the RA-01 and its parts are left off. The firmware notices (`no SX1278 answered`) and runs
  2.4 GHz only.
- **2.4 GHz + 459 MHz** — the RA-01 fitted. The firmware runs the 459 MHz link beside every transmitter
  mode (selectable Single / Dual) and the 459 MHz receiver beside every receiver mode.

The servo outputs are on the same pins for both variants (`firmware/servo_pins.h`).

## What is where

| Folder | |
|---|---|
| `firmware/` | the EFR32 project (Simplicity SDK 2026.6.1, RAIL), `build.ps1`, `cfg.ps1`, prebuilt hex files — see `firmware/README.md` |
| `controller/` | the RF Module Controller: an STM32F103C8 box with a 128×64 display that sets the module up over its serial pins — STM32CubeIDE project, manual |
| `hardware/rf_module/` | the module carrier board: KiCad project, Gerbers, BOM and placement files |
| `hardware/controller/` | the controller board: KiCad project, Gerbers, BOM and placement files |
| `docs/` | operating quick sheet, flashing with an ST-Link, the serial input and setup command reference, the RLINK description |
| `CREDITS.md` | people, projects and components this builds on, with their licences |

## Start here

1. **Build a board** — `hardware/rf_module/README.md` (what to order, what to leave off for the 2.4 GHz-only variant).
2. **Flash it** — with only an ST-Link: `docs/FLASHING_STLINK.md` (the prebuilt firmware and a setup file for each
   role, no USB-serial adapter needed); to build it yourself: `firmware/README.md`.
3. **Use it** — `docs/QUICK_SHEET_EFR32_Module.md`: connections, roles, modes, bind, failsafe, LEDs.
4. **Feed it** — `docs/SERIAL_INPUT.md`: what a transmitter has to send (PPM or the Multiprotocol serial frame),
   and the text commands any programmer can use to set the module up.
5. **Controller** — `controller/README.md` and `controller/docs/MANUAL.md` to build and use the setup box.

## Regulatory note

The 2.4 GHz side is built for EU use: DSMX at 22 ms only and 18 dBm, FrSky D16 EU-LBT (Sub 0/1) with listen-before-talk (the D16 FCC versions, Sub 2/3/5, are for receivers with FCC firmware and have none), RLINK
with a clear-channel check before every packet. The 459 MHz link uses the UK 458.5–459.5 MHz model-control
band at 100 mW; check what your country allows before fitting the RA-01.

## Licence

The firmware, the documentation and the hardware design files are released under the **GNU GPL v3** (see
`LICENSE`). The Controller (`controller/`) is under the **MIT licence** (`controller/LICENSE`). Third-party
components keep their own licences, listed in `CREDITS.md`. The Silicon Labs RAIL radio library is a binary under
the Silicon Labs MSLA and is **not** in this repository: it comes with the Simplicity SDK, and `build.ps1` copies
it in. The firmware licence includes an additional permission to build it with that library (top of `LICENSE`).
