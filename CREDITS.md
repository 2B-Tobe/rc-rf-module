# Credits and third-party licences

## Author

**Tobe (2B-Tobe)** — hardware, firmware, the RLINK, nRF24 and 459 MHz links, the Controller, the documentation.

## Firmware (`firmware/`) — GPL-3.0, with these components

| Component | Where | Licence | Note |
|---|---|---|---|
| Silicon Labs Simplicity SDK 2026.6.1 (platform, emlib, peripherals, CLI, iostream, app_log, device init, DMA manager, CMSIS-Core port) | `firmware/simplicity_sdk_2026.6.1/` | Zlib | the subset the project uses, copied in by the Simplicity Studio project generator; `firmware/autogen/sbom/` holds Silicon Labs' bill of materials with the licence of every component |
| Silicon Labs RAIL library (`librail_efr32xg24_gcc_release.a`) and RAIL utility plugins | `firmware/simplicity_sdk_2026.6.1/rail_library/` | Silicon Labs MSLA | binary radio library, redistributed for use on Silicon Labs devices under the Master Software License Agreement |
| Arm CMSIS-Core | `firmware/simplicity_sdk_2026.6.1/cmsis/` | Apache-2.0 | |
| Tiny printf by Marco Paland | `firmware/simplicity_sdk_2026.6.1/printf/` | MIT | |
| DIY Multiprotocol TX Module (Pascal Langer and contributors) | — | GPL-3.0 | the serial frame format the module takes from a transmitter (header, protocol, sub-protocol, RX number, option, 16 × 11-bit channels); the DSMX channel-list computation (`DSM_calc_dsmx_channel`) that `dsm_tx.c` / `dsm_rx.c` follow |
| Deviation (deviationtx) | — | GPL-3.0 | the CYRF6936 PN code tables and the DSM2/DSMX framing (SOP, SDR/8DR symbols) that the DSM modes reproduce in software on the EFR32 radio |
| FrSky D8 / D16 (ACCST) protocol descriptions | — | — | as documented by the Multiprotocol and Deviation projects and the open-source RC community; implemented here from those descriptions |
| Futaba S.BUS frame format | — | — | public description |
| Multiprotocol telemetry format ("MP" frames: status, protocol list, FrSky hub / S.Port) | — | GPL-3.0 | as defined by the DIY Multiprotocol TX Module, so an EdgeTX radio sees the module as a MULTI module (`etx_multi.c`) |
| EdgeTX (and OpenTX before it) | — | GPL-2.0 | its MULTI driver and telemetry parser were read to match what the radio sends and expects: frame polarity and timing, byte 26, the protocol scan, status and sensor handling. No EdgeTX code is copied |

The RLINK protocol, the nRF24 link, the UK 459 MHz link, the encoder serial control, the EdgeTX MULTI-module behaviour, the stored setup
(`cfg`), the receiver servo/PPM/SBUS output and the capture tools are this project's own work.

## Controller (`controller/`) — MIT, with these components

| Component | Where | Licence |
|---|---|---|
| u8glib by olikraus (display library, subset) | `controller/Core/Src/u8glib/`, `controller/Core/Inc/u8g.h` | BSD-2-Clause |
| Arm CMSIS-Core (CMSIS_5) | `controller/third_party/cmsis_core/` | Apache-2.0 |
| STMicroelectronics cmsis_device_f1 (device header, start-up file, system file) | `controller/third_party/cmsis_device_f1/` | Apache-2.0 |

## Hardware

| Part | Maker | Note |
|---|---|---|
| RF-BM-MG24B2I module (EFR32MG24) | RF-star | the 2.4 GHz radio and MCU |
| RA-01 module (Semtech SX1278) | Ai-Thinker | the 459 MHz radio |
| STM32F103C8 "Blue Pill" class board, UC1701 128×64 display (JLX12864) | various | the Controller |

## Tools

Simplicity Studio 6 and the Silicon Labs VS Code extension, GNU Arm Embedded Toolchain, CMake and Ninja,
pyOCD (Apache-2.0), STM32CubeIDE, KiCad.

## Trademarks

FrSky, ACCST, Spektrum, DSM2, DSMX, Futaba, S.BUS, Jeti, Lemon RX, OrangeRX and the other names used here
belong to their owners and are used only to say which equipment the module talks to or was tested with.
