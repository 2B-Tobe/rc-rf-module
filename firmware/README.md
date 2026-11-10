# Firmware

The EFR32MG24 project: Simplicity SDK 2026.6.1 with RAIL, built with GCC through CMake/Ninja. The Simplicity
Studio project is `rc_module_trx.slcp` (the project keeps its development name; the firmware prints
`BUILD dual-serial` at power-up — that is this firmware).

## Files

| | |
|---|---|
| `app_process.c` | the module: roles, jumper / stored-setup decode, PPM input, LEDs, the main loop; the protocol files are `#include`d into it |
| `app_init.c`, `app_cli.c`, `main.c` | SDK start-up, the log UART and command line |
| `bay_serial.c/.h` | serial input from a transmitter: Multiprotocol frames on PA06, either polarity (see `docs/SERIAL_INPUT.md`) |
| `etx_multi.c/.h` | the module as a MULTI module for an EdgeTX radio: protocol numbers, status, protocol list, telemetry |
| `rx_setup.c/.h` | receiver setup over the air: setup packets from the transmitter, the 3 s window in the receiver |
| `cfg.c/.h`, `cfg.ps1` | the stored setup and the `cfg` command; `cfg.ps1` sends commands from PowerShell |
| `rlink.c/.h` | RLINK transmitter and receiver (`docs/RLINK.md`) |
| `nrf24_rx.c/.h` | nRF24 receiver (the nRF24 transmitter is in `app_process.c`) |
| `frsky_d8.c/.h`, `frsky_d16.c/.h` | FrSky D8 and D16 (ACCST v1/v2, EU-LBT and FCC), transmitter and receiver |
| `dsm_tx.c/.h`, `dsm_rx.c/.h`, `dsm_cap.c/.h` | DSMX transmitter, receiver and the raw-capture tool |
| `uk459_tx.c/.h`, `uk459_rx.c/.h` | the 459 MHz link on the SX1278, both directions |
| `jeti_cap.c/.h`, `tx_test.c` | 802.15.4 capture tool, carrier test |
| `servo_out.c/.h`, `servo_pins.h`, `sbus_out.c/.h` | receiver outputs: 8 servos + PPM on timer compare outputs, SBUS on EUSART1 |
| `rx_store.c/.h` | binds and failsafe presets in flash page 0x08040000 (8 slots); the setup is in page 0x08042000. Both survive a firmware update |
| `config/`, `autogen/` | SDK configuration and generated files (radio configuration in `config/rail/`) |
| `cmake_gcc/` | the CMake build (`build/` is generated and not committed) |
| `simplicity_sdk_2026.6.1/` | the SDK subset the project uses, without the RAIL library and the iostream printf files (they come with the SDK; see below) |
| `Prebuilt/` | ready-to-flash hex files |

## Toolchain

1. Install **Simplicity Studio 6** (or VS Code with the Silicon Labs extension) with the Simplicity SDK
   **2026.6.1** and the GNU Arm toolchain it offers. Python 3 with `pyocd` for flashing
   (`pip install pyocd`, then once: `python -m pyocd pack install efr32mg24a420f1536im48`).
2. Open / import `rc_module_trx.slcp`. Importing generates `cmake_gcc/toolchain.cmake` with the paths of the
   compiler and Ninja on your PC (the `autogen/` and `config/` folders are already in the repository; let the
   generator overwrite them if it asks).
3. Build from the IDE, or from PowerShell:

```
cd C:\dev\rc-rf-module\firmware
.\build.ps1
```

The RAIL radio library (`librail_efr32xg24_gcc_release.a`) and `iostream_printf.c/.h` are Silicon Labs files that are
not in the repository: the library is a binary (Zlib), and the printf files are under the MSLA, which does not allow
sharing their source. Importing the
project normally copies them in from your SDK; if not, `build.ps1` finds them in the installed Simplicity SDK 2026.6.1 (checked by SHA-256) and copies them in. If it cannot find them it stops and says so: install the SDK
2026.6.1 and import the project.

Result: `cmake_gcc\build\base\rc_module_trx.hex`. `build.ps1` touches `app_process.c` first so that every
`#include`d protocol file is rebuilt. A build that prints only `[2/2] Linking` compiled nothing — if files
were copied in with an old time stamp, run
`Get-ChildItem *.c, *.h | ForEach-Object { $_.LastWriteTime = Get-Date }` and build again.

## Flashing (ST-Link V2 or any CMSIS-DAP / J-Link probe, SWD)

```
python -m pyocd flash -t efr32mg24a420f1536im48 C:\dev\rc-rf-module\firmware\cmake_gcc\build\base\rc_module_trx.hex
```

The output must end with `programmed ... identical 0 bytes`; "identical N bytes" means the hex did not change.
A `Transfer error while reading AHB-AP#2 ROM table` line at the start is harmless. Simplicity Commander flashes
the same hex. The module's binds and stored setup are not touched by flashing.

## Setting a module up from the PC

`cfg.ps1` talks to the module's serial pins through a USB-serial adapter (adapter TX → J3 RX PA06, adapter RX
← J3 TX PA05, GND; 3.3 V levels). Close any terminal on that port first.

```
.\cfg.ps1 -Ports
.\cfg.ps1 -Port COM7
.\cfg.ps1 -Port COM7 -Command "cfg rx rlink"
.\cfg.ps1 -Port COM7 -Command "cfg tx rlink dual"
.\cfg.ps1 -Port COM7 -All
```

Commands and answers: `docs/SERIAL_INPUT.md` section 3; what they mean: `docs/QUICK_SHEET_EFR32_Module.md`.
The same commands can be typed in any terminal at 115200 8N1 (end the line with Enter).

## Build-time options

| Define | Default | |
|---|---|---|
| `DTX_EU_LOCK` | 1 | DSMX 22 ms only, 18 dBm (EU). 0 adds DSMX 11 ms |
| `RL_VB_PACK` | 1 | RLINK receiver battery alarm thresholds: 1 = 4 × AA / 5 V, 2 = 2S LiPo |
| `LINK_RX_PIN` | — | (Controller, not here) |

Set them in `cmake_gcc/CMakeLists.txt` under `target_compile_definitions(rc_module_trx PUBLIC ...)`.
