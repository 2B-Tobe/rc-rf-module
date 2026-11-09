# Building with STM32CubeIDE

The folder is a complete STM32CubeIDE project (`.project`, `.cproject`,
linker script, start-up file, run configuration), so it is imported like
any other project archive. Nothing has to be generated or copied by hand.

## Import the zip

1. Start STM32CubeIDE. Use the workspace you want (File → Switch Workspace
   → Other... to make a new one, e.g. `workspace_RF_Module_Controller`).
2. **File → Import...**
3. Open **General**, pick **Existing Projects into Workspace**, Next.
4. Tick **Select archive file**, Browse... to the zip (e.g.
   `C:\dev\RF_Module_Controller_1.2.zip`), Open.
5. `RF Module Controller` appears in the Projects list, ticked. Finish.

The project now sits in the workspace with `Core`, `third_party` and the
other files in the Project Explorer.

## Build and flash

6. Select the project, **Project → Build Project** (or the hammer). The
   Console ends with the size: about 13 KB of flash, no errors, no
   warnings.
7. Plug the ST-Link into the box (SWDIO, SWCLK, GND, 3.3 V) and the box's
   power if the ST-Link does not supply it.
8. Right-click the project → **Run As → STM32 C/C++ Application**. The
   first time, CubeIDE opens the run configuration it found in the project
   (`RF Module Controller.launch`): press **OK**. It flashes the box and
   starts it; the display shows `RF MODULE CONTROLLER v1.2` and then reads
   the module (or `No answer on J3` when none is connected).

Later changes: edit, Build, Run As.

## Options

To receive the module's answer on another pin than PA8, add
`LINK_RX_PIN=10` or `LINK_RX_PIN=3` to the defined symbols: Project →
Properties → C/C++ Build → Settings → MCU GCC Compiler → Preprocessor →
Define symbols (-D), the + button. See `Core/Inc/serial.h`.

## What is where

| | |
|---|---|
| `Core/Inc`, `Core/Src` | the controller: `main.c` (menu), `link.c` (the cfg command and its answers), `serial.c` (USART1 TX, software receiver), `display.c`, `board.c` (pins, clocks, keys), `syscalls.c` (library stubs) |
| `Core/Src/u8glib` | the display library (subset of u8glib, 2-clause BSD) |
| `third_party/cmsis_core` | Arm CMSIS core headers (Apache-2.0) |
| `third_party/cmsis_device_f1` | ST device header, start-up file and system file for the STM32F1 (Apache-2.0) |
| `STM32F103C8TX_FLASH.ld` | linker script, 64 KB flash / 20 KB RAM |
| `build.sh` | command-line build with arm-none-eabi-gcc, for those without the IDE |
| `docs/MANUAL.md` | how to use the box |
