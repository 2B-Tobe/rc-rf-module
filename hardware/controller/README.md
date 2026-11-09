# Controller board (rc-rf-controller)

The hand-held box: STM32F103C8, a 128×64 UC1701 SPI display (JLX12864), five keys, buzzer, two 3-pin leads
to the module and an SWD header. Four-layer board.

## Files

| | |
|---|---|
| `rc-rf-controller.pdf` | the schematic |
| `kicad/` | the KiCad project (KiCad 5 format; KiCad 6 and later open and convert it). The tactile switch symbol is in the project library `rc-rf-controller-parts`; everything else is from KiCad's standard libraries; footprints are embedded in the board file |
| `gerber/` | Gerber and drill files, and the same as one zip for the board house |
| `bom/` | `rc-rf-controller-top-pos.csv` — component placement, top side |

## Connections (STM32 pins, see `controller/Core/Inc/board.h`)

| Function | Pin |
|---|---|
| Display SPI2: SCK, MOSI, CD/A0, RST, backlight | PB13, PB15, PB14, PB12, PB2 |
| Keys to GND: INC, DEC, BACK, ENTER, AUX | PB4, PA10, PB11, PB3, PB8 |
| To the module: J3 RX | PA9 (USART1 TX) |
| From the module: J3 TX | PA8 (software receiver; alternatives PA10, PA3 — `Core/Inc/serial.h`) |
| SWD | SWCLK PA14, SWDIO PA13, GND, 3.3 V |

Supply 3.3 V on board. The module's J3 pins are 3.3 V logic as well; GND common.
