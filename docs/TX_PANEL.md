# TX panel — display, buttons and LED for the transmitter module

The TX panel is an add-on that turns the module into a stand-alone transmitter module with its own settings:
a small display, three (or four) buttons and a red / green LED. Mode, Single / Dual, input, bind and range
check are set on the module itself, without the Controller or a radio menu.

- **One firmware for every board.** At power-up the firmware looks for the display. Found: TX panel (transmitter
  only). Not found: the module works exactly as before.
- **No change to the module board.** The panel uses pins the board already has: the receiver's servo pins, the
  protocol jumpers and BIND. A panel board can therefore only be fitted to a module used as a transmitter.
- **Status:** firmware panel v0.3, bench-tested on the author's boards. The wiring below is for today's
  Multi_Mode board; a plug-in panel board (schematic, PCB, BOM) will follow in `hardware/`.

## 1. Parts

| Part | |
|---|---|
| Display | 0.42" OLED, 72 × 40, SSD1315 (SSD1306-compatible), I²C, 3.3 V; module about 12 × 17 mm, pins GND VCC SCK SDA |
| Buttons | 3 (Up, Down, Enter/Bind) or 4 (+ Back), push buttons to GND |
| LED | two-colour red / green, common cathode (not RGB); a standard green (about 2.1 V), not "pure green" |
| Resistors | 2 × 150 Ω for the LED (about 8 mA from 3.3 V); 2 × 4.7 k I²C pull-ups to 3.3 V **only if the display module has none** |

## 2. Wiring on today's board

| Function | EFR32 pin | Board signal | Header pin |
|---|---|---|---|
| Display SDA | PA08 | servo 6 | J9-5 |
| Display SCK | PA09 | servo 7 | J8-6 |
| Display VCC / GND | — | +3.3 V / GND | J7-2 / J7-1 |
| Up | PC00 | protocol jumper 0 | J1-8 (GND J1-7) |
| Down | PC01 | protocol jumper 1 | J1-6 (GND J1-5) |
| Enter / Bind | PB00 | BIND | J2-6 (GND J2-5) |
| Back (4 buttons only) | PC02 | protocol jumper 2 | J1-4 (GND J1-3) |
| LED red | PA00 | servo 5 | J9-6, through 150 Ω |
| LED green | PB04 | servo 2 | J10-5, through 150 Ω |
| LED common cathode | — | GND | any GND |

- **Check the header pins on your board** against `hardware/rf_module/Multi_Mode.pdf` before soldering: the
  table was read from the schematic.
- **Take the display supply from 3.3 V**, never from the servo headers: their middle pins carry the battery
  voltage.
- **The display needs pull-ups** on SDA and SCK (most 0.42" modules have them on board). The firmware finds the
  panel by them: at power-up it reads PA08 / PA09 with the chip's pull-downs, and only when both read high does it
  try the display. Without pull-ups the module simply runs as before.
- SDA and SCK swapped is found and worked around (the log says so).
- Unchanged: J3 serial (PA06 / PA05), PPM in / telemetry out (PB01), buzzer (PD02), the RA-01 (459 MHz), LED_A /
  LED_B, RANGE and F/S (F/S is ignored with a panel).

## 3. Build options (`firmware/panel_config.h`)

Set before building (`.\build.ps1`); the build prints the two settings.

| Setting | Values |
|---|---|
| `PANEL_INPUT` | `PANEL_IN_MULTI_PPM` (default): MULTI frames or PPM, whichever comes first, with an Input item in the menu · `PANEL_IN_MULTI`: MULTI frames on J3 RX only (EdgeTX radios, MULTI encoders), no PPM · `PANEL_IN_PPM`: PPM on the PPM input (PB01) only, J3 RX is never listened to for MULTI |
| `PANEL_BUTTONS` | `3`: Up, Down, Enter/Bind; the menu ends with a Back line · `4`: + Back (PC02), which leaves a page or the menu; no Back line |

A board without the display runs as before, whatever is set here.

## 4. Using it

### Power-up

- The display shows `MiTo TX` and the panel version; the LED flashes yellow.
- With a panel the module is always a transmitter: a stored receiver setup, F/S and the bind plug are ignored.
  Nothing stored yet: RLINK is stored and used.
- **Enter held at power-up = BIND held**, like the BIND jumper, until it is released.

### Home page

| Line | Shows |
|---|---|
| 1 | the mode (`RLINK`, `D16v2 EU`, ...), `+UK` when the 459 MHz link runs beside it |
| 2 | the input (`MULTI`, `PPM`, `No input`) and `Sngl` / `Dual` |
| 3 | the link: `LQ` and RSSI with telemetry, otherwise the transmit power |
| 4 | what needs attention: `BIND`, `RANGE CHECK`, `LINK LOST`, `RX BATT`, otherwise the receiver battery |

### Menu

Enter on the home page opens the menu. Up / Down move, Enter chooses. 15 s without a key returns to the home page.

| Item | |
|---|---|
| **Mode** | Up / Down choose the mode; Enter held 2 s stores it and restarts; a short Enter cancels |
| **Opt** | Single / Dual (the 459 MHz link beside the 2.4 GHz one), stored the same way |
| **Input** | `PANEL_IN_MULTI_PPM` builds only: Auto (MULTI or PPM) / PPM only, stored the same way |
| **Bind** | Enter held 2 s: restart and bind for 8 s |
| **Range** | range check on / off at once, no restart |
| **Info** | panel version, Prot / Sub, whether an SX127x (RA-01) answered |
| **Back** | 3-button builds: back to the home page |

With MULTI frames the radio chooses the mode: Mode and Opt then show `Set by the radio`, while Bind and Range
still work from the panel.

### LED

| | |
|---|---|
| green steady | linked (RLINK telemetry, FrSky link) or sending (modes without telemetry) |
| yellow steady | weak link |
| red blinking | link lost, receiver battery low, or a mode the module does not have |
| red / green alternating fast | binding |
| yellow slow blink | range check |
| green short blink every second | no input |
| yellow short flash | power-up |

## 5. The three versions

### MULTI + PPM (`PANEL_IN_MULTI_PPM`)

For a module that is fed sometimes by an EdgeTX radio or encoder (MULTI frames) and sometimes by PPM.

1. Connect the radio or encoder: MULTI on J3 RX, or PPM on the PPM input (or on J3 RX).
2. Power up. Whichever input comes first is used.
3. With PPM: choose the mode under **Mode**, Single / Dual under **Opt**.
4. To ignore MULTI frames altogether, set **Input** to `PPM only`.

### MULTI only (`PANEL_IN_MULTI`)

For a module in an EdgeTX radio bay, or behind an encoder that sends MULTI frames.

1. Connect MULTI to J3 RX and power up.
2. The radio chooses the mode; the panel shows it. **Bind**, **Range** and **Info** work from the panel.

### PPM only (`PANEL_IN_PPM`)

For a transmitter with a PPM encoder only (a vintage set, for example). The MULTI part of the firmware is never
used, and the menu has no Input item.

1. Connect PPM to the PPM input (J6, PB01) and power up.
2. Choose the mode under **Mode** and Single / Dual under **Opt**; each is stored and the module restarts.
3. Bind: **Bind** in the menu (Enter held 2 s), or Enter held at power-up.

## 6. The log

The log UART (as in `docs/QUICK_SHEET_EFR32_Module.md`) says what the panel found: `PANEL display at 0x3C: TX
panel ...`, `PANEL no display (PA08 / PA09 not pulled up): the module as before`, and a line for every setting
stored.
