# RF module carrier board (Multi_Mode)

Carrier for the RF-BM-MG24B2I (EFR32MG24) module with the header for the RA-01 (SX1278, 459 MHz) radio.
Both board variants are this one PCB:

| Variant | Fit | Leave off |
|---|---|---|
| 2.4 GHz only | everything except the RA-01 | the RA-01 and its reset capacitor |
| 2.4 GHz + 459 MHz | everything | — |

The firmware is the same for both; it detects the RA-01 at power-up.

## Files

| | |
|---|---|
| `Multi_Mode.pdf` | the schematic |
| `kicad/` | the KiCad project (KiCad 5 format: `.pro`, `.sch`, `.kicad_pcb`; KiCad 6 and later open and convert it). The EFR32 module's symbol comes from the project library `rf_star`; its footprint is embedded in the board file |
| `gerber/` | Gerber and drill files (4 layers), and the same as one zip for the board house |
| `bom/` | `Multi_Mode-top-pos.csv` — component placement, top side |

## Connections (EFR32 pins)

| Function | Pin |
|---|---|
| J3 serial: RX in / TX out, GND | PA06 / PA05 (3.3 V, 1 k in series on RX) |
| PPM in, and telemetry out (to the encoder, or EdgeTX bay pin 5) | PB01, through the Schottky diode D3 (see below) |
| BIND, RANGE, F/S (jumpers / buttons to GND) | PB00, PB02, PB03 |
| Protocol jumpers PC00, PC01, PC02 | to GND |
| LED_A, LED_B | PC04, PC05 |
| Buzzer (passive) | PD02 |
| Receiver servo outputs 1–8 | PA07 PB04 PD04 PD05 PA00 PA08 PA09 PB05 |
| Receiver PPM out, SBUS out | PC03 (270 Ω series), PA03 |
| Receiver battery sense (47 k / 10 k divider, 100 nF) | PA04 |
| RA-01: SCK, MISO, MOSI, NSS, RESET | PC06, PC07, PC08, PC09, PD03 (DIO pins unconnected) |
| SWD (flashing) | SWCLK, SWDIO, GND, 3.3 V |

Supply: 3.3 V LDO on board; mind its heat above ~8 V input with both radios sending.

## The PPM line and D3

PB01 started as a PPM input only. The Schottky diode D3 between the PPM connector and PB01 (net PPM_IN) suits
that: the outside can pull the line low, the module never drives it. Since then the same line also carries
telemetry **out** of the module, to the MiTo encoder and to bay pin 5 (S.Port) of an EdgeTX radio. D3 blocks that
direction, so the radio sees no status, no protocol list and no sensors (a black-and-white radio then cannot even
reach the MiTo protocol).

- **Boards built so far:** on a transmitter module, bridge D3 with a solder blob or a short wire. A receiver does
  not use this line and needs nothing.
- **Next revision:** a 1 k resistor in parallel with D3. PPM still comes in through the diode, telemetry goes out
  through the resistor, and no bridge is needed.

The line keeps its TVS diode either way. The MiTo encoder and EdgeTX radios drive it at 3.3 V; with a radio that
drives 5 V, the 1 k (rather than a plain bridge) also limits the current into the EFR32 pin.
