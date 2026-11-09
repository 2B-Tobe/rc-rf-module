# EFR32 RF module — quick sheet

One board, two roles. As a **transmitter module** it takes the transmitter's channels (serial frames or PPM) and
sends them on 2.4 GHz (nRF24, RLINK, FrSky D8, FrSky D16, DSMX) with the 459 MHz link beside it if the
SX1278 radio is fitted. As a **receiver** it gives 8 servo outputs, PPM and SBUS, and with the 459 MHz radio
fitted it listens on both bands and hands the servos to 459 MHz when 2.4 GHz goes quiet.

## 1. Connections

| Pin | Function |
|---|---|
| **J3 RX** PA06 | serial frames in (Multiprotocol format, 100000 8E2) — or the `cfg` command line in (115200 8N1) |
| **J3 TX** PA05 | log and `cfg` answers out, 115200 8N1 (PuTTY, `cfg.ps1`, the Controller box) |
| PB01 | PPM in (transmitter, PPM input) |
| **BIND** PB00 | jumper / button to GND |
| **RANGE** PB02 | jumper to GND: range check (low power) |
| **F/S** PB03 | jumper to GND: role / input select, see 2 |
| PC00 PC01 PC02 | protocol jumpers to GND, see 3 (only when nothing is stored) |
| LED_A PC04, LED_B PC05 | status, see 7 |
| PD02 | buzzer (RLINK transmitter alarms) |
| Receiver: servos 1–8 | PA07 PB04 PD04 PD05 PA00 PA08 PA09 PB05 (dual board) |
| Receiver: PPM out PC03, SBUS out PA03 | 8 channels, 22.5 ms / SBUS |
| Receiver: PA04 | battery sense (RLINK: 47 k / 10 k divider), reported to the transmitter |

Channel order is passed through as sent: servo 1 = channel 1 (with a TAER transmitter: throttle).

## 2. Role and input: what decides

**Stored setup** (written once over J3 with the Controller box or `cfg`, kept through firmware updates):

| Stored | F/S open | F/S fitted |
|---|---|---|
| **TX** | PPM input, stored protocol and Single/Dual | serial input (the transmitter's frames decide protocol, bind, range) |
| **RX** | stored protocol | protocol from the PC00–PC02 jumpers |

**Nothing stored** (jumpers only): F/S fitted = receiver, F/S open = transmitter; RANGE fitted at power-up
on a transmitter = serial input; PC00–PC02 = protocol. Both ways, BIND and RANGE keep their meaning
(bind at power-up, range check). All jumpers are read at power-up only.

## 3. Modes

| Mode | Serial Prot / Sub | `cfg` word | Jumpers (nothing stored) | TX | RX |
|---|---|---|---|---|---|
| nRF24 | 0 / 0 | `nrf24` | none | yes | yes |
| RLINK | 0 / 1 | `rlink` | PC02 | yes | yes |
| UK 459 MHz only | 0 / 2 | `uk` | TX: none (serial or stored only); RX: PC01 + PC02 + F/S | 2.4 GHz off | yes |
| FrSky D8 | 3 / any | `d8` | PC00 | yes | yes |
| FrSky D16 | 15 / 0–3, 5 (table below) | `d16` (Sub 0), or `15 2` etc. | PC01 (v1 EU-LBT) | yes | yes |
| DSMX 22 ms | 6 / 2 | `dsmx` | PC00 + PC01 | yes | yes |
| Jeti capture (tool) | 0 / 6 | `jeti` | PC00 + PC02 | log only | log only |
| DSM capture (tool) | 0 / 4 | `dsmcap` | PC00 + PC01 + PC02 | — | log only |
| TX test (carrier) | 0 / 7 | `txtest` | PC00 + PC01 + PC02 + F/S | yes | — |

**FrSky D16 versions** (the Sub picks the one the receiver's firmware speaks; a FrSky radio that also offers D8 is
FCC, one with D16 only is EU-LBT). The module's own receiver follows the same Sub.

| Sub | Version | Radio | Bench-tested with |
|---|---|---|---|
| 0 | ACCST D16 v1 EU-LBT | 2405 MHz, 100 kbit/s 2-FSK, listen-before-talk | EFR32 ↔ EFR32 |
| 1 | ACCST D16 v2 EU-LBT | as Sub 0, v2 packets | — |
| 2 | ACCST D16 v1 FCC | 2404 MHz, 70 kbit/s GFSK | FrSky RX8R, RadioMaster R88; receiver: Taranis Q X7 |
| 3 | ACCST D16 v2 FCC | 2404 MHz, 77 kbit/s GFSK | — |
| 5 | ACCST D16 v1 FCC, channels 1–8 only | as Sub 2 | — |

**Single / Dual** (transmitter with a 2.4 GHz link): Single = 2.4 GHz only, Dual = 459 MHz link beside it.
From a transmitter's serial frames: the option byte 0 / 1. PPM input: the stored setup's option. A receiver is dual
whenever its 459 MHz radio is fitted. A mode the module does not have: RF off, both LEDs alternate.

## 4. Setting the module up over J3

Wire J3 RX, J3 TX and GND to the Controller box (its PA9 / PA8 / GND) or to a USB-serial adapter for
`cfg.ps1`. The module answers while it is not being fed serial frames: a receiver, a transmitter on PPM
input, a transmitter waiting for its first frame. A transmitter with input present refuses (LEDs alternate fast).

| Command | Does | Answer |
|---|---|---|
| `cfg` | show stored setup and what runs now | `CFG stored: ...` / `CFG nothing stored ...`, `CFG running now: ...` |
| `cfg rx <mode>` | store receiver + mode, restart | `CFG stored: RX Prot n Sub n = ...`, `CFG restarting` |
| `cfg tx <mode> [dual\|single]` | store transmitter + mode (+ option), restart | `CFG stored: TX ... Opt dual` |
| `cfg bind` | restart with BIND counted as held for 8 s | `CFG bind: restarting ...` |
| `cfg clear` | erase the setup: jumpers decide again | `CFG cleared ...` |

`<mode>` = the `cfg` word from table 3, or the numbers `Prot Sub` (e.g. `cfg rx 0 1`).
LEDs: accepted = both blink together 3×; refused = alternate fast for 2 s.
From PowerShell (PuTTY closed): `.\cfg.ps1 -Port COM7 -Command "cfg rx rlink"`; `.\cfg.ps1 -Port COM7` shows the log.

## 5. Transmitter

**Serial input** (stored TX + F/S fitted, or nothing stored + RANGE fitted at power-up): the transmitter's
serial output → J3 RX, GND; Multiprotocol frames (docs/SERIAL_INPUT.md): protocol / sub (table 3), RX number
= model number (per-model binding), option (Single/Dual). The module waits for frames (both LEDs blink together), starts the mode asked
for, restarts by itself when Prot / Sub / Opt change. The frames' **bind** flag = bind for as long as it is
set; **range** flag = range check; a **failsafe frame** (header 0x57) is passed on to the receiver, which
stores those positions (RLINK, nRF24). No frames = input lost.

**PPM input** (stored TX + F/S open, or nothing stored + no RANGE at power-up): PPM on PB01, protocol from
the stored setup or the PC jumpers.
- **Bind:** BIND fitted at power-up → bind for 1.5 s, longer while BIND stays fitted. nRF24 and the 459 MHz
  link bind their first 1.5 s at every power-up anyway. Receiver in bind at the same time (section 6).
- **Range check:** RANGE fitted → −10 dBm on 2.4 GHz and low power on 459 MHz; walk 30–50 m with full control.
- Input lost (no PPM): the receiver goes to failsafe.

**RLINK transmitter alarms** (buzzer PD02, LED_A): two beeps/s = weak link; three beeps/s = receiver battery
low; continuous 150 ms beeps = land now (link critical, receiver in failsafe or battery critical); one chirp
every 2 s = no receiver heard since power-up. PA07 is high while an alarm is on.

## 6. Receiver

**Power order in use:** transmitter on first, then the receiver (the receiver without its transmitter sits in
failsafe / searching).

**Bind (once per model, per mode):**
1. Transmitter on and in bind: BIND fitted at power-up (PPM input), or the bind flag in its serial frames.
2. Receiver into bind: **BIND fitted at power-up**, or `cfg bind` from the Controller box or over the serial
   lead. LED_B fast-blinks.
   It listens 8 s, then falls back to its stored bind (no stored bind: keeps listening).
3. Link up (LED_B steady) → transmitter bind off, BIND jumper off, power-cycle the receiver: it comes back
   linked. Each mode keeps its own bind; an earlier bind in another mode still holds.
   A receiver that links while BIND is still held also stores that moment's positions as failsafe.
With the 459 MHz radio fitted, BIND binds both links at the same time.

**Failsafe preset:** sticks and switches where the model should go, then
- on the receiver: hold **BIND 0.3 s while linked** (every receiver mode), or
- from the transmitter: a failsafe frame on the serial input (docs/SERIAL_INPUT.md 2.2; RLINK, nRF24).
Link lost (~1 s; nRF24 0.44 s; 459 MHz 1.2 s) → the preset; no preset → the servos hold the last positions.
Link back → normal at once.

**Dual receiver:** servos, PPM and SBUS follow 2.4 GHz; 100 ms without a valid 2.4 GHz packet and fresh
459 MHz data → servos from 459 MHz (log `DUAL servos from 459 MHz`); first 2.4 GHz packet → back.

## 7. LEDs

| Situation | LED_A | LED_B |
|---|---|---|
| Power-up self-test | 3 fast double blinks | with LED_A |
| Transmitter waiting for serial frames | both blink together, slowly | |
| Mode not in this module / 459 MHz radio asked for but not answering | both alternate, 2 Hz | |
| `cfg` accepted / refused | together 3× / alternate fast 2 s | |
| Transmitter, input frames | — | toggles per input frame (heartbeat) |
| RLINK transmitter | steady = link fine; 2 Hz blink = warning; flicker = no telemetry | heartbeat |
| RLINK receiver | toggles per received frame | steady = quality ≥ 95, blinks faster the worse the link; 1 Hz = searching; fast = bind |
| nRF24 receiver | toggles per 10 packets, off without link | steady = link; slow blink = search; fast = bind |
| FrSky transmitter / receiver | TX: steady when telemetry heard; RX: toggles per packet, off = no link | fast = bind; steady = data / live link |
| UK only transmitter | input present | toggles per 459 MHz packet |

## 8. Log (J3 TX, 115200 8N1)

`BOOT:` and `BUILD dual-serial` at power-up; `SETUP stored (cfg): ...` or `JUMPERS ... -> proto n`; transmitter
in serial: `BAY ...` every 5 s (ends with `jump n/worst µs`: must stay 0 with the sticks still); RLINK receiver:
`RL rx link|search|bind q.. a.. b.. lost.. fs.. v..` every 2 s (q = link quality, v = pack volts); any receiver:
`SO f<frames> on|459|off late../.. jump../..` every 2 s; 459 MHz: `UK ...` / `UR ...`; `RL failsafe stored:`
when a preset is taken; `CFG ...` answers.

## 9. Firmware update

```
cd C:\dev\rc-rf-module\firmware
.\build.ps1
python -m pyocd flash -t efr32mg24a420f1536im48 C:\dev\rc-rf-module\firmware\cmake_gcc\build\base\rc_module_trx.hex
```
Flash must end `identical 0 bytes`. The stored setup and the binds survive the update.
