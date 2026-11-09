# RF Module Controller — quick sheet

Sets up an EFR32 RF module without jumpers: role (TX / RX), protocol, Single / Dual, and binds a receiver.
The setting lives in the module; the box writes it and reads it back.

## 1. Wiring

| Box | Module J3 |
|---|---|
| PA9 | RX (PA06) |
| PA8 | TX (PA05) |
| GND | GND |

Module powered as usual, box from its own supply, nothing else on the module's J3. A transmitter module
that is being fed by its encoder cannot be changed (take the encoder lead off J3 RX first); one running
on PPM input refuses while PPM is present.

## 2. Keys

| Key | |
|---|---|
| INC / DEC | cursor up / down |
| ENTER on Role / Proto / Opt | edit (value shown reversed) → INC / DEC change → ENTER done |
| ENTER on **Send** | store the setup in the module (it restarts) |
| ENTER on **Bind**, or **AUX** | the module restarts into an 8 s bind window |
| BACK | read the module |
| BACK held 2 s | clear the module's setup (jumpers decide again) |

Any key leaves an answer page; it leaves by itself after 8 s.

## 3. Screen

```
RX RLINK              what the module has stored (TX adds D / S; "RF MODULE SETUP" = nothing)
▌ Role   RX
  Proto  RLINK
  Opt    Dual         transmitter: 459 MHz link beside 2.4 GHz
  Send     Bind
```
Receiver: row 4 is **Out** = 8 PWM / 5+SBUS (SBUS on connector 6) / 5+PPM (PPM on connector 7) / 5+S+P (both).
After Send: `Stored and running` / what is stored / Opt / `Now:` what runs / where the channels come from
(`radio`, `PPM`, `encoder serial`) and the F/S jumper state.

## 4. Set up a receiver

1. Wire, power both. Role → **RX**, Proto → the mode, Send. Module LEDs blink together 3×, it restarts,
   box shows `Stored and running`.
2. Power off, remove all protocol jumpers, power on: it runs the stored mode (F/S open).
3. Bind once (section 6).

## 5. Set up a transmitter module

1. Module out of the transmitter, wired, powered. Role → **TX**, Proto, Opt → **Dual** or **Single**, Send.
2. Jumpers afterwards: **F/S fitted** = serial input (the transmitter's frames decide; RANGE is only the
   range check now). **F/S open** = PPM input with the stored mode and Opt.

## 6. Bind a receiver from the box

1. Transmitter on the same mode, in bind (bind flag in its serial frames, or BIND at power-up on PPM input).
2. Box: **Bind** (or AUX). Page: `Bind: 8 s window`. Receiver LEDs blink together 3×, it restarts, LED_B
   fast-blinks, then the link comes up.
3. Transmitter bind off. Power-cycle the receiver: it comes back linked.

If the link comes up inside the 8 s, the receiver also stores that moment's positions as failsafe.

## 7. Modes on the box

| Proto | TX | RX | Module Prot / Sub |
|---|---|---|---|
| nRF24 | yes | yes | 0 / 0 |
| RLINK | yes | yes | 0 / 1 |
| UK-459 | 459 MHz only, 2.4 GHz off | 459 MHz receiver | 0 / 2 |
| FrSky D8 | yes | yes | 3 / 0 |
| D16v1 EU | D16 v1 EU-LBT | yes | 15 / 0 |
| D16v2 EU | D16 v2 EU-LBT | yes | 15 / 1 |
| D16v1 FCC | D16 v1 FCC | yes | 15 / 2 |
| D16v2 FCC | D16 v2 FCC | yes | 15 / 3 |
| D16 FCC8 | D16 v1 FCC, ch 1–8 | yes | 15 / 5 |
| DSMX | yes (22 ms) | yes | 6 / 2 |
| Jeti cap | tool | tool | 0 / 6 |
| TX test | carrier | — | 0 / 7 |
| DSM cap | — | tool | 0 / 4 |

The list skips what the chosen role cannot do. Opt applies to transmitter modes with a 2.4 GHz link, Out to receivers.
The box stores these numbers, the same Prot / Sub / Opt as a transmitter's serial frames: a receiver set up here
matches a transmitter on the same Prot / Sub (FrSky D16: same version on both ends; module firmware 1.3 or later).

## 8. If it does not work

| Shows | Do |
|---|---|
| Row 1 `RF MODULE SETUP` with a module connected | BACK: the page says what it found |
| `No answer on J3` / `nothing received` | check PA8 → J3 TX and GND; encoder lead on J3 RX? (take it off) |
| `No answer on J3` with characters and `bytes N bad M` | garbled: check the wire; else build with the answer on PA10 or PA3 (`serial.h`) |
| Module LEDs alternate fast after Send | refused — a mode the role does not have, or a transmitter in use; the page shows why |
| Receiver does not link after Bind | the transmitter was not in bind during the 8 s, or another mode. Repeat 6 |
| Blank display | display cable (contrast is fixed at 180) |
