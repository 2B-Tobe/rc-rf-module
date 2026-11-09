# RF Module Controller — instruction manual

The Controller is a small hand-held box that sets up the EFR32 RF module
(rc_module_trx firmware) without any protocol jumpers: which role the
module has (transmitter or receiver), which protocol it runs, whether the
459 MHz link runs beside the 2.4 GHz one, and it binds a receiver without
the BIND jumper.

The setting is kept in the module itself (its own flash page, untouched
by firmware updates). The box only writes it and reads it back.

---

## 1. Wiring

| Box | Module J3 | Carries |
|---|---|---|
| PA9 | **RX** (PA06) | commands to the module |
| PA8 | **TX** (PA05) | the module's answer |
| GND | GND | |

Both sides 3.3 V logic. The module is powered as usual, the box from its
own supply. Nothing else is connected to the module's J3 while the box is.

The module listens to the box whenever it is not receiving encoder
frames: a receiver while it runs, a transmitter with PPM input while it
runs, and a transmitter in serial mode while it waits for the encoder.
A transmitter that is fed by its encoder cannot be changed at all; one
that is flying its PPM input refuses changes. So nothing on the bench can
re-program a module that is in use.

## 2. Keys

| Key | Function |
|---|---|
| **INC / DEC** | move the cursor up / down |
| **ENTER** on Role, Proto or Opt | start editing: the value is shown reversed. INC / DEC change it, ENTER finishes |
| **ENTER** on **Send** | store the setup in the module |
| **ENTER** on **Bind** | the module restarts and binds (BIND counted as held for 8 s) |
| **BACK** | read the module and show its answer |
| **BACK held 2 s** | clear the module's setup: it goes back to deciding by jumpers |
| **AUX** (extra key) | the same as Bind |

Any key leaves an answer page; it also leaves by itself after 8 s.

## 3. Screen

```
RX RLINK             row 1: what the module has stored (read at power-up;
▌ Role   RX                 a transmitter adds D / S for Dual / Single);
                            "RF MODULE SETUP" if nothing is stored
  Proto  RLINK
  Opt    Dual        transmitter: the 459 MHz link beside the 2.4 GHz one
  Send     Bind      two cursor positions
```

For a receiver row 4 is **Out**, its output layout (module firmware with
receiver output layouts, branch `rx-setup` and later):

| Out | Connectors 1–5 | Connector 6 | Connector 7 | Connector 8 |
|---|---|---|---|---|
| **8 PWM** | ch 1–5 | ch 6 | ch 7 | ch 8 |
| **5+SBUS** | ch 1–5 | SBUS | ch 7 | ch 8 |
| **5+PPM** | ch 1–5 | ch 6 | PPM | ch 8 |
| **5+S+P** | ch 1–5 | SBUS | PPM | ch 8 |

SBUS comes out inverted, as receivers send it; PPM as on the encoders.
Row 1 shows a receiver's layout as `+S`, `+P` or `+SP` after its mode
(nothing for 8 PWM). Older module firmware ignores the choice.

At power-up the box reads the module and presets Role, Proto and Opt to
what is stored, so after a power-up the menu shows the module's current
setup and row 1 confirms it.

Answer page after Send:

```
Stored and running
RX RLINK
                     (Opt dual / single for a transmitter, Out ... for a receiver)
Now: RLINK receiver
radio F/S high
```

`Now:` is what the module runs at this moment, the last row where its
channels come from (radio / PPM / encoder serial) and the state of its
F/S jumper.

## 4. Setting up a receiver

1. Wire the box to the receiver (section 1), power both.
2. Cursor on **Role**, ENTER, INC / DEC until `RX`, ENTER.
3. Cursor on **Proto**, ENTER, INC / DEC to the protocol, ENTER.
4. Cursor on **Send**, ENTER. The module's two LEDs blink together three
   times and it restarts; the box shows `Stored and running`.
5. Power the receiver off, remove all its jumpers, power on: it runs the
   stored protocol. A bind is still needed once (section 5).

To change the protocol later, repeat from step 3. Each protocol keeps its
own bind in the receiver, so going back to a protocol it was bound in
before needs no new bind.

## 5. Binding from the box

1. Transmitter on the same protocol, with its bind selected (the bind flag
   in its serial frames, or BIND at power-up on PPM input; it keeps sending
   bind packets while selected).
2. Box: cursor on **Bind**, ENTER. The page says `Bind: 8 s window`.
3. Receiver: LEDs blink together three times, it restarts, LED_B
   fast-blinks while binding, then the link comes up.
4. Transmitter: bind off.
5. Power-cycle the receiver: it comes back linked, no jumpers anywhere.

If the link comes up inside the 8 s, the receiver also stores the stick
positions of that moment as its failsafe preset, exactly as it does with
the BIND jumper.

## 6. Setting up a transmitter

1. Module out of the transmitter, wired to the box, powered.
2. Role `TX`; Proto; Opt `Dual` (459 MHz link beside the 2.4 GHz one) or
   `Single`; Send.
3. Jumpers on the module afterwards:
   - **F/S fitted** → serial input: the transmitter's frames decide the
     protocol, as before. The RANGE jumper is not needed for serial
     any more (it is the range check).
   - **F/S open** → PPM input, the stored protocol and Opt.

## 7. Reading and clearing

- **BACK** asks the module what it has and runs now.
- **BACK held 2 s** erases the module's setup; it restarts and decides by
  its jumpers again, as it did before any setup was stored.

## 8. Protocols on the box

| Proto | Transmitter | Receiver | Module's Prot / Sub |
|---|---|---|---|
| nRF24 | yes | yes | 0 / 0 |
| RLINK | yes | yes | 0 / 1 |
| UK-459 | UK only: 459 MHz link, 2.4 GHz off | UK 459 MHz receiver | 0 / 2 |
| FrSky D8 | yes | yes | 3 / 0 |
| D16v1 EU | FrSky ACCST D16 v1 EU-LBT | yes | 15 / 0 |
| D16v2 EU | FrSky ACCST D16 v2 EU-LBT | yes | 15 / 1 |
| D16v1 FCC | FrSky ACCST D16 v1 FCC | yes | 15 / 2 |
| D16v2 FCC | FrSky ACCST D16 v2 FCC | yes | 15 / 3 |
| D16 FCC8 | FrSky ACCST D16 v1 FCC, channels 1–8 only | yes | 15 / 5 |
| DSMX | yes (22 ms) | yes | 6 / 2 |
| Jeti cap | capture tool | capture tool | 0 / 6 |
| TX test | carrier test | — | 0 / 7 |
| DSM cap | — | capture tool | 0 / 4 |

The box stores the setup as these numbers (`cfg tx 15 2 dual`, `cfg rx 15 2 1` — the last number a receiver's Out) — the same Prot / Sub / Opt a
transmitter sends in its serial frames. So a module set up from the box runs exactly as one fed serial
frames with those numbers, and a receiver set up from the box matches a transmitter on the same Prot / Sub:
for FrSky D16 the Sub is the version, and transmitter and receiver must use the same one (an FCC receiver
needs `D16v1 FCC` or `D16v2 FCC`, an EU-LBT one `D16v1 EU` or `D16v2 EU`). The D16 versions need module
firmware 1.3 or later.

The list on the box skips what the chosen role cannot do. Opt (Single /
Dual) applies to transmitter modes with a 2.4 GHz link; a receiver is
dual whenever its 459 MHz radio is fitted. Out applies to every receiver.

## 9. What the module does with the stored setup

| Stored role | F/S low at power-up | F/S high at power-up |
|---|---|---|
| TX | encoder serial control | PPM input, stored protocol and Opt |
| RX | protocol from the PC00–PC02 jumpers, as before | stored protocol |
| nothing stored | exactly as before: F/S = receiver, RANGE at power-up = serial, PC00–PC02 = protocol | |

BIND and RANGE keep their meaning in every case: bind at power-up (hold
BIND 0.3 s on a linked receiver = store failsafe), range check on a
transmitter.

## 10. Troubleshooting

- **Row 1 says `RF MODULE SETUP` with a module connected** — the box got
  no answer at power-up. Press BACK: the page says what it found.
- **`No answer on J3`, `nothing received`** — check the PA8 wire to J3 TX
  and the GND; or the module is a transmitter receiving encoder frames
  (take the encoder wire off J3 RX).
- **`No answer on J3` with characters on rows 2–3 and `bytes N bad M`** —
  bytes arrive but garbled. The PA8 receiver sets its polarity from the
  idle level and samples each bit in the middle, which copes with an
  inverting or slow input stage; if that still fails, build with the
  answer on PA10 or PA3 (`Core/Inc/serial.h`).
- **Module LEDs alternate fast after Send** — the module refused (a mode
  its role does not have, or it is a transmitter in use); the page shows
  its reason.
- **Receiver does not link after Bind** — the transmitter's bind was not
  selected during the 8 s, or it is on another protocol. Repeat section 5.
- **Blank display** — the contrast is set at 180 (JLX12864 at 3.3 V);
  check the display cable.
