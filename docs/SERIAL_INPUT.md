# Feeding and setting up the module: PPM, serial frames, the `cfg` commands

What a transmitter (or a programmer) has to send. Pin names are the EFR32 pins; the carrier board's headers
are in `hardware/rf_module/`.

## 1. PPM input (PB01)

Standard PPM: 8 channels, falling edges timestamped, a gap of more than 3.5 ms is the frame sync. Pulse
widths 1000–2000 µs (1500 centre), any frame length (the usual 22.5 ms). Channel order is passed through
as received; the receiver's servo 1 is the first PPM channel. No PPM for 100 ms = input lost → the
receiver goes to failsafe.

Which protocol runs: the stored setup (section 3) or the PC00–PC02 jumpers. BIND fitted at power-up =
bind; RANGE fitted = range check (low power); F/S open = transmitter.

## 2. Serial input (PA06, "J3 RX"): the Multiprotocol frame

One frame every 22 ms, **100000 baud, 8 data bits, even parity, 2 stop bits, not inverted**, 3.3 V. This is the serial format of the DIY Multiprotocol TX Module, so a
transmitter that can drive that module can drive this one.

| Byte | Content |
|---|---|
| 0 | `0x55` (protocol bit 5 clear) or `0x54` (protocol bit 5 set); `0x57` / `0x56` = the same frame as a **failsafe frame** (section 2.2) |
| 1 | protocol bits 0–4, `0x20` range check, `0x40` autobind (ignored), `0x80` **bind** |
| 2 | RX number bits 0–3 \| sub-protocol << 4 (bits 4–6) \| `0x80` low power (ignored) |
| 3 | option, signed (Single / Dual for the 459 MHz link: 0 = single, 1 = dual) |
| 4–25 | 16 channels × 11 bits, little-endian bit stream; 204 = 1000 µs, 1024 = 1500 µs, 1844 = 2000 µs |
| 26 | only for protocols 64 and up (27-byte frame): protocol bits 6–7 in bits 6–7 (this module refuses those) |

Protocol / sub-protocol numbers the module answers to:

| Mode | Protocol | Sub |
|---|---|---|
| nRF24 | 0 | 0 |
| RLINK | 0 | 1 |
| UK 459 MHz only (2.4 GHz off) | 0 | 2 |
| DSM capture tool (receiver) | 0 | 4 |
| Jeti capture tool | 0 | 6 |
| TX test (carrier) | 0 | 7 |
| FrSky D8 | 3 | any |
| FrSky D16 | 15 | 0 = v1 EU-LBT, 1 = v2 EU-LBT, 2 = v1 FCC, 3 = v2 FCC, 5 = v1 FCC channels 1–8 only |
| DSMX 22 ms | 6 | 2 |

Anything else: RF off, both LEDs alternate, the log says why.

**RX number** (byte 2 bits 0–3) is mixed into every transmitter ID (2.4 GHz and 459 MHz): a receiver answers
only the number it was bound with. Send the model number there for per-model binding, or 0 for one ID.

The module starts the mode of the first frame and restarts by itself when protocol, sub-protocol or option
change. **Bind** (byte 1 bit 7) keeps the transmitter binding for as long as it is set (FrSky in 1.5 s turns
with normal data, DSMX one burst). **Range check** (bit 5) lowers the power while set. No frames for 100 ms
= input lost. Only the first 8 channels are used.

When the module takes serial frames, the BIND pin still works beside them; the range check comes from the
frames' flag, not from the RANGE pin (with nothing stored, RANGE fitted at power-up is what selects the
serial input).

### 2.2 Failsafe frame

A frame with header `0x57` (or `0x56` for protocols 32–63) carries the **failsafe positions** instead of
stick positions: the module raises the "store failsafe" flag in its link for 300 ms after each such frame
and the receiver stores the channels of that moment as its failsafe preset (RLINK, nRF24). Send one or a
few such frames with the sticks in the failsafe position, then normal frames again.

### 2.3 An EdgeTX radio (from 1.4)

The module also works as a MULTI (4-in-1) module in the JR bay of a stock EdgeTX radio (tested: FrSky Taranis Q
X7). It is the same firmware: the module tells the radios apart by itself, with nothing to set.

| JR bay pin | | module |
|---|---|---|
| 1 | MULTI frames | J3 RX (PA06) |
| 2 | battery + | supply |
| 3 | GND | GND |
| 4 | not used | — |
| 5 | S.Port, telemetry | PB01, the PPM pin (bridge the Schottky diode D3 on current boards; the next revision has a 1 k in parallel with it: `hardware/rf_module/README.md`) |

- **Telling the radios apart:** EdgeTX sends the same frames inverted and every 7 ms; an encoder sends them
  normal, every 22 ms. The module looks both ways up and at the frame rate.
- **Setting the module up:** in the radio, set Internal RF to OFF and External RF to MULTI. A board stored as a
  transmitter works straight away. A board stored as a receiver notices the radio's frames on J3 RX at start-up,
  stores the transmitter setup by itself (single / dual kept) and restarts once as the transmitter module.
- **The protocol list:** the module answers the radio's status and protocol-list requests, using the Multiprotocol
  numbers, so models made for a real 4-in-1 carry over:
  - FrSkyD (3) D8
  - DSM (6) DSMX_1F / DSMX_2F
  - FrSkyX (15) CH_16 / CH_8 / EU_16
  - FrSkyX2 (64) CH_16 / EU_16
  - **MiTo (120)** nRF24 / RLINK / UK459
  - Anything else shows "Protocol invalid".
- **Single or dual band** (the 459 MHz link beside 2.4 GHz, where the RA-01 is fitted), set per model:
  - MiTo: Option value 0 = single, any other value = dual.
  - FrSkyD, FrSkyX, FrSkyX2: Option value 1 = dual, anything else single.
  - DSM: EdgeTX fills the option byte itself, so its **Max throw** box switches dual on.
  - Dual only helps a receiver that is a MiTo board with the RA-01. A FrSky or Spektrum receiver hears
    2.4 GHz only.
- **Black-and-white radios:** they can't read the list, but they take the module's names and next/previous
  protocol from its status, so MiTo is reachable once telemetry arrives on pin 5.
- **Telemetry:**
  - D8: RSSI and A1.
  - D16 and RLINK: RSSI, TRSS, TQly and RxBt (use "Discover new"). RLINK's RSSI is the receiver's dBm + 130, so
    EdgeTX's 45 / 42 alarms fall at about −85 / −88 dBm.
  - nRF24 and DSMX: status only.
- **Failsafe:** EdgeTX's failsafe frames are ignored, and the module tells the radio it has no failsafe support.
  EdgeTX sends one every 7 s, and the receivers here would store the stick positions of that moment. Set failsafe
  with the module's own method.

## 3. The `cfg` command (PA06 in / PA05 out, 115200 8N1)

The module's setup — role, protocol, option — can be stored in the module so that no protocol jumpers are
needed. Text lines, ended with CR or LF; answers are lines beginning `CFG`. The module listens whenever it
is not fed by serial frames: a receiver, a transmitter on PPM, or a transmitter waiting for its first frame.
A transmitter with input present answers `CFG refused ...` and changes nothing.

| Command | Effect | Answers |
|---|---|---|
| `cfg` | show | `CFG stored: RX Prot 0 Sub 1 = RLINK receiver` or `CFG nothing stored: ...`, then `CFG running now: <mode>, input <radio|PPM|encoder serial>, F/S <high|low> (...)` |
| `cfg rx <mode>` | store receiver + mode, restart | `CFG stored: ...`, `CFG restarting with it (...)` |
| `cfg tx <mode> [dual\|single]` | store transmitter + mode + option, restart | `CFG stored: TX Prot .. Sub .. = .., Opt dual`, `CFG restarting ...` |
| `cfg bind` | restart with BIND counted as held for 8 s | `CFG bind: restarting with BIND held for 8 s` |
| `cfg clear` | erase the setup, restart | `CFG cleared: the jumpers decide again. Restarting` |

`<mode>`: `nrf24 rlink uk d8 d16 dsmx jeti txtest dsmcap`, or the two numbers `Prot Sub` from the table
above (`cfg rx 0 1`). Errors: `CFG unknown protocol '...'`, `CFG Prot n Sub n is not a receiver mode of
this module`, `CFG usage: ...`. The LEDs confirm: both together 3× = accepted, alternating fast 2 s =
refused.

What the stored setup does at power-up:

| Stored | F/S open | F/S fitted |
|---|---|---|
| TX | PPM input, stored mode and option | serial frames decide (section 2) |
| RX | stored mode | PC00–PC02 jumpers decide |

The Controller in this repository is one implementation of a programmer: `controller/Core/Src/link.c`
shows how it sends the commands and reads the answers; the same can be done from a transmitter's own
firmware over the lead that carries the serial frames: switch the UART to 115200 8N1, send `\rcfg rx 0 1\r`,
switch back to the frames. A transmitter module that is being fed frames ignores the text; a receiver on
the same lead takes it.

## 4. The log (PA05, 115200 8N1)

Plain text, one line per event or status: `BOOT:`, `SETUP ...` / `JUMPERS ...` at power-up; `BAY ...` (serial
input statistics), `RL ...` (RLINK), `NR ...` (nRF24 receiver), `UK ...` / `UR ...` (459 MHz), `SO ...`
(servo output), `DUAL servos ...`, `CFG ...`. Nothing has to be answered.
