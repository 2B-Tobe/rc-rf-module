# Jeti Duplex 2.4 ("quasi Jeti") — experimental

> **Experimental, for the bench.** Tested on the author's boards on 25 November 2026: this module as transmitter
> to this module as receiver (with telemetry), and a Jeti TU2 transmitter module to this module as receiver.
> **Not tested:** this module as transmitter with a Jeti receiver (none was at hand), a Jeti DS / DC radio with the
> receiver role, and whether a Jeti transmitter makes use of the receiver role's replies. Not for flying until
> that has been done. Reports with real Jeti equipment are welcome.

| | |
|---|---|
| Firmware | `jeti.c` v0.5 (firmware 1.9) |
| Protocol / Sub (serial frames, `cfg`) | 111 / 0 (111 is mpm-v2's number; Multiprotocol has no Jeti) |
| EdgeTX (External RF = MULTI) | Jeti (111), DUPLEX |
| `cfg` name | `duplex` (`jeti` is the capture tool) |
| Channels | 8 |
| Radio | the EFR32's built-in IEEE 802.15.4 2.4 GHz radio (O-QPSK 250 kbit/s, channels 11–26) |
| Transmit power | full power in flight, −10 dBm in range check |
| Telemetry | receiver voltage, a signal figure, RSSI at the transmitter, share of frames answered |

## Where it comes from

Everything known here about the Jeti Duplex protocol comes from **Raphael Coeffic's mpm-v2**
(github.com/raphaelcoeffic/mpmv2, GPL v3), who traced it and links Duplex R6 and REX10 receivers from his module:

- `src/core/jeti_frame.c`: the frame, and the scrambling of its payload;
- `src/mpm/proto_jeti.c`: the transmitter — channel values, the ring of 5 hop channels carried in every frame,
  the channel sweep, carrier sense, the bind sequence, the layout of a receiver's reply;
- `src/core/bench_jeti.c`: a follower, from which the receiver role here takes its hop clock.

The transmitter role in `jeti.c` is a port of his `proto_jeti.c` for 8 channels. The receiver role is this
project's own: mpm-v2 has no receiver. It was checked against captures of a Jeti TU2 module made with this
module's capture tool (`jeti_cap.c`): 30,206 data frames decode and scramble back to the bytes on the air.

## What it does

**As a transmitter**
- a frame with 8 channels every 20 ms, hopping over 5 channels that every frame names;
- before each frame a clear-channel check, with a random wait and another try when the channel is busy;
- one of the 5 channels moves to another one four times a second, the least answered first, so the link drifts
  away from occupied channels;
- the receiver's replies give the telemetry above: on the log, to the encoder's telemetry frame and to EdgeTX;
- no input for 100 ms: sticks centred, throttle (channel 3) low.

Not ported from mpm-v2: 12 / 16 channels (the module has 8 inputs), the remote JETIBOX, the sensors' EX data.

**As a receiver**
- searches the 16 channels for its transmitter, then follows its hops; every frame brings the 5 channels up to
  date;
- answers each frame as a Jeti receiver does (voltage every 4th reply, if the board has the battery divider on
  PA04). Two bytes of the reply are not documented and are this project's choice: the flags, and the signal
  figure (RSSI + 120; a real receiver close by sends about 0x38);
- 8 channels to the servo outputs, SBUS and PPM as every receiver role; 0.5 s without a frame: search again;
  1 s: failsafe (the stored preset, else hold). BIND pressed 0.3 s while linked stores the preset;
- RANGE fitted at power-up: no replies, the receiver only listens.

## Using it

- **From an EdgeTX radio:** External RF = MULTI, protocol Jeti → DUPLEX. Option value 1 = dual (the 459 MHz link
  beside, which only a receiver of this kind hears). The radio's Bind starts the bind sequence.
- **From the Controller (1.6) or a terminal:** `cfg tx duplex` / `cfg rx duplex`, or by number `cfg tx 111 0` /
  `cfg rx 111 0` (add `dual` for the 459 MHz link on a transmitter).
- **From the TX panel:** Mode → Jeti.
- **Binding two boards of this kind:** BIND on both (the button or jumper at power-up, `cfg bind`, or the
  Controller's Bind), about a metre apart. The transmitter stays 150 ms on each channel in turn for up to 60 s; the
  receiver answers with its address; both store the other.
- **Binding the receiver role to a Jeti transmitter module:** BIND on the receiver, then switch the transmitter
  on close by. The first transmitter at −70 dBm or stronger is taken.
- **Binding a Jeti receiver to the transmitter role (untested here):** put the receiver in pairing (bind plug, or
  the JETIBOX's Pairing), then BIND on the transmitter.

## Seen on the bench

- TU2 → receiver role, about −63 dBm: every frame received (229 in 5 s, one per PPM frame of the TU2's input), no
  resynchronisation of the hop clock, link back by itself after the TU2 was switched off and on.
- With this TU2 the channels arrive 200 µs high (1700 for 1500 at its PPM input, with two different PPM sources).
  The receiver outputs what it is sent; mpm-v2's author sees exactly 1500 from his module, so this is taken to be
  this TU2's PPM input and not the protocol. Not resolved.

## Log

Lines start with `JQ`. `JQ start 1` … `7` trace the start-up. Every 5 s a status line:
`JQ tx sent.. replies.. (answered ..%, sent ..%) busy.. late.. next.. lost.. ...` on a transmitter,
`JQ rx link frames.. other.. resync.. rssi.. replies.. ...` and the frame types and address heard on a receiver.
