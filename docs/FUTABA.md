# Futaba S-FHSS and FASST — experimental

> **Not yet tried with Futaba equipment.** Both modes have been bench-tested only between two boards of this
> kind (one as the transmitter, one as the receiver). Whether a Futaba receiver links to this module, or a
> Futaba transmitter to its receiver role, has **not** been checked here. Use them for experiments on the bench,
> not for flying, until that has been done. Reports with real Futaba gear are welcome.

| | S-FHSS | FASST 10ch |
|---|---|---|
| Firmware | `sfhss.c` v0.4 | `fasst.c` v0.3 |
| Protocol / Sub (serial frames, `cfg`) | 21 / 0 | 110 / 0 |
| EdgeTX (External RF = MULTI) | Futaba (21), SFHSS | FASST (110), 10CH |
| `cfg` name | `sfhss` | `fasst` |
| For | S-FHSS receivers (Futaba R2006GS / R2008SB and compatible) | FASST receivers (Futaba R6008HS and compatible) |
| Channels | 8 | ch1–ch8 from the module, ch9 / ch10 at centre |
| Transmit power | +10 dBm in flight, −10 dBm in range check | +10 dBm in flight, −10 dBm in range check |
| Receiver role | yes, for a transmitter of this kind (and possibly a Futaba transmitter: untested) | yes, for a transmitter of this kind only |

The power is held at +10 dBm until the modes have been proven with Futaba receivers.

## Where they come from

- **S-FHSS:** the radio settings, packet and timing a Futaba transmitter uses, from the DIY Multiprotocol TX
  Module project (`Futaba_cc2500.ino`, GPL v3, itself from Deviation's `sfhss_cc2500.c`). The CRC form was measured
  on a CC2500 by Raphael Coeffic for mpm-v2 (GPL v3). 2-FSK at 128 kbit/s on 30 channels, a 6.8 ms frame of two
  packets.
- **FASST:** everything here comes from Raphael Coeffic's mpm-v2 (github.com/raphaelcoeffic/mpmv2, GPL v3), who
  traced it from a Futaba TM-10 module driving an R6008HS. The framing in `fasst.c` is a port of his
  `src/core/fasst_frame.c`. 2-FSK at 1.5 Mchip/s with 11-chip Barker spreading, 36 channels, one packet every 7 ms.
  The packets this module builds match a TM-10's byte for byte on the PC.

## Using them

### As a transmitter

- **From an EdgeTX radio:** External RF = MULTI, protocol Futaba → SFHSS, or FASST → 10CH. Option value 1 = dual
  (the 459 MHz link beside, which only a receiver of this kind hears).
- **From the Controller or a terminal:** `cfg tx sfhss` or `cfg tx fasst` (add `dual` for the 459 MHz link).
- **From the TX panel:** Mode → S-FHSS or FASST.
- **Binding:** neither protocol binds on the air. The receiver learns the transmitter while its own Link button is
  used (R6008HS: held for more than 2 s with the receiver on). The transmitter ID comes from the chip, with the
  model number (RX number) folded in, so each model has its own ID.
- **Failsafe sent to the receiver:** throttle (channel 3) at 1000 µs, every other channel holds. S-FHSS sends it every
  7 s, FASST once a second (a FASST receiver forgets its failsafe at power-down).
- **No input for 100 ms:** sticks centred, throttle low; LED_B blinks slowly.

### As a receiver (for another board of this kind)

- `cfg rx sfhss` or `cfg rx fasst`, or the Controller.
- **Bind:** BIND at power-up (or nothing bound yet): the first transmitter heard several times in a row at good
  strength becomes this receiver's, and its ID is stored (binding slots 8 and 9). Afterwards only that ID is
  followed. A new bind clears the stored failsafe preset.
- With the RA-01 fitted, the 459 MHz receiver runs beside it and the servos take its data when 2.4 GHz goes quiet.

## Regulatory

Neither mode has listen-before-talk. Check what your country allows before using them, and keep them to the bench
until they have been proven.
