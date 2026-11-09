# RLINK

Resilient two-way 2.4 GHz RC link between two EFR32MG24 modules. Full
details: "RLINK Protocol Specification" and "RLINK System Specification".

## Protocols on this firmware

| Jumpers low at power-up | Protocol |
|---|---|
| none | nRF24 (+ F/S = RX) |
| PC00 | FrSky D8 (+ F/S = RX) |
| PC01 | FrSky D16 EU-LBT (+ F/S = RX) |
| PC02 (PC01 optional) | RLINK (+ F/S = RX) |

## RLINK quick reference

| Action | How |
|---|---|
| Bind | BIND held at power-up on both, RX first |
| Failsafe preset | RX linked, sticks in position, BIND to GND for 0.3 s |
| Range check | TX with RANGE low at power-up (−10 dBm) |
| Bench jam test | RX with RANGE low at power-up |
| Live status | TX bay UART PA05, 115200 8N1 |
| Counters | `rlink_diag` over SWD (address from the .map) |

## nRF24 receiver (no protocol jumper + F/S low)

Receiver for the module's own nRF24 transmitter and the original ATmega/NRF24L01
transmitter of the same link: same 41-channel hop table, 11 ms
frames, PRIMARY/PRIMARY/AUX cadence, CRC16 over pipe + payload. Bind = BIND
held at RX power-up (or nothing stored yet), then power the TX: its 1.5 s
power-up bind burst is caught, the pipe is stored in flash slot 2. Link comes
up within ~2 s of the TX's power-up. Outputs PPM PC03, 8 servos, SBUS PA03
from the first packet. Failsafe after 440 ms: stored preset, else centre with
throttle 1100 us; preset = TX F/S button held (over the air) or BIND held
0.3 s while linked. RX bay UART PA05 115200: `NR s.. r.. c.. m.. l.. h.. q.. f..`
(state, packets/line, CRC errors, misses now, misses/line, hop index, RSSI,
failsafe). Counters in `nrf_diag` (see nrf24_rx.h). Sync-word handling as
RLINK: the private pipe is verified before every RX, otherwise RAIL's first
radio operation silently puts the bind pipe back.

## Listen-before-talk

Every RLINK transmission (copy A, copy B, telemetry, bind) is preceded by one
100 µs clear-channel check with a 400 µs hard limit. Busy → that packet is
skipped, never retried or delayed; copy B and the next frame carry on. Busy
threshold −70 dBm at +20 dBm EIRP, scaled with the actual power (RL_ANT_GAIN_DB
= 2 dBi assumed). If the radio ever refuses an LBT start, the packet is sent
without it and state bit 64 is set. UART field `l` = packets skipped since the
previous line. Diag 52 = skipped packets, 53 = threshold used. Build with
RL_LBT_ENABLE=0 to switch it off.

## SBUS output (all receiver roles)

PA03, Futaba-style: 100 kbaud 8E2, inverted (idle low), 25-byte frame,
channels 1-8 from the link (1000-2000 us -> 172-1811), channels 9-16 at
centre, flags byte carries frame-lost and failsafe. Written by sbus_out.c and
used by the RLINK, FrSky D8 and FrSky D16 receiver roles; a frame goes out per
received frame and per missed frame. Bytes are pushed from the main loop,
never blocking. SBUS_ENABLE=0 turns it off; RLINK can use a fixed period
instead of per-frame with RL_SBUS_PERIOD_US (14000 = standard rate).
Diag 54 (RLINK) = SBUS frames sent.

## RX battery voltage

Divider on the RX board: Vbatt – 47 k – PA04 – 10 k – GND, 100 nF from PA04
to GND. The RX measures it (IADC0, filtered ~0.6 s) and sends it in every
telemetry stats packet (10 bit, 10 mV). No divider fitted → found at power-up
(PA04 reads 0 with its pull-down) → "no sensor", no alarm. The IADC runs with
16× oversampling and the chip's factory gain and offset trims (r_vb_scale(),
worked out as Silicon Labs' em_iadc.c does); the reported voltage should be
within a few tens of mV of a meter. Oct 2026 fix: the offset register had been
left at its reset value, which fits 2× oversampling only, and every reading came
out 1.9 V too low.

| TX signal | Meaning |
|---|---|
| 2 beeps / s | weak link |
| 3 beeps / s | RX battery below *warning* for 3 s (both warnings: alternating seconds) |
| continuous | land now: link critical, RX failsafe, or RX battery below *critical* for 3 s |

| Pack (RL_VB_PACK in rlink.c) | Warning | Critical |
|---|---|---|
| 1 (the build): 4.8 V NiMH, 4 × AA, 5 V BEC | 4.6 V | 4.4 V |
| 2: 2S LiPo | 7.2 V | 6.8 V |
| 0: picked by the TX from the first reading (> 6.8 V = 2S) and kept until power-off | as above | as above |

The pack type is fixed to 4 × AA (RL_VB_PACK 1) since October 2026: the automatic choice stuck on
whichever pack the transmitter heard first. 5-cell NiMH or 2S LiFe: set fixed values in rlink.c. A battery alarm is latched: it clears only after 10 s at least 0.2 V
above the warning level (i.e. a fresh pack). UART field `v` = RX battery in volts (v0.00 = no sensor); `s`
= highest alarm level. Diag 55 = battery mV, 56 = battery alarm level (RX:
sensor state). Knobs in rlink.c: RL_VB_WARN_MV, RL_VB_CRIT_MV (+ _2S_), RL_VB_TRIM
(per mille, = 1000 × multimeter / reported), RL_VB_ENABLE.

## Build and flash

```powershell
(Get-Item C:\dev\rc_module_trx\app_process.c).LastWriteTime = Get-Date
C:\dev\rc_module_trx\build.ps1
python -m pyocd flash -t efr32mg24a420f1536im48 C:\dev\rc_module_trx\cmake_gcc\build\base\rc_module_trx.hex
```
