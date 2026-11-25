/*
 * jeti.h — "quasi" Jeti Duplex 2.4 for the rc_module_trx EFR32MG24 module (jeti.c v0.5, EXPERIMENTAL).
 *
 * Protocol 111 (mpm-v2's number), sub 0.   cfg name: duplex   (cfg tx duplex / cfg rx duplex)
 *
 * Where it comes from: Raphael Coeffic's mpm-v2 (github.com/raphaelcoeffic/mpmv2, GPL v3): src/core/jeti_frame.c
 * (framing, scrambler), src/mpm/proto_jeti.c (the transmitter: channel slots, hop ring, channel sweep, carrier
 * sense, bind, the reply's layout; there it binds and gets telemetry from Duplex R6 and REX10 receivers) and
 * src/core/bench_jeti.c (the follower: the hop clock). Checked here against our own captures of a Jeti TU2 module
 * (30,206 data frames parse and scramble back to the bytes on the air). v0.3 brings the transmitter up to
 * proto_jeti.c and adds what a Jeti receiver does on the air (replies, pairing) to the receiver.
 *
 * The radio: RAIL's built-in IEEE 802.15.4 2.4 GHz PHY (O-QPSK 250 kbit/s, channels 11..26), PSDUs both ways, FCS
 * by RAIL. Frame: 01 08 | seq | PAN | dst | payload; the payload's first byte (type) plain, the rest scrambled.
 * Sequence number: +1 per frame; a reply carries the number of the frame it answers.
 *
 * TRANSMITTER role (as proto_jeti.c, 8 channels):
 *   - a data frame (type 0xB1, 0x21, 8 slots: 8 x (us - 1500) - 1) every 20 ms = every 2nd 10 ms step, on the
 *     step's channel of a ring of 5 (first from the chip ID), to the paired receiver (broadcast if none);
 *   - carrier sense: a clear-channel check (-70 dBm) after a random wait; busy: wait and check again (802.15.4
 *     CSMA-CA), the frame's place bits following its send time; past the step's window the frame goes on the next
 *     step, past that one it is lost;
 *   - channel sweep: every 12 frames one ring slot moves to another channel, the slot with the most unanswered
 *     frames first; a channel with a frame skipped or unanswered is passed over for about 10 s;
 *   - the receiver's reply (17 bytes, about 3 ms after the frame, same channel): its voltage, signal figure, the
 *     RSSI here and the share of frames answered -> the log, and the encoder's telemetry frame (bay);
 *   - bind (BIND at power-up, or BIND while running): 150 ms on each channel in turn, a frame every 10 ms (5 x
 *     0x23, then 0x22 with channel data) to the broadcast address, until a receiver answers with its address
 *     (stored, rx_store slot 11) or 60 s have passed;
 *   - no input for 100 ms: sticks centred, throttle (channel 3) low. RANGE: low power.
 *   Not ported: 12 / 16 channels (the module has 8 inputs), the remote JETIBOX, the sensors' EX data.
 *
 * RECEIVER role (our own; mpm-v2 has no receiver. For a Jeti transmitter module such as the TU2, or a board of
 * this kind as transmitter):
 *   - search: 220 ms on each of the 16 channels until a frame of its transmitter comes; then it follows the
 *     transmitter's 10 ms steps through the ring the frames carry (the hop clock as bench_jeti.c: each frame's
 *     time less its place bits, averaged; 2.5 ms off: set anew). Every frame brings the ring up to date;
 *   - replies as a Jeti receiver (payload: type x2, 0x13, a counter, the voltage every 4th reply if the board has
 *     the battery divider on PA04, flags, signal figure), after a clear-channel check, to the transmitter's PAN
 *     and short address. The reply's type byte upper half is a guess (the frame's own), and whether a Jeti
 *     transmitter accepts the replies is NOT yet tested. RANGE held at power-up: no replies (listens only);
 *   - bind: BIND at power-up (or nothing bound): the first transmitter heard at -70 dBm or stronger; its 0x22
 *     frames are answered with our own address appended (the Jeti pairing); after 10 frames its short address and
 *     PAN are stored (rx_store slot 10). Afterwards only that transmitter is followed. A transmitter whose PAN no
 *     frame carried (it sent 0x21 frames only) is followed without replies;
 *   - 8 channels -> servo outputs / SBUS / PPM as the other receiver roles; the DS-14's masked blocks are not read;
 *   - 0.5 s without a frame: search again; 1 s: failsafe (the stored preset, else hold). BIND pressed 0.3 s while
 *     linked stores the failsafe preset.
 *
 * Log lines start with "JQ".
 */
#ifndef JETI_H
#define JETI_H

#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

#define RX_SLOT_JETI    10u               /* rx_store slot, receiver: the transmitter's short address hop[0..1], PAN hop[2..3] */
#define RX_SLOT_JETITX  11u               /* rx_store slot, transmitter: the receiver's PAN hop[0..1], short hop[2..3] */

typedef struct {
  bool     link, bound, binding;
  int8_t   rssi_tx;                       /* the reply's RSSI at the module, dBm */
  uint8_t  signal;                        /* the receiver's own signal figure */
  uint8_t  lq;                            /* frames answered, % of the last 50 */
  uint16_t vbat_10mv;                     /* the receiver's voltage, 0 = none reported */
} jeti_telem_t;

bool jeti_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool range_low, bool rx_role, bool bind_held);
void jeti_on_events(sl_rail_events_t events);
void jeti_rx_process(void);               /* both roles: the held packets (main loop, after rx_held) */
void jeti_action(void);                   /* transmitter: every main-loop pass */
void jeti_input(bool input_present);
void jeti_range(bool range_low);
void jeti_bind(bool want);                /* transmitter: every main-loop pass; a new BIND starts the bind sequence */
void jeti_telem(uint32_t now, jeti_telem_t *t);   /* transmitter */
void jeti_rx_service(bool bind_held);     /* receiver: every main-loop pass */
bool jeti_rx_fresh(uint32_t now, uint32_t within_us);

/* the pure pieces */
void     jeti_scramble(const uint8_t *plain, uint8_t *out, uint8_t n);     /* n: PSDU bytes without the FCS; in place allowed */
void     jeti_descramble(const uint8_t *in, uint8_t *out, uint8_t n);
uint16_t jeti_slot(uint16_t us);          /* 1500 us -> 0x3FFF (= -1), 8 counts per us, 14 bits */
uint16_t jeti_slot_us(uint16_t slot);     /* back to us, limited to 800..2200 */

#endif
