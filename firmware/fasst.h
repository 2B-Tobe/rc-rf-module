/*
 * fasst.h — Futaba FASST 10ch transmitter role, and a receiver role for another board of this kind, for the
 * EFR32MG24 module (fasst.c, v0.3: experimental; the packets match a Futaba TM-10's byte for byte on the PC, the
 * transmitter sends on the bench; not yet tried with Futaba equipment).
 *
 * Everything known here about FASST comes from Raphael Coeffic's mpm-v2 (github.com/raphaelcoeffic/mpmv2, GPL v3:
 * src/core/fasst_frame.c, src/mpm/proto_fasst.c, docs/mpm-core.md "Futaba FASST"), who traced it from a TM-10 module
 * driving an R6008HS and links that receiver from a TI CC2652. The framing code in fasst.c is a port of his.
 *
 *   radio   2-FSK at 1.5 Mchip/s, deviation 500 kHz (an R6008HS takes 1.46..1.545 Mchip/s and 200..512 kHz)
 *   chips   every bit is sent as 11 Barker chips (11100010010 for a 0, inverted for a 1): 64 preamble bits
 *           (1010..10), 16 sync bits (0x2BB1, bit 0 first), 176 packet bits = 2816 chips = 352 bytes
 *   air     36 channels, 2405.376 MHz + k x 2.048 MHz; one packet per 7 ms, then the hop:
 *           k + step mod 37, skipping 36 (the step, 1..36, is in every packet; chosen at power-up)
 *   packet  22 bytes: ID (4), hop position k + 1 and the step (2), four groups of two 11-bit channels with a flag
 *           each (ch1/ch2 or ch9/ch10 in the first group, every other frame; ch3..ch8), two type bytes,
 *           CRC-16/KERMIT; then the whole packet XORed with a fixed stream (the same for every transmitter)
 *   value   11 bits, pulse = 880 + 0.625 x value us: 1024 = 1520 us, 224 = 1020 us, 1824 = 2020 us
 *   bind    none on the air: the receiver learns the transmitter nearby while its Link button is held
 *           (R6008HS: more than 2 s with the receiver on)
 *   F/S     a cycle of 6 packets (types 1..6: the failsafe position or "hold" of two channels each) once a second;
 *           the receiver forgets its failsafe at power-down, so it is sent all the time
 *
 * Here: the module's 8 channels as FASST ch1..ch8, ch9 / ch10 at centre. Input 1500 us = 1520 us at the receiver,
 * 1 us per us, same direction (1000 us in = 1020 us out). ID: 0x12 (as the TM-10's), then 24 bits from the chip's
 * EUI64 with the model number folded in (per-model binding). Step: a random one at every power-up (mpm-v2 listens
 * first for the steps other FASST transmitters use; this module cannot receive FASST, so it does not).
 * Failsafe sent: throttle (channel 3) at FASST_FS_THROTTLE_US, every other channel holds.
 * Power: flight FASST_TX_POWER_DDBM, RANGE low = FASST_RANGE_POWER_DDBM, followed at any time.
 * No input for 100 ms: centre sticks, throttle low; LED_B blinks slowly. No telemetry (FASST has none).
 * The frames run from the radio's TX-done events on a fixed 7 ms grid; a slot that cannot be met is skipped with its
 * hop (as sfhss.c), so the receiver, which hops by the step, finds the transmitter where it expects it.
 *
 * The radio sends the chips as plain bits: the PHY "fasst" in radio_settings.radioconf is 2-FSK 1.5 Mbit/s with
 * neither CRC nor whitening; its 32-bit sync word is set to the first 32 chips, the other 348 bytes are the payload.
 * UART: "FA boot ...", every 5 s "FA tx frames.. skipped.. late.. err.. hop.. ch ..".
 *
 * Receiver role (v0.2), for a transmitter board of this kind: the radio receives the chips after its sync word (the
 * first 32 chips, straight after the radio's own preamble), the chips are despread and the 16 sync bits looked for,
 * then CRC, ID and hop are checked. Whether it also hears a Futaba transmitter (which sends no radio preamble before
 * the chips) is not known.
 *   bind    BIND at power-up (or nothing stored): a transmitter at -75 dBm or stronger, followed hop by hop until 10
 *           of its frames came, becomes ours; its ID is kept in flash (slot 9). Otherwise only that ID is followed.
 *   search  one channel at a time, 300 ms each (the transmitter passes every channel within 252 ms); a packet of
 *           ours gives the hop position and the step, and the frame timing
 *   link    one packet per 7 ms, then the hop by the step in the packet; a slot with none counts as lost and the hop
 *           goes on by the clock. 0.2 s lost: wait on one channel. 1 s lost: failsafe; 3 s: search
 *   out     8 PWM / SBUS / PPM as every receiver role; ch9 / ch10 are not used
 *   failsafe hold, or the positions the transmitter sends in its F/S cycle, or - first choice - the preset stored
 *           with BIND held 0.3 s while linked
 *   UART    "FA rx ...": state, frames, lost, bad, RSSI, hop, step, the 8 channels
 */
#ifndef FASST_H
#define FASST_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

/* Returns false if the fasst PHY is not in the build (rail_config.c not generated). */
bool fasst_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool range_low, bool rx_role, bool bind_held);
void fasst_rx_process(void);             /* receiver: a packet is held in RAIL */
void fasst_rx_service(bool bind_held);   /* receiver, every main-loop pass */
bool fasst_rx_fresh(uint32_t now, uint32_t within_us);
#define RX_SLOT_FASST 9u                 /* rx_store slot: id in hop[0..3] */
void fasst_input(bool input_present);    /* every main-loop pass */
void fasst_range(bool range_low);        /* every main-loop pass */
void fasst_action(void);                 /* every main-loop pass */
void fasst_on_events(sl_rail_events_t events);   /* from the RAIL event callback */

/* host-testable pieces (ported from mpm-v2's fasst_frame.c) */
#define FASST_CHANNELS    10u
#define FASST_PACKET_LEN  22u
#define FASST_CHIPS_LEN   352u           /* (64 + 16 + 176) bits x 11 chips / 8 */
#define FASST_HOPS        36u
#define FASST_FS_TYPES    6u
void     fasst_set_id(uint32_t id);
uint8_t  fasst_next_hop(uint8_t k, uint8_t step);
uint32_t fasst_freq_hz(uint8_t k);
uint16_t fasst_value(uint16_t us);       /* pulse in us at the module's input -> the 11-bit value */
/* One packet for hop k and the step. ch: 10 values. type 0: normal (alt: ch9 / ch10 in place of ch1 / ch2);
 * type 1..6: the F/S cycle, fs = the 10 failsafe values, hold bit i = channel i holds instead. */
void     fasst_packet(uint8_t *p, uint8_t k, uint8_t step, const uint16_t *ch, bool alt, uint8_t type,
                      const uint16_t *fs, uint16_t hold);
void     fasst_spread(uint8_t *chips, const uint8_t *p);     /* FASST_CHIPS_LEN bytes, first chip in bit 7 of byte 0 */
uint32_t fasst_rail_sync(const uint8_t *chips);              /* the first 32 chips as RAIL's sync word (bit 0 first on air) */
uint16_t fasst_us(uint16_t value);
bool     fasst_rx_decode(const uint8_t *pl, uint8_t *raw, uint16_t *late_chips);
bool     fasst_unpack(const uint8_t *raw, uint8_t *plain, uint8_t *k, uint8_t *step);
void     fasst_slots(const uint8_t *plain, const uint8_t *raw, uint16_t *v, uint16_t *live, uint16_t *fsp, uint16_t *fsh);

#define FASST_DIAG_N 8u
extern volatile int32_t fasst_diag[FASST_DIAG_N];
#define FA_TX        0   /* packets sent                                    */
#define FA_LATE      1   /* packets launched late                           */
#define FA_TXERR     2   /* refused or failed sends                         */
#define FA_HOP       3   /* hop of the last packet, 0..35                   */
#define FA_ID        4   /* the 32-bit ID                                   */
#define FA_STEP      5   /* TX: hop step 1..36 / RX: 0 bind, 1 search, 2 link */
#define FA_INPUT     6   /* 1 = input present                               */
#define FA_CFG_ERR   7   /* 1 chans 2 fixlen 4 fifo 8 rx fifo 16 rx transitions 32 sync 64 starttx 128 no PHY */
#endif
