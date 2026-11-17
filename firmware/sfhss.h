/*
 * sfhss.h — Futaba S-FHSS transmitter and receiver roles for the EFR32MG24 module (sfhss.c, v0.4: links between two boards of this kind on the bench; not yet tried with Futaba equipment here).
 *
 * The channels go out as Futaba S-FHSS for S-FHSS receivers (Futaba R2006GS / R2008SB, and the compatible ones).
 * Protocol facts from the DIY-Multiprotocol-TX-Module project, Futaba_cc2500.ino (GPL v3, itself from Deviation's
 * sfhss_cc2500.c): the registers a Futaba transmitter writes to its CC2500, the packet and the timing.
 *
 *   radio   2-FSK 128143 bit/s, deviation 38.086 kHz, 4 bytes of preamble (0xAA), sync 0xD391 sent twice
 *           (CC2500 SYNC_MODE 3 = 30 of 32 bits), 13 data bytes, CRC-16, no whitening
 *   CRC     the CC2500 runs in its "CC2400 compatible" CRC mode (PKTCTRL0 CC2400_EN): CRC-16 0x8005 from 0 with the
 *           data bit entering at x0. Measured on a CC2500 by Raphael Coeffic (mpm-v2, github.com/raphaelcoeffic/mpmv2,
 *           GPL v3: "from 0 over all bytes but the last 2, which are XORed into it unprocessed" - the same thing); a
 *           Futaba R2008SB links with it there. (v0.3 had three candidate forms on Sub 0..2; Sub 0 was the right one.)
 *   air     30 channels, 2403.999 MHz + n x 1.499634 MHz (CC2500 channel 16 + 6 n, 249.939 kHz each)
 *   frame   6.8 ms: packet A, packet B 1.625 ms later on the same channel, then the hop
 *           next = ch + code + 2; over 29: (30 -> once more) - 31     (code 0..27, sent in every packet)
 *   packet  81 id0 id1 00 00 | ch(5) v0(12) v1(12) v2(12) v3(12) code(5) cmd(4), bit-packed, MSB first
 *           cmd bit 0 = packet B, bit 1 = 1 (0 around the failsafe packets), bit 2 = failsafe values, bit 3 = channels 5-8
 *           order A: 1-4, B: 5-8, A: 5-8, B: 1-4, ... (the Futaba T8J's)
 *   value   1520 = centre, 2020 = 988 us, 1020 = 2012 us (the value falls as the pulse grows)
 *   bind    none on the air: the receiver learns the transmitter's ID when its own link button is used
 *
 * ID: 16 bits from the chip's EUI64 (with the model number folded in: per-model binding), no run of more than six
 * equal bits (some receivers need that). Hop code: 0..27, new at every power-up.
 * Failsafe: every 1024 frames (7 s) two packets carry failsafe words (cmd bit 2), as Multi sends them: throttle
 * (channel 3) at SFHSS_FS_THROTTLE_US, every other channel "hold". Words as a Futaba T14SG's (mpm-v2): 0..1023 no
 * pulses, 1024..2047 hold, 2048.. a position = 0x800 | signed 10-bit offset from 1520 in 1.25 steps.
 * Power: flight SFHSS_TX_POWER_DDBM, RANGE low = SFHSS_RANGE_POWER_DDBM, followed at any time.
 * No input for 100 ms: centre sticks, throttle low (as the DSMX role); LED_B blinks slowly.
 * The frames run from the radio's TX-done events on a fixed 6.8 ms grid; a slot that cannot be met is skipped with
 * its hop, so a receiver that hops by the clock stays with the transmitter (v0.3).
 * Dual: as every 2.4 GHz mode. Opt 1 on a board with the SX1278 runs the UK 459 MHz link beside S-FHSS (for a
 * receiver of this kind; a Futaba receiver hears 2.4 GHz only). The receiver role runs the 459 MHz receiver
 * beside S-FHSS and the servos take its data within 100 ms of losing 2.4 GHz.
 * UART: "SF boot ...", every 5 s "SF tx frames.. skipped.. late.. err.. ch ..".
 *
 * Receiver role (v0.2): the board is an S-FHSS receiver, for another board of this kind or a Futaba transmitter.
 *   bind    BIND at power-up (or nothing stored): the first transmitter heard 10 times in a row at -75 dBm or
 *           stronger becomes ours; its ID is kept in flash (slot 8). Otherwise only that ID is followed.
 *   search  one channel at a time, 300 ms each (the transmitter passes every channel within 204 ms); a packet of
 *           ours gives the channel and the hop code, and the frame timing from its sync word
 *   link    packet A, packet B 1.625 ms later (the radio stays in receive between them), then the hop; a frame with
 *           neither counts as lost and the hop goes on by the clock. 0.2 s lost: wait on one channel instead (the
 *           transmitter may have restarted; it passes every channel within 204 ms). 1 s lost: failsafe; 3 s: search
 *   out     8 PWM / SBUS / PPM as every receiver role (servo_out.c, sbus_out.c); channels 5-8 centre until heard
 *   failsafe hold, or the positions the transmitter sends as failsafe (cmd bit 2), or - first choice - the preset
 *           stored with BIND held 0.3 s while linked
 *   UART    "SF rx ...": state, frames, lost, RSSI, the 8 channels
 */
#ifndef SFHSS_H
#define SFHSS_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

/* rx_role: receiver; bind_held: BIND at power-up (receiver).
 * Returns false if the sfhss PHY is not in the build (rail_config.c not generated). */
bool sfhss_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool range_low, bool rx_role, bool bind_held);
void sfhss_rx_process(void);             /* receiver: a packet is held in RAIL */
void sfhss_rx_service(bool bind_held);   /* receiver, every main-loop pass */
bool sfhss_rx_fresh(uint32_t now, uint32_t within_us);
uint16_t sfhss_us(uint16_t value);       /* the packet's 12-bit value -> pulse in us */
#define RX_SLOT_SFHSS 8u                 /* rx_store slot: id in hop[0..1] (7 until v0.4: that is the UK 459 MHz receiver's) */
void sfhss_input(bool input_present);    /* every main-loop pass */
void sfhss_range(bool range_low);        /* every main-loop pass */
void sfhss_action(void);                 /* every main-loop pass */
void sfhss_on_events(sl_rail_events_t events);   /* from the RAIL event callback */

/* host-testable pieces */
uint16_t sfhss_crc(const uint8_t *d, uint8_t n);
uint16_t sfhss_fs_word(uint16_t value);   /* failsafe: a position (live value) -> the F/S word */
uint16_t sfhss_fs_value(uint16_t word);   /* failsafe: an F/S position word -> the live value  */
uint8_t  sfhss_next_chan(uint8_t ch, uint8_t code);
void     sfhss_build(uint8_t *p, uint16_t id, uint8_t rf_ch, uint8_t code, uint8_t cmd, const uint16_t v[4]);
uint16_t sfhss_value(uint16_t us);
uint16_t sfhss_fix_id(uint32_t seed);

#define SFHSS_DIAG_N 12u
extern volatile int32_t sfhss_diag[SFHSS_DIAG_N];
#define SF_TX        0   /* packets sent                                    */
#define SF_LATE      1   /* packets launched late                           */
#define SF_TXERR     2   /* refused or failed sends                         */
#define SF_CH        3   /* channel of the last frame, 0..29                */
#define SF_ID        4   /* the 16-bit ID                                   */
#define SF_CODE      5   /* hop code 0..27                                  */
#define SF_INPUT     6   /* 1 = input present                               */
#define SF_CFG_ERR   7   /* 1 chans 2 fixlen 4 fifo 32 sync 64 starttx 128 no PHY */
#define SF_FS         8   /* TX: failsafe packets sent                       */
#define SF_STATE      9   /* RX: 0 bind, 1 search, 2 link                    */
#define SF_LOST      10   /* RX: frames lost                                 */
#define SF_RSSI      11   /* RX: last packet, dBm                            */
#endif
