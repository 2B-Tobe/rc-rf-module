/*
 * jeti_cap.h — Jeti Duplex sniffer for the rc_module_trx EFR32MG24 module (jeti_cap.c, v1.0).
 *
 * The Jeti DUPLEX transmitter module and receivers are built on the Atmel AT86RF230, an IEEE 802.15.4
 * 2.4 GHz transceiver: O-QPSK with 32-chip direct-sequence spreading, 2 Mchip/s, 250 kbit/s, on the 16
 * standard channels 11..26 (2405 + 5 x (ch - 11) MHz). Every frame on the air has the 802.15.4 PHY framing:
 * 4 zero bytes of preamble, SFD 0xA7, one length byte (PHR), up to 127 bytes, 16-bit FCS. What Jeti puts
 * inside is theirs, and that is what this role records.
 *
 * The EFR32MG24 has that PHY in hardware (RAIL IEEE 802.15.4 mode). In promiscuous mode RAIL delivers every
 * frame that is on the channel, no address or frame-type filtering; with the CRC-error option it delivers
 * frames with a bad FCS as well, marked. This role:
 *   - sweeps the 16 channels, JC_DWELL_US on each, and prints every frame with the time, channel, RSSI,
 *     length, FCS result, the time since the previous frame on that channel, and the bytes in hex;
 *   - prints a census line after every sweep: frames per channel, ok and FCS-bad;
 *   - with BIND held at power-up ("lock"): after JC_LOCK_SWEEPS sweeps it parks on the busiest channel and
 *     prints every frame there, so the frame period and the sequence numbers can be read;
 *   - with F/S fitted at power-up: lock on channel JC_FIXED_CH (11) right away. (RANGE is not used here: RANGE at
 *     power-up means encoder control in this firmware.)
 * The log goes to the usual UART (PA05, 115200). Lines are rate-limited (JC_PRINT_PER_DWELL in a sweep,
 * JC_PRINT_PER_SEC when locked); frames beyond that are counted, not printed.
 *
 * Selection: jumpers PC00 + PC02 at power-up (PC01 open), or serial frames Prot 0 Sub 6 (RANGE jumper).
 * No PPM or encoder frames are needed; the role only listens.
 *
 * Bytes printed: exactly what RAIL hands over. The first byte is the PHR (the length the Jeti chip wrote,
 * counting the FCS); the last two bytes are the FCS as received (SL_RAIL_RX_OPTION_STORE_CRC).
 */
#ifndef JETI_CAP_H
#define JETI_CAP_H

#include <stdbool.h>
#include <stdint.h>
#include "sl_rail.h"

void jeti_cap_start(sl_rail_handle_t rh, bool lock_after_sweeps, bool lock_fixed_ch);
void jeti_cap_rx_process(void);                 /* main loop, when a packet is held in the FIFO           */
void jeti_cap_service(void);                    /* main loop, every pass: channel timing, census, lock     */
void jeti_cap_on_events(sl_rail_events_t ev);   /* RAIL event callback (ISR): counters, overflow recovery  */

#define JETI_CAP_DIAG_N 8u
extern volatile int32_t jeti_cap_diag[JETI_CAP_DIAG_N];
#define JC_INIT     0   /* 0 ok, else the RAIL status that failed (1xx init, 2xx radio config, 3xx options) */
#define JC_FRAMES   1   /* frames received, FCS ok                                                          */
#define JC_BAD      2   /* frames received, FCS bad                                                         */
#define JC_OVF      3   /* RX FIFO overflows                                                                */
#define JC_ABORT    4   /* RX packets aborted / frame errors                                                */
#define JC_CH       5   /* channel listened to now                                                          */
#define JC_LOCKED   6   /* 1 while parked on one channel                                                    */
#define JC_SWEEPS   7   /* sweeps completed                                                                 */

#endif /* JETI_CAP_H */
