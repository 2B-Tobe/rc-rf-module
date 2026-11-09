/*
 * dsm_rx.h — DSMX receiver role for the EFR32MG24 module.  v1.3
 *
 * Receives a Spektrum-compatible DSMX transmitter (measured on the Lemon DIY module, cross-checked
 * with the Multi-module source) with the EFR32 modem: the dsm_raw PHY (2GFSK 1 Mbps, +-400 kHz)
 * syncs on the CYRF6936 preamble (1001..), the frame is despread in software (64-chip SOP, 8DR
 * symbols, CRC-16), and the receiver follows the 23-channel hop sequence: two packets per 11 ms
 * frame, 4.01 ms apart on consecutive channels, each channel every 126.5 ms.
 *
 * Boot jumpers:  PROTOCOL_00 + PROTOCOL_01 (PC00 + PC01) + F/S (PB03) fitted  (PC00 + PC01 alone = transmitter, + PC02 = capture).
 *   BIND held at boot (or nothing stored)  -> bind: scans 2402..2480 MHz for the transmitter's
 *                                             bind burst (hold the Lemon's bind button, switch the
 *                                             transmitter on), stores the id in flash, then answers
 *                                             with the bind response for 5 s (the transmitter leaves
 *                                             bind mode by itself) and searches
 *   otherwise                              -> search the stored id's channels, then follow
 *   BIND held 0.3 s while linked           -> store the current positions as the failsafe preset
 * Outputs (as the nRF24 receiver role): PPM PC03, servo pulses PA07 PB04 PD04 PD05 PA00 PA08 PA09 PB05 (dual build),
 *   SBUS PA03 (DRX_SBUS 1, on EUSART1; the log keeps PA05).  Output order AETR: out1 aileron, out2 elevator, out3 throttle, out4 rudder,
 *   out5 gear, out6..8 aux1..3 (DSM channel ids 1 2 0 3 4 5 6 7; DRX_OUT_MAP in dsm_rx.c).
 *   Values: DSM 0..2047 -> 900..2100 us (1024 = 1500 us, +-100 % = 1100..1900 us).
 * Failsafe: 500 ms without packets -> stored preset, else centre + throttle 1000 us; search
 *   again after 2 s.
 * LEDs: LED_B bind = fast blink, search = slow blink, link = steady; LED_A toggles per 10 packets.
 * UART PA05 115200:
 *   "DR bind: scanning 2402..2480 MHz"            "DR bound: id 37 22 1A 2D type B2 8 ch (stored)"
 *   "DR search"   "DR link idx=9 (2440 MHz) pos=1"   "DR lost"
 *   "DR s2 r36 c0 n0 m0 l0 h9 q-24 f0"  5x/s: s state (0 bind 1 search 2 link), r good packets/line
 *        (36 = all of them), c CRC errors total, n frames without our SOP total, m consecutive
 *        misses, l misses/line, h hop index, q RSSI, f failsafe
 *   "DR ch 1500 1629 998 1545 1629 1978 1500 2003  q=63 P=11023 e=12"  1x/s: the 8 outputs in us
 *        (AETR order), worst symbol match of the last packet, tracked frame period, last timing error
 *
 * Over SWD: dsm_rx_diag[] (int32).
 */
#ifndef DSM_RX_H
#define DSM_RX_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

void dsm_rx_start(sl_rail_handle_t rh, bool bind_held);
void dsm_rx_service(bool bind_held);     /* every main-loop pass */
void dsm_rx_process(void);               /* a received packet is held in RAIL */

#define RX_SLOT_DSM  3u                  /* rx_store slot: id in hop[0..3], type in hop[4], channels in hop[5] */

#define DRX_DIAG_N 24u
extern volatile int32_t dsm_rx_diag[DRX_DIAG_N];
#define DR_STATE     0   /* 0 bind, 1 search, 2 link, 3 bind response                 */
#define DR_RX        1   /* frames delivered by the modem (any)                       */
#define DR_NOSOP     2   /* frames without our SOP (other channel / other TX / noise) */
#define DR_CRC_BAD   3   /* frames with the SOP but a bad CRC                         */
#define DR_PKT       4   /* good packets                                              */
#define DR_MISS      5   /* consecutive misses right now                              */
#define DR_LOST      6   /* misses total                                              */
#define DR_HOP       7   /* hop index listening on (0..22)                            */
#define DR_CH        8   /* RF channel listening on (2400 + n MHz)                    */
#define DR_RSSI      9   /* RSSI of the last frame, dBm                               */
#define DR_FS       10   /* 1 = outputs hold the failsafe positions                   */
#define DR_FS_STORE 11   /* failsafe presets written to flash                         */
#define DR_SEARCHES 12   /* times the search started                                  */
#define DR_CFG_ERR  13   /* 1 chans 2 fixlen 4 fifo 8 startrx 16 timer 32 sync 64 starttx 128 BIND low at boot */
#define DR_ID       14   /* id0..id3 as one word (id0 = MSB)                          */
#define DR_SBUS     15   /* SBUS frames sent                                          */
#define DR_LINKS    16   /* times the link was (re)acquired                           */
#define DR_BINDS    17   /* bind packets accepted                                     */
#define DR_Q        18   /* worst symbol match of the last good packet (64 = perfect) */
#define DR_SOP_AT   19   /* chip position of the SOP in the last good frame           */
#define DR_SYNC_FIX 20   /* sync word found reset and re-applied                      */
#define DR_LUT_MISS 21   /* symbols that needed the full search (noisy)               */
#define DR_TYPE     22   /* bind type byte (0xB2 = DSMX 11 ms 2 frames)               */
#define DR_SCANS    23   /* bind scan sweeps                                          */
#endif
