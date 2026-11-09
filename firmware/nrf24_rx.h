/*
 * nrf24_rx.h — receiver role for Tobe's nRF24 250 kbps link (EFR32MG24 emulation).
 * #included into app_process.c like the other protocols. Talks to the nRF24
 * transmitter in app_process.c and to the Arduino/NRF24L01 one (Rf_module_3_packet_tx.ino).
 *
 * Boot jumpers:  no protocol jumper + F/S low  = nRF24 receiver
 *                + BIND held                   = (re)bind: catches the TX's 1.5 s power-up
 *                                                bind burst, so power the RX first, then the TX
 *   RX outputs:  PPM PC03, 8 servo pulses (servo_pins.h), SBUS on PA03 (start with the first packet)
 *   Failsafe:    440 ms without packets -> stored preset, else centre + throttle 1100 us
 *                preset = TX F/S button (over the air) or hold BIND 0.3 s while linked
 *   Bay UART:    PA05 115200: "NR s.. r.. c.. m.. l.. h.. q.. f.." 5x/s
 *                s state (0 bind 1 search 2 link)  r packets/line  c CRC errors (total)
 *                m consecutive misses  l misses/line  h hop index  q RSSI dBm  f failsafe
 */
#ifndef NRF24_RX_H
#define NRF24_RX_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

void nrf_rx_start(sl_rail_handle_t rh, bool bind_held);
void nrf_rx_service(bool bind_held);     /* every main-loop pass */
void nrf_rx_process(void);               /* a received packet is held in RAIL */

#define NRX_DIAG_N 24u
extern volatile int32_t nrf_diag[NRX_DIAG_N];
#define N_STATE     0   /* 0 bind, 1 search, 2 link                             */
#define N_RX        1   /* packets received (any)                               */
#define N_CRC_BAD   2   /* wrong length or CRC                                  */
#define N_BIND      3   /* bind packets accepted                                */
#define N_PRIM      4   /* PRIMARY packets decoded (ch 1-5)                     */
#define N_AUX       5   /* AUX packets decoded (ch 6-8)                         */
#define N_MISS      6   /* consecutive misses right now                         */
#define N_LOST      7   /* misses total                                         */
#define N_HOP       8   /* hop index listening on                               */
#define N_CH        9   /* RF channel listening on (2400 + n MHz)               */
#define N_SYNC_NOW 10   /* sync word the radio is using                         */
#define N_SYNC_FIX 11   /* times it had been reset and was re-applied           */
#define N_SYNC_ST  12   /* status of the last sync-word write (0 = OK)          */
#define N_RSSI     13   /* RSSI of the last packet, dBm                         */
#define N_FS       14   /* 1 = outputs hold the failsafe positions              */
#define N_FS_STORE 15   /* failsafe presets written to flash                    */
#define N_SEARCHES 16   /* times the search started                             */
#define N_CFG_ERR  17   /* 1 chans 2 fixlen 8 startrx 128 BIND low at boot      */
#define N_PIPE     18   /* pipe address bytes 0..3 as one word (byte 0 = MSB)   */
#define N_SBUS     19   /* SBUS frames sent                                     */
#define N_SEQ      20   /* frameSeq of the last PRIMARY (0..63)                 */
#define N_LINKS    21   /* times the link was (re)acquired                      */
#define N_ODD      22   /* CRC-good packets of an unexpected type               */
#endif
