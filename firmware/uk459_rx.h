/*
 * uk459_rx.h — 459 MHz UK receiver role on the SX1278 (RA-01), DUAL build.
 *
 * Port of GFSK_Rx_UK459_HOP V7 (GFSK_Rx_UK459_HOP.ino, sx1278_fsk.ino, def_fsk.h): the receiver side of
 * the UK 459 MHz hopping link that uk459_tx.c transmits. Works with the original V9 transmitter and with the
 * module's UK link (every transmitter mode in dual, or UK only, encoder Prot 0 Sub 2).
 *
 *   Jumpers: PC01 + PC02 + F/S = UK receiver only (PC02 + F/S alone stays the RLINK receiver).
 *   BIND jumper at power-up: listen on 458.5375 MHz for three matching bind packets, store ID + lane in the
 *   module's bind page (rx_store slot 7). With a stored bind and BIND held but nothing heard for 8 s: back to the
 *   stored transmitter. Without a stored bind: bind until one is heard.
 *   Data: 10-byte packets every 40 ms, 5 per channel; the 5th is the sync beacon (byte 9 = 0xFE, byte 8 = ID0 ^
 *   next hop index): retune right after it. Missed beacon: hop on the 250 ms fallback clock. No valid packet for
 *   1.2 s: failsafe (stored preset, else throttle 1000 us and centre), park on hop 0 and listen.
 *   Failsafe capture packet from V9 (byte 9 = 0xFF): the channel bytes become the stored preset. BIND held
 *   0.3 s while linked: the current positions become the preset (as the other receiver roles).
 *   Outputs as every receiver role: 8 servos (servo_pins.h), PPM on PC03, SBUS on PA03.
 *   Channels: byte -> 950 + 4 x byte us, clamped 1000..2000 (V7), servo 1 = channel 1 as the encoder sends them.
 *
 *   #included into app_process.c after uk459_tx.c (shares its SPI and radio helpers).
 */
#ifndef UK459_RX_H
#define UK459_RX_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

#define UR_DIAG_N   12u
#define UR_STATE     0   /* 0 bind, 1 data                                                    */
#define UR_RX        1   /* valid packets                                                      */
#define UR_BEACON    2   /* beacons followed                                                   */
#define UR_FALLBACK  3   /* hops on the 250 ms fallback clock (beacon missed)                  */
#define UR_FS        4   /* 1 while in failsafe                                                */
#define UR_HOP       5   /* current hop index                                                  */
#define UR_RSSI      6   /* RSSI of the last packet, dBm                                       */
#define UR_BIND      7   /* bind packets accepted                                              */
#define UR_ODD       8   /* packets with a wrong ID or marker                                  */
#define UR_FSSTORE   9   /* failsafe presets stored                                            */
#define UR_LINKS    10   /* times the link came up                                             */
#define UR_SBUS     11   /* SBUS frames sent                                                   */
extern volatile int32_t ukrx_diag[UR_DIAG_N];

void uk_rx_start(sl_rail_handle_t rh, bool bind_held, bool alone);   /* alone: the UK receiver role (outputs, LEDs); else beside a 2.4 GHz receiver role */
void uk_rx_service(bool bind_held);                       /* every main-loop pass */
const volatile uint16_t *uk_rx_channels(void);            /* channels 1..8 in us (dual receiver, later) */
bool uk_rx_fresh(uint32_t now, uint32_t within_us);       /* a valid packet within this long */
#endif
