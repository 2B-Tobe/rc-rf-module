/*
 * frsky_d16.h — FrSky D16 EU-LBT (ACCST X series, v1 format) transmitter for
 * the rc_module_trx EFR32MG24 module, on the `frsky_d16eu` PHY (2-FSK 100 kbps,
 * dev 70 kHz, 2405 MHz + 300 kHz * ch, preamble 0xAA x4, sync 0xD391, fixed
 * length, no HW CRC — D16 carries its own CRC).  RAIL 3 API.   Version 0.
 *
 * From Multi-Module FrSkyX_cc2500.ino / FrSkyDVX_common.ino (EU/LBT path):
 *   9 ms slot: listen 0.4 ms (LBT) -> TX 33 bytes -> at +4.4 ms open RX for
 *   the 15-byte telemetry reply until +8.6 ms.  Hop every packet:
 *   hop_no = (hop_no + chanskip) % 47.  Bind on channel 0 with the hop table.
 *
 * frsky16_start(..., rx_role=true) turns the board into a D16 EU receiver (the
 * app selects it with jumper PC01 + F/S PB03 low at boot): binds to the first
 * transmitter heard on channel 0 (kept in flash), follows
 * its hops, answers telemetry, and outputs 8-channel PPM on PC03 (300 us low
 * pulses, idle high, 22.5 ms frame) plus servo PWM ch1..8 on PA07 PB04 PD04 PD05
 * PA00 PA08 PA09 PB05 (sequential pulses; dual build, servo_pins.h). Outputs stop ~1 s after the link is lost.
 * Bind is stored in the USERDATA flash page (rx_store.c): the receiver comes up
 * bound; hold BIND (PB00) at power-up to forget it and bind again.
 * Failsafe ~1 s after link loss: stored preset if one exists (hold BIND >0.3 s
 * while linked to store the current positions), else hold (FRSKY16_FS_DEFAULT 1 = no pulses).
 * Diag [2] in RX role: +1 bind stored, +100 per preset stored.
 * Diag in RX role: [1] 0 bind-listen / 10 bound-searching / 11 tracking,
 * [3] telemetry replies sent, [7] CRC-good packets, [8] bind, [9] data,
 * [10] rejected, [11..12] ch1/ch2 us, [13] hop_no, [14] our RSSI, [15] misses.
 * frsky16_link_up() in RX role = tracking and fewer than ~1 s of misses.
 */
#ifndef FRSKY_D16_H
#define FRSKY_D16_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

typedef enum { FRSKY16_PHASE_BIND = 0, FRSKY16_PHASE_DATA, FRSKY16_PHASE_RX } frsky16_phase_t;

void            frsky16_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool bind_held, bool rx_role);
void            frsky16_service(bool bind_held, bool range_low);
/* Transmitter, every loop: true = BIND on in the serial frames. While it is, 1.5 s of bind and 1.5 s of
 * flying data take turns. Jumper mode passes false: 1.5 s of bind at power-up, as before. */
void            frsky16_bind(bool held);
frsky16_phase_t frsky16_phase(void);
bool            frsky16_link_up(void);
int8_t          frsky16_tele_rssi_dbm(void);   /* TX: RSSI of the last telemetry packet heard, dBm (RX: of the last data packet) */
void            frsky16_on_events(sl_rail_events_t events);
void            frsky16_rx_process(void);
void            frsky16_on_rx_held(void);                 /* RX: from the RAIL event handler, after a packet is held */
const uint8_t  *frsky16_last_pkt(uint16_t *n);            /* the last packet heard, raw, and its length (log) */
void            frsky16_tx_stats(uint32_t *busy, uint32_t *resync, uint32_t *refused);   /* TX: LBT busy, chain restarts, refused scheduled sends */
extern uint8_t  frsky16_variant;                          /* Sub: 0 v1 EU, 1 v2 EU, 2 v1 FCC, 3 v2 FCC, 5 v1 FCC 8 ch */

extern volatile int32_t frsky16_diag[16];
#define G_ROLE        0
#define G_PHASE       1
#define G_BIND_SENT   2
#define G_DATA_SENT   3
#define G_TX_OK       4
#define G_TX_ERR      5
#define G_CFG_ERR     6
#define G_RX_PACKETS  7   /* CRC-good (software CRC) packets                    */
#define G_RX_BIND     8   /* RX role: bind pkts | TX role: telemetry with our id */
#define G_RX_DATA     9   /* RX role: data pkts                                  */
#define G_RX_ODD      10  /* wrong length / header / software CRC                */
#define G_RX_CH0_US   11  /* RX role: ch1 us     | TX role: A1                   */
#define G_RX_CH1_US   12  /* RX role: ch2 us     | TX role: A2                   */
#define G_RX_HOPNO    13  /* RX role: hop_no byte| TX role: RX-reported RSSI     */
#define G_RX_RSSI     14  /* our RSSI, quarter-dBm                               */
#define G_LBT_BUSY    15  /* TX role: packets sent at reduced power (LBT busy)   */
#endif
