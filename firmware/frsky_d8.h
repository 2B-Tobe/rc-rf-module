/*
 * frsky_d8.h — FrSky D8 (ACCST "D" series) transmitter for the rc_module_trx
 * EFR32MG24 module, on the `frsky_d8` PHY (2GFSK 31 kbps, dev 31 kHz,
 * 2404 MHz + 300 kHz * ch, preamble 0xAA x4, sync 0xD391, fixed 18 bytes,
 * HW CRC-16 0x8005 seed 0xFFFF).  RAIL 3 (sl_rail_*) API.   Version 1.
 *
 * Protocol facts from Multi-Module FrSkyD_cc2500.ino / FrSkyDVX_common.ino:
 *   bind : 18-byte packets on channel 0 every 9 ms, ten of them carry the
 *          47-entry hop table (5 per packet), then repeat
 *   data : 9 ms slots, counter 0..187, channel = hop[counter % 47];
 *          slots 0,1,2 transmit, slot 3 is the receiver's telemetry slot,
 *          where we listen for its 18-byte reply (id, A1, A2, RSSI, hub data)
 *   frame: 11 id3 id2 counter 00 id1 | ch0..3 lo | nib(1,0) nib(3,2) |
 *          ch4..7 lo | nib(5,4) nib(7,6)     (12-bit value = us * 1.5)
 *
 * Knobs at the top of frsky_d8.c:
 *   frsky_start(..., rx_role=true) makes the board a D8 receiver (app: jumper
 *   PC00 + F/S PB03 low at boot): binds to the first TX heard on channel 0
 *   (kept in flash), follows the counter/hop cycle, answers telemetry in slot 3
 *   and outputs 8-channel PPM on PC03 (300 us low pulses, idle high, 22.5 ms).
 *   Servo PWM: ch1..8 on PA07 PB04 PD04 PD05 PA00 PA08 PA09 PB05 (sequential pulses; dual build, servo_pins.h).
 *   Bind is stored in the USERDATA flash page (rx_store.c): the receiver comes up
 *   bound; hold BIND (PB00) at power-up to forget it and bind again.
 *   Failsafe ~1 s after link loss: stored preset if one exists (hold BIND >0.3 s
 *   while linked to store the current positions), else hold (FRSKY_FS_DEFAULT 1 = no pulses).
 *   Diag [2] in RX role: +1 bind stored, +100 per preset stored.
 *   Diag in RX role: [1] 0 bind / 10 bound-searching / 11 tracking,
 *   [3] telemetry replies, [7] packets, [8] bind, [9] data, [10] rejected,
 *   [11..12] ch1/ch2 us, [13] counter, [14] our RSSI, [15] misses.
 *   FRSKY_BIND_PIN         1 = bind while BIND is held at boot; 0 = bind for
 *                          FRSKY_BIND_SLOTS after boot (PB00 not verified yet)
 *   FRSKY_DATA_POWER_DDBM  flight power, deci-dBm
 */
#ifndef FRSKY_D8_H
#define FRSKY_D8_H

#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

typedef enum { FRSKY_PHASE_BIND = 0, FRSKY_PHASE_DATA, FRSKY_PHASE_RX } frsky_phase_t;

/* ch_us: 8 channel pulse widths in microseconds (900..2100), read live.
 * bind_held: BIND input state at boot (true = held). rx_role: receiver role. */
void          frsky_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool bind_held, bool rx_role);
/* Main loop. bind_held: current BIND input; range_low: RANGE input active. */
void          frsky_service(bool bind_held, bool range_low);
/* Transmitter, every loop: true = BIND on in the serial frames. While it is, 1.5 s of bind and 1.5 s of
 * flying data take turns. Jumper mode passes false: 1.5 s of bind at power-up, as before. */
void          frsky_bind(bool held);
frsky_phase_t frsky_phase(void);
/* True when the receiver's telemetry has been heard within the last 500 ms. */
bool          frsky_link_up(void);
int8_t        frsky_tele_rssi_dbm(void);   /* TX: RSSI of the last telemetry packet heard, dBm (RX: of the last data packet) */

/* From the RAIL event callback (ISR): raw event mask. */
void          frsky_on_events(sl_rail_events_t events);
/* From the main loop when the callback held an RX packet. */
void          frsky_rx_process(void);
void          frsky_on_rx_held(void);   /* RX: from the RAIL event handler, after a packet is held */

/* Diagnostics block, one SWD read: python -m pyocd commander ... "read32 <frsky_diag> 64" */
extern volatile int32_t frsky_diag[16];
/* Packets that failed the CC2500 CRC (noise, another system), and good packets the radio's own CRC agreed with. */
uint32_t frsky_crc_bad(void);
void frsky_tx_stats(uint32_t *chg, uint32_t *step_us, uint32_t *resync, uint32_t *sched_fail);   /* TX: value changes, biggest step, chain restarts, refused scheduled sends */
uint32_t frsky_hwcrc_agree(void);
/* The last packet heard: its length, the radio's CRC verdict, its CRC bytes and the CC2500 CRC of bytes 0..17. */
void frsky_crc_last(uint16_t *n, uint8_t *hw, uint16_t *rx, uint16_t *sw);
const uint8_t *frsky_last_pkt(void);   /* the last packet's 20 bytes */
#define F_ROLE        0   /* 0 tx, 1 rx                                          */
#define F_PHASE       1   /* frsky_phase_t                                       */
#define F_BIND_SENT   2   /* bind packets started                                */
#define F_DATA_SENT   3   /* data packets started                                */
#define F_TX_OK       4   /* TX_PACKET_SENT events                               */
#define F_TX_ERR      5   /* TX completion errors + refused starts               */
#define F_CFG_ERR     6   /* 1 chans 2 fixlen 8 startrx                          */
#define F_RX_PACKETS  7   /* CRC-good 18-byte packets received (any)             */
#define F_RX_BIND     8   /* RX role: bind packets   | TX role: telemetry w/ our id */
#define F_RX_DATA     9   /* RX role: data packets   | TX role: (unused)         */
#define F_RX_ODD      10  /* packets with unexpected length/format/id            */
#define F_RX_CH0_US   11  /* RX role: decoded ch0 us | TX role: telemetry A1     */
#define F_RX_CH1_US   12  /* RX role: decoded ch1 us | TX role: telemetry A2     */
#define F_RX_COUNTER  13  /* RX role: counter byte   | TX role: RX-reported RSSI */
#define F_RX_RSSI     14  /* our RSSI of the last packet, quarter-dBm            */
#define F_EV_SYNC     15  /* sync-detect events                                  */

#endif /* FRSKY_D8_H */
